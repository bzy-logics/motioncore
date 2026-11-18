//! Step/dir stepper channel: the build-time firmware module that executes
//! motion segments, homing and controlled stops for one axis.
//!
//! Step timing model: [`StepperAxis::tick`] is called at a fixed rate and
//! emits every step that became due since the previous tick, so step-edge
//! jitter is bounded by one tick period. Per-edge timer-compare scheduling is a
//! planned refinement for high step rates.
#![cfg_attr(not(test), no_std)]
#![forbid(unsafe_code)]

use heapless::Deque;
use motioncore_firmware_hal::Hal;
use motioncore_protocol::{AxisConfig, AxisStatus, FaultCode, HomeParams, NackReason, Segment};

/// Something the axis wants reported to the host.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AxisEvent {
    HomingDone { position_steps: i32 },
}

#[derive(Debug, Clone, Copy)]
struct Active {
    seg: Segment,
    /// Steps emitted so far relative to segment start.
    emitted: i32,
}

#[derive(Debug, Clone, Copy)]
struct Homing {
    params: HomeParams,
    acc: f32,
    travelled: u32,
    last_us: u64,
}

#[derive(Debug, Clone, Copy)]
struct Stopping {
    velocity: f32,
    acc: f32,
    last_us: u64,
}

/// One step/dir channel with its own bounded segment queue of depth `Q`.
#[derive(Debug)]
pub struct StepperAxis<const Q: usize> {
    channel: u8,
    config: Option<AxisConfig>,
    position: i32,
    enabled: bool,
    queue: Deque<Segment, Q>,
    /// Position the axis will be at once every queued segment has executed.
    queued_end_position: i32,
    active: Option<Active>,
    homing: Option<Homing>,
    stopping: Option<Stopping>,
    velocity: f32,
    /// Side (`true` = positive) on which the endstop was found while homing.
    endstop_side: Option<bool>,
}

impl<const Q: usize> StepperAxis<Q> {
    pub fn new(channel: u8) -> Self {
        Self {
            channel,
            config: None,
            position: 0,
            enabled: false,
            queue: Deque::new(),
            queued_end_position: 0,
            active: None,
            homing: None,
            stopping: None,
            velocity: 0.0,
            endstop_side: None,
        }
    }

    pub fn channel(&self) -> u8 {
        self.channel
    }
    pub fn config(&self) -> Option<AxisConfig> {
        self.config
    }
    pub fn position(&self) -> i32 {
        self.position
    }
    pub fn enabled(&self) -> bool {
        self.enabled
    }
    pub fn is_moving(&self) -> bool {
        self.active.is_some() || self.homing.is_some() || self.stopping.is_some()
    }
    /// Queued plus executing segments.
    pub fn queue_depth(&self) -> usize {
        self.queue.len() + self.active.is_some() as usize
    }

    /// `true` if homing established on which side the endstop is, so steps
    /// away from a triggered endstop are permitted.
    pub fn endstop_side_known(&self) -> bool {
        self.endstop_side.is_some()
    }

    fn busy(&self) -> bool {
        self.is_moving() || !self.queue.is_empty()
    }

    /// Endstop level after optional inversion.
    pub fn endstop(&self, hal: &mut impl Hal) -> bool {
        let raw = hal.endstop_raw(self.channel);
        match self.config {
            Some(c) if c.invert_endstop => !raw,
            _ => raw,
        }
    }

    pub fn status(&self, hal: &mut impl Hal) -> AxisStatus {
        AxisStatus {
            position_steps: self.position,
            velocity: self.velocity,
            enabled: self.enabled,
            moving: self.is_moving(),
            homing: self.homing.is_some(),
            endstop: self.endstop(hal),
        }
    }

    // ------------------------------------------------------------------
    // Commands
    // ------------------------------------------------------------------

    pub fn configure(&mut self, cfg: AxisConfig) -> Result<(), NackReason> {
        if cfg.min_steps >= cfg.max_steps || !(cfg.max_rate.is_finite() && cfg.max_rate > 0.0) || !(cfg.stop_decel.is_finite() && cfg.stop_decel > 0.0) {
            return Err(NackReason::InvalidParams);
        }
        if self.busy() {
            return Err(NackReason::Busy);
        }
        self.config = Some(cfg);
        Ok(())
    }

    pub fn enable(&mut self, hal: &mut impl Hal) -> Result<(), NackReason> {
        if self.config.is_none() {
            return Err(NackReason::InvalidState);
        }
        self.enabled = true;
        hal.set_enable(self.channel, true);
        Ok(())
    }

    /// Immediately stop and de-energise (no deceleration).
    pub fn disable(&mut self, hal: &mut impl Hal) {
        self.halt();
        self.enabled = false;
        hal.set_enable(self.channel, false);
    }

    /// Discard all motion state without touching the driver enable.
    pub fn halt(&mut self) {
        self.queue.clear();
        self.active = None;
        self.homing = None;
        self.stopping = None;
        self.velocity = 0.0;
        self.queued_end_position = self.position;
    }

    pub fn set_position(&mut self, steps: i32) -> Result<(), NackReason> {
        if self.busy() {
            return Err(NackReason::Busy);
        }
        self.position = steps;
        self.queued_end_position = steps;
        Ok(())
    }

    /// Validate and append a segment.
    pub fn enqueue(&mut self, seg: Segment, now_us: u64, late_tolerance_us: u64) -> Result<(), NackReason> {
        let cfg = self.config.ok_or(NackReason::InvalidState)?;
        if !self.enabled || self.homing.is_some() || self.stopping.is_some() {
            return Err(NackReason::InvalidState);
        }
        if self.queue.is_full() {
            return Err(NackReason::QueueFull);
        }
        let v1 = seg.end_velocity();
        let limit = cfg.max_rate * 1.01 + 1.0;
        if seg.duration_us == 0 || !seg.v0.is_finite() || !seg.accel.is_finite() || seg.v0.abs() > limit || v1.abs() > limit {
            return Err(NackReason::InvalidParams);
        }
        // The integer step count must agree with the kinematics.
        let t = seg.duration_us as f32 * 1e-6;
        let ideal = seg.v0 * t + 0.5 * seg.accel * t * t;
        if (ideal - seg.steps as f32).abs() > 1.5 + ideal.abs() * 1e-4 {
            return Err(NackReason::InvalidParams);
        }
        // Must start in the future (with tolerance) and after the queue tail.
        if seg.start_us + late_tolerance_us < now_us {
            return Err(NackReason::InvalidParams);
        }
        if let Some(tail) = self.queue.back().copied().or(self.active.map(|x| x.seg)) {
            if seg.start_us < tail.end_us() {
                return Err(NackReason::InvalidParams);
            }
        }
        let end = self.queued_end_position as i64 + seg.steps as i64;
        if end < cfg.min_steps as i64 || end > cfg.max_steps as i64 {
            return Err(NackReason::SoftLimit);
        }
        self.queued_end_position = end as i32;
        let _ = self.queue.push_back(seg);
        Ok(())
    }

    pub fn home(&mut self, params: HomeParams, now_us: u64) -> Result<(), NackReason> {
        let cfg = self.config.ok_or(NackReason::InvalidState)?;
        if !self.enabled {
            return Err(NackReason::InvalidState);
        }
        if self.busy() {
            return Err(NackReason::Busy);
        }
        if !(params.speed.is_finite() && params.speed > 0.0 && params.speed <= cfg.max_rate) || params.max_travel_steps == 0 {
            return Err(NackReason::InvalidParams);
        }
        self.homing = Some(Homing { params, acc: 0.0, travelled: 0, last_us: now_us });
        Ok(())
    }

    /// Controlled stop: drop queued motion and decelerate at `stop_decel`.
    pub fn stop(&mut self, now_us: u64) {
        self.queue.clear();
        self.homing = None;
        let v = self.velocity;
        self.active = None;
        self.stopping = if v.abs() > 0.5 {
            Some(Stopping { velocity: v, acc: 0.0, last_us: now_us })
        } else {
            self.velocity = 0.0;
            None
        };
        self.queued_end_position = self.position;
    }

    // ------------------------------------------------------------------
    // Real-time tick
    // ------------------------------------------------------------------

    /// Advance to `now_us`, emitting all due steps. A returned fault must be
    /// latched by the caller (which disables every drive).
    pub fn tick(&mut self, now_us: u64, late_tolerance_us: u64, hal: &mut impl Hal) -> Result<Option<AxisEvent>, FaultCode> {
        let Some(cfg) = self.config else { return Ok(None) };

        if let Some(mut h) = self.homing {
            let dt = now_us.saturating_sub(h.last_us) as f32 * 1e-6;
            h.last_us = now_us;
            h.acc += h.params.speed * dt;
            let dir = h.params.positive;
            loop {
                if self.endstop(hal) {
                    self.homing = None;
                    self.velocity = 0.0;
                    self.position = h.params.home_steps;
                    self.queued_end_position = self.position;
                    self.endstop_side = Some(dir);
                    return Ok(Some(AxisEvent::HomingDone { position_steps: h.params.home_steps }));
                }
                if h.acc < 1.0 {
                    break;
                }
                if h.travelled >= h.params.max_travel_steps {
                    return Err(FaultCode::HomingFailed);
                }
                h.acc -= 1.0;
                h.travelled += 1;
                self.step(dir, cfg, hal);
            }
            self.velocity = if dir { h.params.speed } else { -h.params.speed };
            self.homing = Some(h);
            return Ok(None);
        }

        if let Some(mut s) = self.stopping {
            let dt = now_us.saturating_sub(s.last_us) as f32 * 1e-6;
            s.last_us = now_us;
            let dv = cfg.stop_decel * dt;
            let v_prev = s.velocity;
            let v_new = if v_prev > 0.0 { (v_prev - dv).max(0.0) } else { (v_prev + dv).min(0.0) };
            s.acc += 0.5 * (v_prev + v_new) * dt;
            while s.acc.abs() >= 1.0 {
                let fwd = s.acc > 0.0;
                s.acc -= if fwd { 1.0 } else { -1.0 };
                self.check_step_allowed(fwd, cfg, hal)?;
                self.step(fwd, cfg, hal);
            }
            s.velocity = v_new;
            self.velocity = v_new;
            self.stopping = if v_new == 0.0 { None } else { Some(s) };
            self.queued_end_position = self.position;
            return Ok(None);
        }

        loop {
            if self.active.is_none() {
                match self.queue.front() {
                    Some(seg) if seg.start_us <= now_us => {
                        if now_us - seg.start_us > late_tolerance_us {
                            return Err(FaultCode::SegmentLate);
                        }
                        let seg = self.queue.pop_front().unwrap();
                        self.active = Some(Active { seg, emitted: 0 });
                    }
                    _ => {
                        self.velocity = 0.0;
                        return Ok(None);
                    }
                }
            }
            let Active { seg, emitted } = self.active.unwrap();
            let elapsed = now_us.saturating_sub(seg.start_us);
            let done = elapsed >= seg.duration_us as u64;
            let target = if done {
                seg.steps
            } else {
                let t = elapsed as f32 * 1e-6;
                let p = round_half_away(seg.v0 * t + 0.5 * seg.accel * t * t) as i32;
                if seg.steps >= 0 {
                    p.clamp(0, seg.steps)
                } else {
                    p.clamp(seg.steps, 0)
                }
            };
            let mut emitted_now = emitted;
            while emitted_now != target {
                let fwd = target > emitted_now;
                self.check_step_allowed(fwd, cfg, hal)?;
                self.step(fwd, cfg, hal);
                emitted_now += if fwd { 1 } else { -1 };
            }
            if !done {
                self.active = Some(Active { seg, emitted: emitted_now });
                self.velocity = seg.v0 + seg.accel * (elapsed as f32 * 1e-6);
                return Ok(None);
            }
            self.active = None;
            let next_contiguous = self.queue.front().map(|n| n.start_us <= seg.end_us()).unwrap_or(false);
            if !seg.last && !next_contiguous {
                return Err(FaultCode::QueueUnderrun);
            }
            if seg.last {
                self.velocity = 0.0;
            }
            if !next_contiguous {
                return Ok(None);
            }
            // Contiguous continuation: never treat it as late.
            if let Some(n) = self.queue.front_mut() {
                if n.start_us < seg.end_us() {
                    n.start_us = seg.end_us();
                }
            }
        }
    }

    fn check_step_allowed(&self, forward: bool, cfg: AxisConfig, hal: &mut impl Hal) -> Result<(), FaultCode> {
        let next = self.position as i64 + if forward { 1 } else { -1 };
        if next < cfg.min_steps as i64 || next > cfg.max_steps as i64 {
            return Err(FaultCode::SoftLimit);
        }
        if self.endstop(hal) {
            // Moving away from the endstop discovered during homing is fine.
            let toward = match self.endstop_side {
                Some(side) => side == forward,
                None => true,
            };
            if toward {
                return Err(FaultCode::LimitHit);
            }
        }
        Ok(())
    }

    fn step(&mut self, forward: bool, cfg: AxisConfig, hal: &mut impl Hal) {
        self.position += if forward { 1 } else { -1 };
        hal.step(self.channel, forward ^ cfg.invert_dir);
    }
}

/// `f32::round` is not available in `core`; round half away from zero.
pub fn round_half_away(x: f32) -> f32 {
    let t = x as i64 as f32; // truncation toward zero
    let frac = x - t;
    if frac >= 0.5 {
        t + 1.0
    } else if frac <= -0.5 {
        t - 1.0
    } else {
        t
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn rounding() {
        assert_eq!(round_half_away(1.49), 1.0);
        assert_eq!(round_half_away(1.5), 2.0);
        assert_eq!(round_half_away(-1.5), -2.0);
        assert_eq!(round_half_away(-0.2), 0.0);
    }

    struct NoHal;
    impl Hal for NoHal {
        fn step(&mut self, _: u8, _: bool) {}
        fn set_enable(&mut self, _: u8, _: bool) {}
        fn endstop_raw(&mut self, _: u8) -> bool {
            false
        }
    }

    #[test]
    fn unconfigured_axis_rejects_everything() {
        let mut a: StepperAxis<4> = StepperAxis::new(0);
        assert_eq!(a.enable(&mut NoHal), Err(NackReason::InvalidState));
        let seg = Segment { axis: 0, start_us: 10, duration_us: 10, steps: 0, v0: 0.0, accel: 0.0, last: true };
        assert_eq!(a.enqueue(seg, 0, 5_000), Err(NackReason::InvalidState));
        assert_eq!(a.tick(100, 5_000, &mut NoHal), Ok(None));
    }
}
