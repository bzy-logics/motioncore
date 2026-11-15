//! End-to-end tests of the RP2040 output path on the host: the real shared
//! firmware core (`McuCore` + `StepperAxis`) drives a HAL that does exactly
//! what `boards/rp2040` does (buffer steps per tick, encode PIO commands,
//! refuse to queue while the previous window is still pending, abort on
//! disable), with the cycle-accurate PIO model standing in for the six state
//! machines. The emulated STEP/DIR pins are the ground truth for step counts,
//! directions and pulse timing.

use motioncore_firmware_core::{BoardInfo, Hal, McuCore};
use motioncore_protocol::{caps, AxisConfig, FaultCode, HomeParams, HostFrame, HostMsg, McuMsg, Segment};

use crate::emu::{check_constraints, pulses, Sm};
use crate::*;

/// Same values as `boards/rp2040/src/board.rs`.
const T: Timing = Timing { sm_clock_hz: 5_000_000, high_cycles: 10, min_low_cycles: 10, dir_setup_cycles: 10, window_budget_pct: 90 };
const TICK_US: u64 = 200;
const CYCLES_PER_TICK: u64 = TICK_US * 5;
const LATE_TOLERANCE_US: u64 = 10 * TICK_US;
const AXES: usize = 6;
/// Joined TX FIFO depth.
const FIFO: usize = 8;

struct PioHal {
    sms: Vec<Sm>,
    runs: [StepRuns; AXES],
    enabled: [bool; AXES],
    /// Endstop pressed while the axis' physical position <= this.
    endstop_at: [Option<i64>; AXES],
    /// Cycle at which each axis was last disabled (output inhibited).
    disabled_at: [Option<u64>; AXES],
}

impl PioHal {
    fn new() -> Self {
        let p = program(&T).unwrap();
        Self {
            sms: (0..AXES).map(|_| Sm::new(&p)).collect(),
            runs: [StepRuns::new(); AXES],
            enabled: [false; AXES],
            endstop_at: [None; AXES],
            disabled_at: [None; AXES],
        }
    }
}

impl Hal for PioHal {
    fn step(&mut self, axis: u8, forward: bool) {
        assert!(self.enabled[axis as usize], "step requested on disabled axis {axis}");
        self.runs[axis as usize].push(forward);
    }
    fn set_enable(&mut self, axis: u8, enabled: bool) {
        let i = axis as usize;
        if !enabled {
            self.runs[i].clear();
            self.sms[i].abort();
            self.disabled_at[i] = Some(self.sms[i].cycle);
        }
        self.enabled[i] = enabled;
    }
    fn endstop_raw(&mut self, axis: u8) -> bool {
        let i = axis as usize;
        matches!(self.endstop_at[i], Some(p) if self.sms[i].position <= p)
    }
    fn flush_pulses(&mut self) -> bool {
        let mut ok = true;
        for i in 0..AXES {
            if self.runs[i].is_empty() {
                continue;
            }
            if !self.sms[i].fifo.is_empty() {
                self.runs[i].clear();
                ok = false;
                continue;
            }
            match self.runs[i].take_commands(&T, TICK_US as u32) {
                Ok(c) => {
                    for &w in c.words() {
                        if self.sms[i].fifo.len() < FIFO {
                            self.sms[i].fifo.push_back(w);
                        } else {
                            ok = false;
                        }
                    }
                }
                Err(_) => ok = false,
            }
        }
        ok
    }
}

struct Rig {
    core: McuCore<AXES, 32>,
    hal: PioHal,
    now: u64,
    seq: u32,
    heartbeats: bool,
}

fn cfg(axis: u8) -> AxisConfig {
    AxisConfig { axis, min_steps: -100_000, max_steps: 100_000, max_rate: 50_000.0, stop_decel: 200_000.0, invert_dir: false, invert_endstop: false }
}

impl Rig {
    fn new() -> Self {
        let core = McuCore::new(BoardInfo {
            target: "rp2040",
            board: "rp2040-pico",
            firmware_version: (0, 2, 0),
            build_hash: 0,
            capabilities: caps::STEPPER_STEP_DIR | caps::ENDSTOP | caps::WATCHDOG,
            late_tolerance_us: LATE_TOLERANCE_US,
        });
        let mut r = Rig { core, hal: PioHal::new(), now: 1_000, seq: 0, heartbeats: true };
        for a in 0..AXES as u8 {
            r.cmd(HostMsg::ConfigureAxis(cfg(a)));
            r.cmd(HostMsg::Enable { axis: a });
        }
        r.drain();
        assert!(r.hal.enabled.iter().all(|&e| e));
        r
    }

    fn cmd(&mut self, msg: HostMsg) {
        self.seq += 1;
        self.core.handle(HostFrame { seq: self.seq, msg }, self.now, &mut self.hal);
    }

    fn drain(&mut self) -> Vec<McuMsg> {
        let mut v = Vec::new();
        while let Some(f) = self.core.poll_outgoing() {
            v.push(f.msg);
        }
        v
    }

    /// One tick interrupt, then one window of PIO time.
    fn tick(&mut self) {
        self.now += TICK_US;
        if self.heartbeats && self.now % 50_000 < TICK_US {
            self.cmd(HostMsg::Heartbeat);
        }
        self.core.tick(self.now, &mut self.hal);
        self.clock_pio(CYCLES_PER_TICK);
    }

    fn clock_pio(&mut self, cycles: u64) {
        for sm in &mut self.hal.sms {
            for _ in 0..cycles {
                sm.clock();
            }
        }
    }

    fn run_for(&mut self, us: u64) {
        let end = self.now + us;
        while self.now < end {
            self.tick();
        }
    }

    fn queue(&mut self, segs: Vec<Segment>) {
        for s in segs {
            self.cmd(HostMsg::QueueSegment(s));
            let reply = self.drain();
            assert!(reply.iter().any(|m| matches!(m, McuMsg::Ack { .. })), "segment rejected: {reply:?}");
        }
    }

    fn rising_edges_after(&self, axis: usize, cycle: u64) -> usize {
        pulses(&self.hal.sms[axis].edges).iter().filter(|p| p.rise > cycle).count()
    }
}

/// Constant-velocity move of `n` contiguous segments.
fn const_move(axis: u8, start_us: u64, steps_per_seg: i32, n: usize, seg_us: u32) -> Vec<Segment> {
    let v = steps_per_seg as f32 / (seg_us as f32 * 1e-6);
    (0..n)
        .map(|k| Segment {
            axis,
            start_us: start_us + k as u64 * seg_us as u64,
            duration_us: seg_us,
            steps: steps_per_seg,
            v0: v,
            accel: 0.0,
            last: k == n - 1,
        })
        .collect()
}

#[test]
fn six_axes_exact_step_counts_both_directions_and_timing() {
    let mut r = Rig::new();
    let start = r.now + 2_000;
    // steps per 10 ms segment, per axis: 500 .. 8000 steps/s, mixed signs.
    let per_seg = [5, -8, 12, -3, 80, -40];
    for (a, &s) in per_seg.iter().enumerate() {
        r.queue(const_move(a as u8, start, s, 10, 10_000));
    }
    r.run_for(130_000);
    assert_eq!(r.core.fault(), None);
    for (a, &s) in per_seg.iter().enumerate() {
        let expected = s as i64 * 10;
        assert_eq!(r.hal.sms[a].position, expected, "axis {a} physical steps");
        assert_eq!(r.core.position(a) as i64, expected, "axis {a} firmware position");
        check_constraints(&T, &r.hal.sms[a].edges);
    }
}

#[test]
fn direction_reversal_mid_move() {
    let mut r = Rig::new();
    let start = r.now + 2_000;
    let mut segs = const_move(0, start, 40, 3, 10_000);
    segs.iter_mut().for_each(|s| s.last = false);
    let mut back = const_move(0, start + 30_000, -40, 3, 10_000);
    back.last_mut().unwrap().last = true;
    segs.extend(back);
    r.queue(segs);
    r.run_for(80_000);
    assert_eq!(r.core.fault(), None);
    assert_eq!(r.hal.sms[0].position, 0);
    let dirs: Vec<bool> = pulses(&r.hal.sms[0].edges).iter().map(|p| p.dir).collect();
    assert_eq!(dirs.iter().filter(|&&d| d).count(), 120);
    assert_eq!(dirs.iter().filter(|&&d| !d).count(), 120);
    check_constraints(&T, &r.hal.sms[0].edges);
}

#[test]
fn synchronized_segment_start_across_six_axes() {
    let mut r = Rig::new();
    let start = r.now + 2_000;
    for a in 0..AXES as u8 {
        r.queue(const_move(a, start, 20, 2, 10_000));
    }
    r.run_for(30_000);
    let firsts: Vec<u64> = (0..AXES).map(|a| pulses(&r.hal.sms[a].edges)[0].rise).collect();
    let spread = firsts.iter().max().unwrap() - firsts.iter().min().unwrap();
    // Same tick for every axis; the host test has no push skew, so equal.
    assert!(spread < CYCLES_PER_TICK, "first steps spread over {spread} cycles: {firsts:?}");
}

#[test]
fn queue_underrun_faults_and_inhibits_output() {
    let mut r = Rig::new();
    let mut segs = const_move(0, r.now + 2_000, 40, 2, 10_000);
    segs.last_mut().unwrap().last = false; // promises more, never sent
    r.queue(segs);
    r.run_for(40_000);
    assert_eq!(r.core.fault(), Some(FaultCode::QueueUnderrun));
    assert!(!r.hal.enabled[0]);
    let at = r.hal.disabled_at[0].unwrap();
    r.run_for(10_000);
    assert_eq!(r.rising_edges_after(0, at), 0, "no pulses after the fault");
}

#[test]
fn delayed_tick_faults_without_a_burst() {
    let mut r = Rig::new();
    r.queue(const_move(0, r.now + 2_000, 40, 10, 10_000));
    r.run_for(20_000);
    let before = r.hal.sms[0].position;
    // The tick interrupt is held off for 3 ms while PIO keeps running.
    r.now += 3_000;
    r.clock_pio(3_000 * 5);
    r.tick();
    assert_eq!(r.core.fault(), Some(FaultCode::SegmentLate));
    r.run_for(5_000);
    // At most the window queued before the stall was emitted.
    let extra = r.hal.sms[0].position - before;
    assert!(extra <= T.max_steps_per_window(TICK_US as u32) as i64, "burst of {extra} steps");
    assert!(!r.hal.enabled[0]);
}

#[test]
fn endstop_stops_motion_within_one_window() {
    let mut r = Rig::new();
    r.hal.endstop_at[0] = Some(-15);
    r.queue(const_move(0, r.now + 2_000, -20, 3, 10_000));
    r.run_for(40_000);
    assert_eq!(r.core.fault(), Some(FaultCode::LimitHit));
    // The firmware sees the physical switch one window late at most.
    let pos = r.hal.sms[0].position;
    assert!((-17..=-15).contains(&pos), "stopped at {pos}");
}

#[test]
fn homing_completes_on_the_physical_switch() {
    let mut r = Rig::new();
    r.hal.endstop_at[2] = Some(-300);
    r.cmd(HostMsg::Home(HomeParams { axis: 2, positive: false, speed: 2_000.0, max_travel_steps: 1_000, home_steps: -5_000 }));
    r.drain();
    r.run_for(300_000);
    assert_eq!(r.core.fault(), None);
    assert!(r.drain().iter().any(|m| matches!(m, McuMsg::HomingDone { axis: 2, position_steps: -5_000 })));
    assert_eq!(r.core.position(2), -5_000);
    let pos = r.hal.sms[2].position;
    assert!((-302..=-300).contains(&pos), "homed at physical {pos}");
    check_constraints(&T, &r.hal.sms[2].edges);
}

#[test]
fn emergency_stop_during_motion_cuts_pulses_immediately() {
    let mut r = Rig::new();
    for a in 0..AXES as u8 {
        r.queue(const_move(a, r.now + 2_000, 80, 10, 10_000));
    }
    r.run_for(20_000);
    assert!(r.hal.sms.iter().all(|s| s.position > 0));
    r.cmd(HostMsg::EmergencyStop);
    let cut = r.hal.sms[0].cycle;
    r.run_for(20_000);
    assert_eq!(r.core.fault(), Some(FaultCode::EmergencyStop));
    for a in 0..AXES {
        assert!(!r.hal.enabled[a]);
        assert_eq!(r.rising_edges_after(a, cut), 0, "axis {a} pulsed after e-stop");
        assert!(!r.hal.sms[a].step, "STEP left high");
    }
}

#[test]
fn watchdog_timeout_during_motion_stops_output() {
    let mut r = Rig::new();
    r.queue(const_move(0, r.now + 2_000, 60, 30, 30_000)); // 900 ms of motion
    r.heartbeats = false;
    r.run_for(700_000);
    assert_eq!(r.core.fault(), Some(FaultCode::CommunicationLost));
    let at = r.hal.disabled_at[0].unwrap();
    r.run_for(20_000);
    assert_eq!(r.rising_edges_after(0, at), 0);
}

#[test]
fn rate_beyond_pulse_timing_faults_instead_of_bursting() {
    let mut r = Rig::new();
    r.cmd(HostMsg::ConfigureAxis(AxisConfig { max_rate: 400_000.0, ..cfg(1) }));
    // 300 k steps/s = 60 per 200 µs window; the timing allows 44.
    r.queue(const_move(1, r.now + 2_000, 3_000, 2, 10_000));
    r.run_for(10_000);
    assert_eq!(r.core.fault(), Some(FaultCode::StepRateExceeded));
    assert!(r.hal.sms[1].position < 60, "no window of more than the allowed steps");
    check_constraints(&T, &r.hal.sms[1].edges);
}
