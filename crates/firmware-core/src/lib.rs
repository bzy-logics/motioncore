//! Target-independent MCU runtime for MotionCore firmware.
//!
//! [`McuCore`] handles protocol dispatch, the communication watchdog, fault
//! latching and status reporting, and composes one
//! [`StepperAxis`](motioncore_firmware_stepper::StepperAxis) per channel.
//! Hardware is reached only through [`Hal`] (`firmware-hal`), so the same code
//! runs on real boards (`boards/*`, driven by a hardware timer) and inside the
//! simulator (`crates/simulator`, driven by a simulated clock). That shared
//! runtime is what gives virtual/physical behavioural parity.
#![cfg_attr(not(test), no_std)]
#![forbid(unsafe_code)]

use heapless::{Deque, Vec};
pub use motioncore_firmware_hal::{Clock, Hal};
use motioncore_firmware_stepper::{AxisEvent, StepperAxis};
use motioncore_protocol::{FaultCode, HostFrame, HostMsg, Identity, McuFrame, McuMsg, NackReason, Status, MAX_AXES, PROTOCOL_VERSION};

/// Default lateness tolerance for boards with a fast hardware tick.
pub const DEFAULT_LATE_TOLERANCE_US: u64 = 5_000;
/// Default communication watchdog timeout.
pub const DEFAULT_WATCHDOG_MS: u32 = 500;
/// Outgoing message buffer depth.
const OUTBOX: usize = 16;

/// Static board description baked into the firmware image.
#[derive(Debug, Clone)]
pub struct BoardInfo {
    pub target: &'static str,
    pub board: &'static str,
    pub firmware_version: (u8, u8, u8),
    pub build_hash: u32,
    pub capabilities: u32,
    /// Segments that become due more than this late fault with `SegmentLate`.
    /// Must exceed the worst-case tick period of the target.
    pub late_tolerance_us: u64,
}

/// The MCU runtime. `AXES` = number of step channels, `Q` = per-axis queue depth.
pub struct McuCore<const AXES: usize, const Q: usize> {
    info: BoardInfo,
    axes: [StepperAxis<Q>; AXES],
    fault: Option<FaultCode>,
    /// Axis the latched fault was reported on, if any.
    fault_axis: Option<u8>,
    watchdog_us: u64,
    last_rx_us: u64,
    /// Time of the previous [`McuCore::tick`], to detect stalled ticks.
    last_tick_us: Option<u64>,
    tx_seq: u32,
    outbox: Deque<McuMsg, OUTBOX>,
}

impl<const AXES: usize, const Q: usize> McuCore<AXES, Q> {
    pub fn new(info: BoardInfo) -> Self {
        assert!(AXES <= MAX_AXES && Q <= u8::MAX as usize);
        Self {
            info,
            axes: core::array::from_fn(|i| StepperAxis::new(i as u8)),
            fault: None,
            fault_axis: None,
            watchdog_us: DEFAULT_WATCHDOG_MS as u64 * 1000,
            last_rx_us: 0,
            last_tick_us: None,
            tx_seq: 0,
            outbox: Deque::new(),
        }
    }

    pub fn fault(&self) -> Option<FaultCode> {
        self.fault
    }

    pub fn position(&self, axis: usize) -> i32 {
        self.axes[axis].position()
    }

    /// Next frame to transmit, if any.
    pub fn poll_outgoing(&mut self) -> Option<McuFrame> {
        let msg = self.outbox.pop_front()?;
        self.tx_seq = self.tx_seq.wrapping_add(1);
        Some(McuFrame { seq: self.tx_seq, msg })
    }

    fn send(&mut self, msg: McuMsg) {
        if self.outbox.is_full() {
            // Never drop a fault report: evict the oldest non-fault message.
            let Some(idx) = self.outbox.iter().position(|m| !matches!(m, McuMsg::Fault { .. })) else { return };
            let kept: Vec<McuMsg, OUTBOX> = self.outbox.iter().enumerate().filter(|(i, _)| *i != idx).map(|(_, m)| m.clone()).collect();
            self.outbox.clear();
            for m in kept {
                let _ = self.outbox.push_back(m);
            }
        }
        let _ = self.outbox.push_back(msg);
    }

    /// Latch a fault: stop all motion, disable all drives, report once.
    pub fn trip(&mut self, code: FaultCode, axis: Option<u8>, hal: &mut impl Hal) {
        for a in self.axes.iter_mut() {
            a.disable(hal);
        }
        if self.fault.is_none() {
            self.fault = Some(code);
            self.fault_axis = axis;
            self.send(McuMsg::Fault { code, axis });
        }
    }

    /// A latched fault may only be cleared once the hazard behind it is gone,
    /// verified from fresh inputs. Clearing never re-enables a drive.
    fn check_reset_allowed(&mut self, hal: &mut impl Hal) -> Result<(), NackReason> {
        if !hal.interlock_ok() {
            return Err(NackReason::InvalidState);
        }
        if self.axes.iter().any(|a| a.enabled() || a.is_moving()) {
            return Err(NackReason::InvalidState);
        }
        if let (Some(FaultCode::LimitHit), Some(i)) = (self.fault, self.fault_axis) {
            // Still on the switch with no known safe direction: the axis must
            // be moved off it by hand first.
            if let Some(a) = self.axes.get(i as usize) {
                if a.endstop(hal) && !a.endstop_side_known() {
                    return Err(NackReason::InvalidState);
                }
            }
        }
        // CommunicationLost needs no extra check: this command just arrived.
        Ok(())
    }

    pub fn status(&self, now_us: u64, hal: &mut impl Hal) -> Status {
        let depth = self.axes.iter().map(|a| a.queue_depth()).max().unwrap_or(0);
        Status { mcu_time_us: now_us, fault: self.fault, queue_depth: depth as u8, axes: self.axes.iter().map(|a| a.status(hal)).collect() }
    }

    /// Queue a status report (call at the telemetry rate).
    pub fn report_status(&mut self, now_us: u64, hal: &mut impl Hal) {
        let s = self.status(now_us, hal);
        self.send(McuMsg::Status(s));
    }

    // ------------------------------------------------------------------
    // Command handling
    // ------------------------------------------------------------------

    pub fn handle(&mut self, frame: HostFrame, now_us: u64, hal: &mut impl Hal) {
        self.last_rx_us = now_us;
        let seq = frame.seq;
        match self.dispatch(frame.msg, now_us, hal) {
            Ok(true) => self.send(McuMsg::Ack { seq }),
            Ok(false) => {}
            Err(reason) => self.send(McuMsg::Nack { seq, reason }),
        }
    }

    fn axis(&mut self, axis: u8) -> Result<&mut StepperAxis<Q>, NackReason> {
        self.axes.get_mut(axis as usize).ok_or(NackReason::UnknownAxis)
    }

    /// `Ok(true)` → Ack, `Ok(false)` → command produced its own reply.
    fn dispatch(&mut self, msg: HostMsg, now_us: u64, hal: &mut impl Hal) -> Result<bool, NackReason> {
        let faulted = self.fault.is_some();
        let late = self.info.late_tolerance_us;
        match msg {
            HostMsg::Identify => {
                let id = Identity {
                    protocol_version: PROTOCOL_VERSION,
                    firmware_version: self.info.firmware_version,
                    target: heapless::String::try_from(self.info.target).unwrap_or_default(),
                    board: heapless::String::try_from(self.info.board).unwrap_or_default(),
                    build_hash: self.info.build_hash,
                    capabilities: self.info.capabilities,
                    axes: AXES as u8,
                    queue_capacity: Q as u8,
                };
                self.send(McuMsg::Identity(id));
                Ok(false)
            }
            HostMsg::ClockSync { host_time_us } => {
                self.send(McuMsg::ClockSyncReply { host_time_us, mcu_time_us: now_us });
                Ok(false)
            }
            HostMsg::Heartbeat => Ok(false),
            HostMsg::QueryStatus => {
                self.report_status(now_us, hal);
                Ok(false)
            }
            HostMsg::SetWatchdog { timeout_ms } => {
                if !(10..=10_000).contains(&timeout_ms) {
                    return Err(NackReason::InvalidParams);
                }
                self.watchdog_us = timeout_ms as u64 * 1000;
                Ok(true)
            }
            HostMsg::EmergencyStop => {
                self.trip(FaultCode::EmergencyStop, None, hal);
                Ok(true)
            }
            HostMsg::ResetFault => {
                // Drives were disabled when the fault latched and stay disabled.
                self.check_reset_allowed(hal)?;
                self.fault = None;
                self.fault_axis = None;
                Ok(true)
            }
            HostMsg::Disable { axis } => {
                self.axis(axis)?.disable(hal);
                Ok(true)
            }
            _ if faulted => Err(NackReason::Faulted),
            HostMsg::ConfigureAxis(cfg) => self.axis(cfg.axis)?.configure(cfg).map(|_| true),
            HostMsg::Enable { axis } => {
                if !hal.interlock_ok() {
                    self.trip(FaultCode::EmergencyStop, None, hal);
                    return Err(NackReason::Faulted);
                }
                self.axis(axis)?.enable(hal).map(|_| true)
            }
            HostMsg::SetPosition { axis, steps } => self.axis(axis)?.set_position(steps).map(|_| true),
            HostMsg::QueueSegment(seg) => self.axis(seg.axis)?.enqueue(seg, now_us, late).map(|_| true),
            HostMsg::Home(params) => self.axis(params.axis)?.home(params, now_us).map(|_| true),
            HostMsg::Stop => {
                for a in self.axes.iter_mut() {
                    a.stop(now_us);
                }
                Ok(true)
            }
        }
    }

    // ------------------------------------------------------------------
    // Real-time tick
    // ------------------------------------------------------------------

    /// Advance to `now_us`. Must be called periodically; every call emits all
    /// steps that became due.
    ///
    /// A gap between ticks longer than the board's late tolerance while an
    /// axis is moving faults with `SegmentLate` instead of emitting the
    /// backlog as one burst of steps.
    pub fn tick(&mut self, now_us: u64, hal: &mut impl Hal) {
        let gap = self.last_tick_us.map_or(0, |t| now_us.saturating_sub(t));
        self.last_tick_us = Some(now_us);
        if self.fault.is_none() && !hal.interlock_ok() {
            self.trip(FaultCode::EmergencyStop, None, hal);
            return;
        }
        if self.fault.is_none() {
            let any_enabled = self.axes.iter().any(|a| a.enabled());
            if any_enabled && now_us.saturating_sub(self.last_rx_us) > self.watchdog_us {
                self.trip(FaultCode::CommunicationLost, None, hal);
                return;
            }
        }
        if self.fault.is_some() {
            return;
        }
        let late = self.info.late_tolerance_us;
        if gap > late && self.axes.iter().any(|a| a.is_moving()) {
            self.trip(FaultCode::SegmentLate, None, hal);
            return;
        }
        for i in 0..AXES {
            match self.axes[i].tick(now_us, late, hal) {
                Ok(None) => {}
                Ok(Some(AxisEvent::HomingDone { position_steps })) => {
                    self.send(McuMsg::HomingDone { axis: i as u8, position_steps });
                }
                Err(code) => {
                    self.trip(code, Some(i as u8), hal);
                    return;
                }
            }
        }
        if !hal.flush_pulses() {
            self.trip(FaultCode::StepRateExceeded, None, hal);
        }
    }
}

#[cfg(test)]
mod tests;
