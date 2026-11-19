//! Message definitions shared by host and MCU.
//!
//! Units on the wire are always MCU-native: **steps**, **microseconds** of the
//! MCU clock, steps/s and steps/s². Conversion from physical joint units
//! (rad, m, deg, mm) happens on the host only.

use heapless::{String, Vec};
use serde::{Deserialize, Serialize};

use crate::MAX_AXES;

/// Capability bit flags reported by the MCU in [`Identity::capabilities`].
pub mod caps {
    pub const STEPPER_STEP_DIR: u32 = 1 << 0;
    pub const ENDSTOP: u32 = 1 << 1;
    pub const WATCHDOG: u32 = 1 << 2;
    pub const ENCODER_QUADRATURE: u32 = 1 << 3;
    pub const GPIO_INPUT: u32 = 1 << 4;
    pub const GPIO_OUTPUT: u32 = 1 << 5;
    pub const SERVO_PWM: u32 = 1 << 6;

    /// Bench test build: a spare input tied to GND makes the firmware panic
    /// (Cargo feature `bench-panic`). Never in production firmware.
    pub const BENCH_PANIC: u32 = 1 << 24;
    /// Bench test build: a spare output is high while the motion tick
    /// interrupt runs (Cargo feature `bench-isr-probe`).
    pub const BENCH_ISR_PROBE: u32 = 1 << 25;
    /// Every bench-build bit. A firmware reporting any of them is a test build.
    pub const BENCH_MASK: u32 = BENCH_PANIC | BENCH_ISR_PROBE;

    /// Human-readable capability names.
    pub const NAMES: [(u32, &str); 9] = [
        (STEPPER_STEP_DIR, "stepper.step_dir"),
        (ENDSTOP, "endstop"),
        (WATCHDOG, "watchdog"),
        (ENCODER_QUADRATURE, "encoder.quadrature"),
        (GPIO_INPUT, "gpio.input"),
        (GPIO_OUTPUT, "gpio.output"),
        (SERVO_PWM, "servo.pwm"),
        (BENCH_PANIC, "bench.panic"),
        (BENCH_ISR_PROBE, "bench.isr_probe"),
    ];

    /// Cargo feature of a bench build and the capability bit its firmware
    /// reports.
    pub const BENCH_FEATURES: [(&str, u32); 2] = [("bench-panic", BENCH_PANIC), ("bench-isr-probe", BENCH_ISR_PROBE)];

    /// The bench features a capability mask reports.
    pub fn bench_features(mask: u32) -> impl Iterator<Item = &'static str> {
        BENCH_FEATURES.iter().filter(move |(_, bit)| mask & bit != 0).map(|(f, _)| *f)
    }

    /// Look up a capability bit by its name.
    pub fn from_name(name: &str) -> Option<u32> {
        NAMES.iter().find(|(_, n)| *n == name).map(|(b, _)| *b)
    }
}

// ---------------------------------------------------------------------------
// Host -> MCU
// ---------------------------------------------------------------------------

/// A frame sent from host to MCU. `seq` is monotonically increasing and is
/// echoed back in [`McuMsg::Ack`] / [`McuMsg::Nack`].
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct HostFrame {
    pub seq: u32,
    pub msg: HostMsg,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum HostMsg {
    /// Request [`McuMsg::Identity`]. Idempotent.
    Identify,
    /// Clock synchronisation probe; answered by [`McuMsg::ClockSyncReply`].
    ClockSync { host_time_us: u64 },
    /// Keep-alive. If the MCU sees no frame for the watchdog timeout it faults
    /// with [`FaultCode::CommunicationLost`] and disables all drives.
    Heartbeat,
    /// Configure the MCU-wide communication watchdog. Idempotent.
    SetWatchdog { timeout_ms: u32 },
    /// Configure an axis. Idempotent; rejected while the axis is moving.
    ConfigureAxis(AxisConfig),
    /// Enable the driver of one axis.
    Enable { axis: u8 },
    /// Disable the driver of one axis (immediately, no deceleration).
    Disable { axis: u8 },
    /// Append a motion segment to the queue. **Not idempotent**: the host must
    /// never blindly retransmit; on doubt it stops and resynchronises.
    QueueSegment(Segment),
    /// Start a homing move; completion reported by [`McuMsg::HomingDone`].
    Home(HomeParams),
    /// Overwrite the step position counter of an idle axis.
    SetPosition { axis: u8, steps: i32 },
    /// Controlled stop: discard queued segments and decelerate every moving
    /// axis with its configured `stop_decel`. Drives stay enabled.
    Stop,
    /// Software emergency stop: halt step generation and disable all drives
    /// immediately. Latches [`FaultCode::EmergencyStop`].
    EmergencyStop,
    /// Clear a latched fault. Never resumes motion; drives stay disabled.
    ResetFault,
    /// Request an immediate [`McuMsg::Status`].
    QueryStatus,
}

/// Static per-axis configuration, all in MCU units.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct AxisConfig {
    pub axis: u8,
    /// Soft limits in steps; the MCU faults if motion would leave them.
    pub min_steps: i32,
    pub max_steps: i32,
    /// Hard ceiling on step rate (steps/s) the firmware will ever produce.
    pub max_rate: f32,
    /// Deceleration used by [`HostMsg::Stop`] (steps/s²).
    pub stop_decel: f32,
    pub invert_dir: bool,
    pub invert_endstop: bool,
}

/// One constant-acceleration motion segment for a single axis.
///
/// Position relative to segment start: `p(t) = v0·t + ½·a·t²` (steps, t in s).
/// After `duration_us` the axis must have moved exactly `steps` steps; the
/// firmware snaps to that integer so rounding never accumulates.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Segment {
    pub axis: u8,
    /// Absolute MCU clock time at which the segment begins.
    pub start_us: u64,
    pub duration_us: u32,
    /// Exact integer step delta of this segment (signed).
    pub steps: i32,
    /// Start velocity in steps/s (signed).
    pub v0: f32,
    /// Acceleration in steps/s² (signed).
    pub accel: f32,
    /// `true` if this is the last segment of a move, i.e. the axis is expected
    /// to be at rest afterwards. If the queue runs dry after a segment with
    /// `last == false` the MCU faults with [`FaultCode::QueueUnderrun`].
    pub last: bool,
}

impl Segment {
    pub fn end_us(&self) -> u64 {
        self.start_us + self.duration_us as u64
    }

    pub fn end_velocity(&self) -> f32 {
        self.v0 + self.accel * (self.duration_us as f32 * 1e-6)
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct HomeParams {
    pub axis: u8,
    /// Search direction: `true` = positive step direction.
    pub positive: bool,
    /// Search speed in steps/s (unsigned).
    pub speed: f32,
    /// Fault with [`FaultCode::HomingFailed`] if no trigger within this many steps.
    pub max_travel_steps: u32,
    /// Position assigned to the axis at the trigger point.
    pub home_steps: i32,
}

// ---------------------------------------------------------------------------
// MCU -> Host
// ---------------------------------------------------------------------------

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct McuFrame {
    pub seq: u32,
    pub msg: McuMsg,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum McuMsg {
    Identity(Identity),
    ClockSyncReply {
        host_time_us: u64,
        mcu_time_us: u64,
    },
    Ack {
        seq: u32,
    },
    Nack {
        seq: u32,
        reason: NackReason,
    },
    Status(Status),
    /// Sent once when a fault latches.
    Fault {
        code: FaultCode,
        axis: Option<u8>,
    },
    HomingDone {
        axis: u8,
        position_steps: i32,
    },
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Identity {
    pub protocol_version: u16,
    pub firmware_version: (u8, u8, u8),
    pub target: String<16>,
    pub board: String<24>,
    /// First 4 bytes of the firmware build hash.
    pub build_hash: u32,
    pub capabilities: u32,
    pub axes: u8,
    pub queue_capacity: u8,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum NackReason {
    UnknownAxis,
    QueueFull,
    InvalidState,
    InvalidParams,
    SoftLimit,
    Faulted,
    Busy,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum FaultCode {
    CommunicationLost,
    QueueUnderrun,
    SegmentLate,
    LimitHit,
    SoftLimit,
    HomingFailed,
    EmergencyStop,
    StepRateExceeded,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Status {
    pub mcu_time_us: u64,
    pub fault: Option<FaultCode>,
    pub queue_depth: u8,
    pub axes: Vec<AxisStatus, MAX_AXES>,
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct AxisStatus {
    pub position_steps: i32,
    /// Current velocity in steps/s.
    pub velocity: f32,
    pub enabled: bool,
    pub moving: bool,
    pub homing: bool,
    pub endstop: bool,
}

/// Convert a capability bitmask to names (host-side helper).
pub fn capability_names(mask: u32) -> impl Iterator<Item = &'static str> {
    caps::NAMES.iter().filter(move |(bit, _)| mask & bit != 0).map(|(_, name)| *name)
}
