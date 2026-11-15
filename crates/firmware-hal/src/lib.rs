//! Hardware interfaces for MotionCore firmware.
//!
//! Every board crate (`boards/*`) and the simulator implement these traits;
//! everything above them (`firmware-stepper`, `firmware-core`) is target
//! independent. Keep this crate tiny and dependency-free: it is the boundary
//! that makes virtual/physical parity possible.
#![no_std]
#![forbid(unsafe_code)]

/// Step/dir outputs, driver enables and endstop inputs, indexed by channel.
pub trait Hal {
    /// Emit exactly one step pulse on `axis`. `forward` is the *electrical*
    /// DIR level (direction inversion has already been applied).
    fn step(&mut self, axis: u8, forward: bool);
    /// Drive the enable output (`true` = driver energised).
    fn set_enable(&mut self, axis: u8, enabled: bool);
    /// Raw (un-inverted) endstop input level.
    fn endstop_raw(&mut self, axis: u8) -> bool;
    /// `true` while an external safety interlock (E-stop chain, guard door,
    /// driver-power monitor) reports a safe state. While it is open the
    /// firmware latches an emergency stop and refuses to clear faults.
    ///
    /// Boards without such an input keep this default. The interlock input is
    /// only a *monitor*: the physical E-stop circuit must remove driver power
    /// independently of the firmware.
    fn interlock_ok(&mut self) -> bool {
        true
    }
    /// Called once at the end of every motion tick, after all `step` calls of
    /// that tick.
    ///
    /// Boards that emit each pulse directly inside [`Hal::step`] (STM32, the
    /// simulator) keep this default. Boards with hardware-timed pulse output
    /// (RP2040 PIO) buffer the tick's steps in `step` and hand them to the
    /// hardware here, spread over the next tick window.
    ///
    /// Returns `false` if the pulse hardware cannot keep up (output still
    /// busy with an earlier window, buffer overflow, or rate beyond the pulse
    /// timing limits); the firmware then latches `StepRateExceeded`.
    fn flush_pulses(&mut self) -> bool {
        true
    }
}

/// Monotonic microsecond clock of the MCU.
pub trait Clock {
    fn now_us(&mut self) -> u64;
}
