//! Hardware-timed STEP/DIR pulse generation for the RP2040 PIO.
//!
//! The shared stepper runtime (`firmware-stepper`) decides **which** steps are
//! due in each motion tick. This crate decides **when** they appear on the
//! wire: the steps of one tick are handed to a PIO state machine as a command
//! that spreads them evenly over the next tick window, with guaranteed
//! minimum pulse-high, pulse-low and direction setup/hold times. Overdue
//! steps are therefore never emitted as an uncontrolled burst.
//!
//! Everything here is target independent and has no dependencies, so the
//! program encoding and timing arithmetic are verified on the host against a
//! cycle-accurate model of the PIO program (see the tests).
//!
//! # PIO program (one state machine per axis)
//!
//! Side-set: 1 bit, mandatory, drives STEP. OUT pin base: DIR (1 pin).
//! OSR shifts right, autopull off.
//!
//! ```text
//! 0  pull block          side 0         ; wait for a command
//! 1  out pins, 1         side 0 [S]     ; DIR, then setup delay
//! 2  out x, 15           side 0         ; pulse count - 1
//! 3  out isr, 16         side 0         ; low-time loop count L
//! 4  pulse: nop          side 1 [H]     ; STEP high for H+1 cycles
//! 5  mov y, isr          side 0
//! 6  low: jmp y-- low    side 0         ; L+1 cycles
//! 7  jmp x-- pulse       side 0
//!    (wrap 7 -> 0)
//! ```
//!
//! Command word (LSB first): bit 0 = DIR level, bits 1..16 = pulses - 1,
//! bits 16..32 = L.
//!
//! Timing in state-machine cycles, with `high = H + 1`:
//! * STEP high: `high`
//! * STEP low between pulses: `L + 3`
//! * pulse period: `high + L + 3`
//! * DIR setup (DIR change to first rising edge): `S + 3`
//! * DIR hold (last falling edge to a DIR change by the next command):
//!   `L + 4` (> minimum low time)
//! * command duration: `S + 4 + pulses * period`
#![cfg_attr(not(test), no_std)]
#![forbid(unsafe_code)]

/// Instructions in [`Program::code`].
pub const PROGRAM_LEN: usize = 8;
/// Largest pulse count of one command.
pub const MAX_PULSES_PER_COMMAND: u32 = 1 << 15;
/// Largest low-time loop count `L`.
pub const MAX_LOW_LOOP: u32 = u16::MAX as u32;
/// Direction runs one axis may buffer within one tick. Each run becomes one
/// FIFO word, so this must not exceed the TX FIFO depth.
pub const MAX_RUNS: usize = 4;

/// Pulse timing of one board, in PIO state-machine cycles.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Timing {
    /// State-machine clock after the divider (Hz).
    pub sm_clock_hz: u32,
    /// STEP high time, 1..=16 cycles.
    pub high_cycles: u8,
    /// Minimum STEP low time, >= 3 cycles.
    pub min_low_cycles: u16,
    /// DIR setup before the first STEP rising edge, 3..=18 cycles.
    pub dir_setup_cycles: u8,
    /// Share of each tick window pulses may occupy (1..=100 %). The rest is
    /// slack for command overhead and interrupt jitter.
    pub window_budget_pct: u8,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TimingError {
    HighCycles,
    MinLowCycles,
    DirSetupCycles,
    WindowBudget,
    SmClock,
}

impl Timing {
    pub fn validate(&self) -> Result<(), TimingError> {
        if !(1..=16).contains(&self.high_cycles) {
            return Err(TimingError::HighCycles);
        }
        if self.min_low_cycles < 3 || self.min_low_cycles as u32 > MAX_LOW_LOOP + 3 {
            return Err(TimingError::MinLowCycles);
        }
        if !(3..=18).contains(&self.dir_setup_cycles) {
            return Err(TimingError::DirSetupCycles);
        }
        if !(1..=100).contains(&self.window_budget_pct) {
            return Err(TimingError::WindowBudget);
        }
        if self.sm_clock_hz == 0 {
            return Err(TimingError::SmClock);
        }
        Ok(())
    }

    /// Fixed cycles per command before the first rising edge.
    pub fn command_overhead_cycles(&self) -> u32 {
        self.dir_setup_cycles as u32 + 1
    }

    /// Shortest pulse period the timing allows.
    pub fn min_period_cycles(&self) -> u32 {
        self.high_cycles as u32 + self.min_low_cycles as u32
    }

    /// Highest step rate (steps/s) the pulse timing allows, ignoring windows.
    pub fn max_step_rate(&self) -> u32 {
        self.sm_clock_hz / self.min_period_cycles()
    }

    /// Cycles of a tick window available for pulses.
    pub fn window_budget_cycles(&self, window_us: u32) -> u32 {
        let cycles = window_us as u64 * self.sm_clock_hz as u64 / 1_000_000;
        (cycles * self.window_budget_pct as u64 / 100) as u32
    }

    /// Most steps one axis can emit in one window of `window_us` (one run).
    pub fn max_steps_per_window(&self, window_us: u32) -> u32 {
        let budget = self.window_budget_cycles(window_us);
        budget.saturating_sub(self.command_overhead_cycles()) / self.min_period_cycles()
    }

    /// Cycles a command with `pulses` pulses and low loop `low_loop` takes.
    pub fn command_cycles(&self, pulses: u32, low_loop: u32) -> u32 {
        self.command_overhead_cycles() + pulses * (self.high_cycles as u32 + low_loop + 3)
    }
}

/// The assembled program plus its wrap points (relative to load offset 0).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Program {
    pub code: [u16; PROGRAM_LEN],
    /// Last instruction before wrapping (EXECCTRL.WRAP_TOP).
    pub wrap_source: u8,
    /// Wrap destination (EXECCTRL.WRAP_BOTTOM).
    pub wrap_target: u8,
    /// Side-set bit count (PINCTRL.SIDESET_COUNT), no optional bit.
    pub side_set_bits: u8,
}

/// Address of the `pulse` label; JMP targets are relative to the load offset
/// and must be relocated by the loader.
pub const LABEL_PULSE: u8 = 4;
/// Address of the `low` label.
pub const LABEL_LOW: u8 = 6;

const fn side_delay(side: u16, delay: u16) -> u16 {
    // 1 side-set bit (non-optional) occupies bit 12; delay bits 11..8.
    (side << 12) | (delay << 8)
}

/// Assemble the step program for `t`.
pub fn program(t: &Timing) -> Result<Program, TimingError> {
    t.validate()?;
    let setup_delay = (t.dir_setup_cycles - 3) as u16;
    let high_delay = (t.high_cycles - 1) as u16;
    let code = [
        0x80A0 | side_delay(0, 0),                     // pull block
        0x6001 | side_delay(0, setup_delay),           // out pins, 1 [S]
        0x6020 | 15 | side_delay(0, 0),                // out x, 15
        0x60C0 | 16 | side_delay(0, 0),                // out isr, 16
        0xA042 | side_delay(1, high_delay),            // nop (mov y, y) side 1 [H]
        0xA046 | side_delay(0, 0),                     // mov y, isr
        0x0080 | LABEL_LOW as u16 | side_delay(0, 0),  // jmp y-- low
        0x0040 | LABEL_PULSE as u16 | side_delay(0, 0), // jmp x-- pulse
    ];
    Ok(Program { code, wrap_source: 7, wrap_target: 0, side_set_bits: 1 })
}

/// Relocate a program instruction for loading at `offset` (JMP targets only).
pub fn relocate(instr: u16, offset: u8) -> u16 {
    if instr & 0xE000 == 0 {
        let addr = (instr & 0x1F) as u8;
        (instr & !0x1F) | ((addr.wrapping_add(offset) & 0x1F) as u16)
    } else {
        instr
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PulseError {
    /// The steps do not fit the window at the minimum pulse period.
    TooFast,
    /// More direction changes in one tick than [`MAX_RUNS`].
    TooManyRuns,
}

/// Encode one command: `pulses` steps in direction `dir_level`, spread over
/// `span_cycles`. Returns the FIFO word and its duration in cycles.
pub fn command(t: &Timing, dir_level: bool, pulses: u32, span_cycles: u32) -> Result<(u32, u32), PulseError> {
    debug_assert!(pulses >= 1);
    if pulses == 0 || pulses > MAX_PULSES_PER_COMMAND {
        return Err(PulseError::TooFast);
    }
    let avail = span_cycles.saturating_sub(t.command_overhead_cycles());
    let period = avail / pulses;
    if period < t.min_period_cycles() {
        return Err(PulseError::TooFast);
    }
    let low_loop = (period - t.high_cycles as u32 - 3).min(MAX_LOW_LOOP);
    let word = dir_level as u32 | ((pulses - 1) << 1) | (low_loop << 16);
    Ok((word, t.command_cycles(pulses, low_loop)))
}

/// The steps of one axis requested during one tick, as direction runs.
#[derive(Debug, Clone, Copy, Default)]
pub struct StepRuns {
    runs: [(bool, u32); MAX_RUNS],
    len: usize,
    overflow: bool,
}

impl StepRuns {
    pub const fn new() -> Self {
        Self { runs: [(false, 0); MAX_RUNS], len: 0, overflow: false }
    }

    /// Record one step at electrical DIR level `dir_level`.
    pub fn push(&mut self, dir_level: bool) {
        if self.len > 0 && self.runs[self.len - 1].0 == dir_level {
            self.runs[self.len - 1].1 += 1;
        } else if self.len < MAX_RUNS {
            self.runs[self.len] = (dir_level, 1);
            self.len += 1;
        } else {
            self.overflow = true;
        }
    }

    pub fn is_empty(&self) -> bool {
        self.len == 0 && !self.overflow
    }

    pub fn total(&self) -> u32 {
        self.runs[..self.len].iter().map(|r| r.1).sum()
    }

    pub fn clear(&mut self) {
        *self = Self::new();
    }

    /// Turn the buffered runs into commands that together fit into one tick
    /// window, then clear the buffer. Each run gets a share of the window
    /// proportional to its step count, so the step rate stays even across a
    /// direction change.
    pub fn take_commands(&mut self, t: &Timing, window_us: u32) -> Result<Commands, PulseError> {
        let runs = *self;
        self.clear();
        if runs.overflow {
            return Err(PulseError::TooManyRuns);
        }
        let mut out = Commands { words: [0; MAX_RUNS], len: 0, cycles: 0 };
        let total = runs.total();
        if total == 0 {
            return Ok(out);
        }
        let budget = t.window_budget_cycles(window_us);
        for &(dir, n) in &runs.runs[..runs.len] {
            let span = (budget as u64 * n as u64 / total as u64) as u32;
            let (word, cycles) = command(t, dir, n, span)?;
            out.words[out.len] = word;
            out.len += 1;
            out.cycles += cycles;
        }
        Ok(out)
    }
}

/// FIFO words for one axis and one window.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Commands {
    words: [u32; MAX_RUNS],
    len: usize,
    /// Total state-machine cycles the commands take.
    pub cycles: u32,
}

impl Commands {
    pub fn words(&self) -> &[u32] {
        &self.words[..self.len]
    }
}

/// How the axes are mapped onto the RP2040's two PIO blocks (four state
/// machines each). One state machine per axis; the program (8 of 32
/// instruction slots) is loaded once per block that is used.
pub const fn sm_for_axis(axis: u8) -> (u8, u8) {
    (axis / 4, axis % 4)
}

#[cfg(test)]
mod emu;
#[cfg(test)]
mod tests;
#[cfg(test)]
mod pipeline_tests;
