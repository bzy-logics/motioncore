//! Raspberry Pi Pico pin map and [`Hal`] implementation. Must match the
//! `rp2040-pico` entry in `crates/device-graph/src/boards.rs` (the resolver
//! rejects profiles that disagree):
//!
//! | channel | STEP | DIR  | ENABLE (active low) | ENDSTOP (to GND) | PIO / SM |
//! |---------|------|------|---------------------|------------------|----------|
//! | 0       | GP0  | GP1  | GP12                | GP18             | PIO0 SM0 |
//! | 1       | GP2  | GP3  | GP13                | GP19             | PIO0 SM1 |
//! | 2       | GP4  | GP5  | GP14                | GP20             | PIO0 SM2 |
//! | 3       | GP6  | GP7  | GP15                | GP21             | PIO0 SM3 |
//! | 4       | GP8  | GP9  | GP16                | GP22             | PIO1 SM0 |
//! | 5       | GP10 | GP11 | GP17                | GP26             | PIO1 SM1 |
//!
//! GP27: safety interlock input (pull-up). It must be pulled to GND through
//! the E-stop chain's monitoring contact; open = unsafe. GP25 (on-board LED)
//! is lit while any drive is enabled.
//!
//! RP2040 I/O is 3.3 V and **not** 5 V tolerant. Drivers need 3.3 V-compatible
//! logic inputs (or level shifters), and every ENABLE line needs an external
//! pull-up so a reset or unprogrammed MCU leaves the drivers disabled.

use embedded_hal::digital::{InputPin, OutputPin};
use motioncore_firmware_core::Hal;
use motioncore_firmware_pio_step::{program, sm_for_axis, StepRuns, Timing};
use motioncore_protocol::caps;
use rp2040_hal::gpio::{DynPinId, FunctionPio0, FunctionPio1, FunctionSioInput, FunctionSioOutput, Pin, PinState, Pins, PullDown, PullUp};

use crate::pio::{load_program, StepSm};

pub const AXES: usize = 6;
pub const CAPABILITIES: u32 = caps::STEPPER_STEP_DIR | caps::ENDSTOP | caps::WATCHDOG;

/// Motion tick period = PIO pulse window.
pub const TICK_US: u32 = 200;

/// System clock the PIO divider assumes (set by `init_clocks_and_plls`).
pub const SYS_CLOCK_HZ: u32 = 125_000_000;

/// PIO pulse timing: 5 MHz state-machine clock (200 ns per cycle), 2 µs
/// STEP high, ≥ 2 µs STEP low, 2 µs DIR setup, pulses within 90 % of each
/// window. Check against the driver datasheet before changing.
pub const TIMING: Timing = Timing { sm_clock_hz: 5_000_000, high_cycles: 10, min_low_cycles: 10, dir_setup_cycles: 10, window_budget_pct: 90 };

pub const PIO_CLKDIV: u16 = (SYS_CLOCK_HZ / TIMING.sm_clock_hz) as u16;
// `is_multiple_of` needs Rust 1.87; the workspace MSRV is 1.85.
#[allow(clippy::manual_is_multiple_of)]
const _: () = assert!(SYS_CLOCK_HZ % TIMING.sm_clock_hz == 0, "integer PIO divider only (no fractional jitter)");

const STEP_PINS: [u8; AXES] = [0, 2, 4, 6, 8, 10];
const DIR_PINS: [u8; AXES] = [1, 3, 5, 7, 9, 11];

type Out = Pin<DynPinId, FunctionSioOutput, PullDown>;
type In = Pin<DynPinId, FunctionSioInput, PullUp>;

pub struct BoardHal {
    enable: [Out; AXES],
    endstop: [In; AXES],
    interlock: In,
    led: Out,
    sms: [StepSm; AXES],
    runs: [StepRuns; AXES],
    enabled: [bool; AXES],
}

macro_rules! pio_pins {
    ($pins:ident, $f:ty: $($p:ident),*) => {
        $( let _ = $pins.$p.into_function::<$f>(); )*
    };
}

impl BoardHal {
    /// Takes the GPIO bank; drives stay disabled. `pio0`/`pio1` must already
    /// be out of reset.
    pub fn new(pins: Pins) -> Self {
        // ENABLE first, driven high (disabled) before anything else.
        let enable = [
            pins.gpio12.into_push_pull_output_in_state(PinState::High).into_dyn_pin(),
            pins.gpio13.into_push_pull_output_in_state(PinState::High).into_dyn_pin(),
            pins.gpio14.into_push_pull_output_in_state(PinState::High).into_dyn_pin(),
            pins.gpio15.into_push_pull_output_in_state(PinState::High).into_dyn_pin(),
            pins.gpio16.into_push_pull_output_in_state(PinState::High).into_dyn_pin(),
            pins.gpio17.into_push_pull_output_in_state(PinState::High).into_dyn_pin(),
        ];
        let endstop = [
            pins.gpio18.into_pull_up_input().into_dyn_pin(),
            pins.gpio19.into_pull_up_input().into_dyn_pin(),
            pins.gpio20.into_pull_up_input().into_dyn_pin(),
            pins.gpio21.into_pull_up_input().into_dyn_pin(),
            pins.gpio22.into_pull_up_input().into_dyn_pin(),
            pins.gpio26.into_pull_up_input().into_dyn_pin(),
        ];
        let interlock = pins.gpio27.into_pull_up_input().into_dyn_pin();
        let led = pins.gpio25.into_push_pull_output_in_state(PinState::Low).into_dyn_pin();

        pio_pins!(pins, FunctionPio0: gpio0, gpio1, gpio2, gpio3, gpio4, gpio5, gpio6, gpio7);
        pio_pins!(pins, FunctionPio1: gpio8, gpio9, gpio10, gpio11);

        let prog = program(&TIMING).unwrap_or_else(|_| panic!("invalid PIO timing"));
        load_program(0, &prog);
        load_program(1, &prog);
        let sms = core::array::from_fn(|i| {
            let (block, sm) = sm_for_axis(i as u8);
            StepSm::new(block, sm, &prog, STEP_PINS[i], DIR_PINS[i], PIO_CLKDIV)
        });
        Self { enable, endstop, interlock, led, sms, runs: [StepRuns::new(); AXES], enabled: [false; AXES] }
    }
}

impl Hal for BoardHal {
    fn step(&mut self, axis: u8, forward: bool) {
        if let Some(r) = self.runs.get_mut(axis as usize) {
            r.push(forward);
        }
    }

    fn set_enable(&mut self, axis: u8, enabled: bool) {
        let i = axis as usize;
        if i >= AXES {
            return;
        }
        if enabled {
            let _ = self.enable[i].set_low();
        } else {
            // Driver off first, then inhibit any pulses still queued.
            let _ = self.enable[i].set_high();
            self.runs[i].clear();
            self.sms[i].abort();
        }
        self.enabled[i] = enabled;
        let _ = if self.enabled.iter().any(|&e| e) { self.led.set_high() } else { self.led.set_low() };
    }

    fn endstop_raw(&mut self, axis: u8) -> bool {
        // Switch to GND with pull-up: pressed == low.
        self.endstop.get_mut(axis as usize).is_some_and(|p| p.is_low().unwrap_or(true))
    }

    fn interlock_ok(&mut self) -> bool {
        // Closed (pulled to GND) == safe. Open wire == unsafe.
        self.interlock.is_low().unwrap_or(false)
    }

    fn flush_pulses(&mut self) -> bool {
        let mut ok = true;
        for i in 0..AXES {
            if self.runs[i].is_empty() {
                continue;
            }
            // The previous window must have been taken by the state machine;
            // otherwise output is lagging behind the tick and we stop rather
            // than let the lag grow.
            if !self.sms[i].tx_empty() {
                self.runs[i].clear();
                ok = false;
                continue;
            }
            match self.runs[i].take_commands(&TIMING, TICK_US) {
                Ok(cmds) => {
                    for &w in cmds.words() {
                        ok &= self.sms[i].push(w);
                    }
                }
                Err(_) => ok = false,
            }
        }
        ok
    }
}
