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
//! GP28 is used by bench test builds only, never in production firmware:
//! * feature `bench-panic`: pull-up input; tying it to GND makes the firmware
//!   panic in the next motion tick, to measure the panic safe state;
//! * feature `bench-isr-probe`: output, high while `TIMER_IRQ_0` runs, to
//!   measure the motion tick's interrupt time and period.
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
pub const CAPABILITIES: u32 = caps::STEPPER_STEP_DIR | caps::ENDSTOP | caps::WATCHDOG | BENCH_CAPABILITIES;

#[cfg(all(feature = "bench-panic", feature = "bench-isr-probe"))]
compile_error!("features `bench-panic` and `bench-isr-probe` both use GP28; build one at a time");

/// Reported at handshake so `firmware verify` and the engine can tell a
/// bench test build from a production build.
const BENCH_CAPABILITIES: u32 =
    if cfg!(feature = "bench-panic") { caps::BENCH_PANIC } else { 0 } | if cfg!(feature = "bench-isr-probe") { caps::BENCH_ISR_PROBE } else { 0 };

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
    #[cfg(feature = "bench-panic")]
    panic_trigger: In,
    #[cfg(feature = "bench-isr-probe")]
    isr_probe: Out,
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
        #[cfg(feature = "bench-panic")]
        let panic_trigger = pins.gpio28.into_pull_up_input().into_dyn_pin();
        #[cfg(feature = "bench-isr-probe")]
        let isr_probe = pins.gpio28.into_push_pull_output_in_state(PinState::Low).into_dyn_pin();
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
        Self {
            enable,
            endstop,
            interlock,
            led,
            #[cfg(feature = "bench-panic")]
            panic_trigger,
            #[cfg(feature = "bench-isr-probe")]
            isr_probe,
            sms,
            runs: [StepRuns::new(); AXES],
            enabled: [false; AXES],
        }
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

#[cfg(feature = "bench-panic")]
impl BoardHal {
    /// `true` while the bench panic trigger (GP28) is tied to GND.
    pub fn panic_requested(&mut self) -> bool {
        self.panic_trigger.is_low().unwrap_or(false)
    }
}

#[cfg(feature = "bench-isr-probe")]
impl BoardHal {
    /// Drive the tick-interrupt probe (GP28).
    pub fn isr_probe(&mut self, high: bool) {
        let _ = self.isr_probe.set_state(PinState::from(high));
    }
}

/// ENABLE outputs GP12–GP17 (active low: high = driver disabled).
const ENABLE_MASK: u32 = 0x3f << 12;
/// STEP and DIR outputs GP0–GP11 (driven by PIO during operation).
const STEP_DIR_MASK: u32 = 0xfff;
/// On-board LED (GP25), lit while a drive is enabled.
const LED_MASK: u32 = 1 << 25;
/// IO_BANK0 `GPIOx_CTRL.FUNCSEL` value for software-controlled I/O (SIO).
const FUNCSEL_SIO: u32 = 5;

/// Put every output into its safe state, from any context (panic handler,
/// fault paths), without relying on the state of the code that failed:
///
/// 1. interrupts off, so the motion tick cannot run again;
/// 2. every ENABLE line to its disabled level (drivers off) — first, because
///    it is what matters;
/// 3. both PIO blocks stopped, so no queued step window keeps running;
/// 4. STEP and DIR handed from PIO back to SIO and driven low, so no pin is
///    left high in the middle of a pulse;
/// 5. LED off.
///
/// Uses direct register writes only. Infallible. The external E-stop chain
/// remains the safety function; this only keeps a firmware fault from leaving
/// drives energised until the watchdog resets the MCU (ADR 0002, #68).
pub fn safe_shutdown() {
    cortex_m::interrupt::disable();
    // SAFETY: single-core shutdown path with interrupts disabled; plain
    // register writes to SIO, PIO and IO_BANK0, which are valid in any state.
    unsafe {
        let sio = &*rp2040_hal::pac::SIO::ptr();
        sio.gpio_out_set().write(|w| w.bits(ENABLE_MASK));
        sio.gpio_oe_set().write(|w| w.bits(ENABLE_MASK));
        for pio in [&*rp2040_hal::pac::PIO0::ptr(), &*rp2040_hal::pac::PIO1::ptr()] {
            pio.ctrl().write(|w| w.bits(0));
        }
        sio.gpio_out_clr().write(|w| w.bits(STEP_DIR_MASK | LED_MASK));
        sio.gpio_oe_set().write(|w| w.bits(STEP_DIR_MASK));
        let io = &*rp2040_hal::pac::IO_BANK0::ptr();
        for pin in 0..12 {
            io.gpio(pin).gpio_ctrl().write(|w| w.bits(FUNCSEL_SIO));
        }
    }
}
