//! Pin mapping for the single-axis bench. Must match
//! `robot-profiles/single-axis-stm32f411.toml`:
//!
//! | signal  | pin | notes |
//! |---------|-----|-------|
//! | STEP0   | PA0 | push-pull, ≥2 µs pulse |
//! | DIR0    | PA1 | push-pull |
//! | EN0     | PA2 | push-pull, **active low**, external pull-up required |
//! | ENDSTOP0| PB0 | input with pull-up, switch to GND (active low) |
//!
//! | PANIC   | PB1 | feature `bench-panic` only: pull-up input, tie to GND to make the firmware panic (bench test of the panic safe state) |
//!
//! STM32 I/O is 3.3 V. Verify driver logic thresholds and never connect
//! 5 V/12 V switch circuits directly.

use motioncore_firmware_core::Hal;
use motioncore_protocol::caps;
#[cfg(feature = "bench-panic")]
use stm32f4xx_hal::gpio::PB1;
use stm32f4xx_hal::gpio::{Input, Output, PushPull, PA0, PA1, PA2, PB0};

pub const AXES: usize = 1;
pub const CAPABILITIES: u32 = caps::STEPPER_STEP_DIR | caps::ENDSTOP | caps::WATCHDOG;

/// Cycles for the minimum STEP high time, STEP low time and DIR setup time
/// at 96 MHz (~2 µs each). Check against the driver datasheet.
const PULSE_CYCLES: u32 = 192;

pub struct BoardHal {
    step: PA0<Output<PushPull>>,
    dir: PA1<Output<PushPull>>,
    enable: PA2<Output<PushPull>>,
    endstop: PB0<Input>,
    dir_state: bool,
    #[cfg(feature = "bench-panic")]
    panic_trigger: PB1<Input>,
}

impl BoardHal {
    pub fn new(pa0: PA0, pa1: PA1, pa2: PA2, pb0: PB0, #[cfg(feature = "bench-panic")] pb1: PB1) -> Self {
        let mut enable = pa2.into_push_pull_output();
        enable.set_high(); // disabled
        Self {
            step: pa0.into_push_pull_output(),
            dir: pa1.into_push_pull_output(),
            enable,
            endstop: pb0.into_pull_up_input(),
            dir_state: false,
            #[cfg(feature = "bench-panic")]
            panic_trigger: pb1.into_pull_up_input(),
        }
    }

    /// `true` while the bench panic trigger (PB1) is tied to GND.
    #[cfg(feature = "bench-panic")]
    pub fn panic_requested(&self) -> bool {
        self.panic_trigger.is_low()
    }
}

/// Put every output into its safe state, from any context (panic handler,
/// fault paths), without relying on the state of the code that failed:
/// interrupts off, EN0 (PA2, active low) high = driver disabled, STEP0 (PA0)
/// and DIR0 (PA1) low. Direct register writes only; infallible. The external
/// E-stop chain remains the safety function (ADR 0002, #68).
pub fn safe_shutdown() {
    cortex_m::interrupt::disable();
    // SAFETY: single-core shutdown path with interrupts disabled; one atomic
    // BSRR write to GPIOA, valid in any state.
    unsafe {
        let gpioa = &*stm32f4xx_hal::pac::GPIOA::ptr();
        // Set PA2 (bit 2); reset PA0 and PA1 (bits 16 and 17).
        gpioa.bsrr().write(|w| w.bits((1 << 2) | (1 << 16) | (1 << 17)));
    }
}

impl Hal for BoardHal {
    fn step(&mut self, axis: u8, forward: bool) {
        if axis != 0 {
            return;
        }
        if forward != self.dir_state {
            if forward {
                self.dir.set_high();
            } else {
                self.dir.set_low();
            }
            self.dir_state = forward;
            cortex_m::asm::delay(PULSE_CYCLES);
        }
        self.step.set_high();
        cortex_m::asm::delay(PULSE_CYCLES);
        self.step.set_low();
        // Minimum low time, so back-to-back steps in one tick (catch-up after
        // a delayed tick) still meet the driver's pulse spacing.
        cortex_m::asm::delay(PULSE_CYCLES);
    }

    fn set_enable(&mut self, axis: u8, enabled: bool) {
        if axis == 0 {
            if enabled {
                self.enable.set_low();
            } else {
                self.enable.set_high();
            }
        }
    }

    fn endstop_raw(&mut self, axis: u8) -> bool {
        // Switch to GND with pull-up: pressed == low.
        axis == 0 && self.endstop.is_low()
    }
}
