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
//! STM32 I/O is 3.3 V. Verify driver logic thresholds and never connect
//! 5 V/12 V switch circuits directly.

use motioncore_firmware_core::Hal;
use motioncore_protocol::caps;
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
}

impl BoardHal {
    pub fn new(pa0: PA0, pa1: PA1, pa2: PA2, pb0: PB0) -> Self {
        let mut enable = pa2.into_push_pull_output();
        enable.set_high(); // disabled
        Self {
            step: pa0.into_push_pull_output(),
            dir: pa1.into_push_pull_output(),
            enable,
            endstop: pb0.into_pull_up_input(),
            dir_state: false,
        }
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
