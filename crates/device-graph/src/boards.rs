//! Board metadata used by the resolver and the firmware builder.
//!
//! Only boards listed here can be targeted. Adding a board means adding its
//! metadata *and* a firmware target; the resolver never guesses.

#[derive(Debug)]
pub struct Board {
    pub id: &'static str,
    pub description: &'static str,
    /// Board crate directory under `boards/` (package `motioncore-board-<target>`).
    pub firmware_target: Option<&'static str>,
    /// Rust target triple used to build firmware.
    pub rust_target: Option<&'static str>,
    pub is_virtual: bool,
    pub step_channels: u8,
    pub max_step_rate: u32,
    /// Usable GPIO pins; empty + `is_virtual` means "any pin name".
    pub pins: &'static [&'static str],
    pub io_voltage: Option<f32>,
    /// `true` while the board choice is still an open design decision.
    pub provisional: bool,
}

impl Board {
    pub fn has_pin(&self, pin: &str) -> bool {
        if self.is_virtual {
            !pin.is_empty()
        } else {
            self.pins.contains(&pin)
        }
    }
}

pub static BOARDS: &[Board] = &[
    Board {
        id: "virtual",
        description: "Simulated MCU running the shared firmware core",
        firmware_target: None,
        rust_target: None,
        is_virtual: true,
        step_channels: 6,
        max_step_rate: 50_000,
        pins: &[],
        io_voltage: None,
        provisional: false,
    },
    Board {
        id: "stm32f411-blackpill",
        description: "WeAct STM32F411CEU6 'Black Pill' (provisional MVP board)",
        firmware_target: Some("stm32f4"), // boards/stm32f4
        rust_target: Some("thumbv7em-none-eabihf"),
        is_virtual: false,
        // The v0.1 firmware wires one step/dir channel (see boards/stm32f4/src/board.rs).
        step_channels: 1,
        // Bounded by the 20 kHz core tick of the v0.1 firmware.
        max_step_rate: 20_000,
        pins: &[
            "PA0", "PA1", "PA2", "PA3", "PA4", "PA5", "PA6", "PA7", "PA8", "PA9", "PA10", "PA15",
            "PB0", "PB1", "PB3", "PB4", "PB5", "PB6", "PB7", "PB8", "PB9", "PB10", "PB12", "PB13",
            "PB14", "PB15",
        ],
        io_voltage: Some(3.3),
        provisional: true,
    },
];

pub fn find(id: &str) -> Option<&'static Board> {
    BOARDS.iter().find(|b| b.id == id)
}
