//! Board metadata used by the resolver and the firmware builder.
//!
//! Only boards listed here can be targeted. Adding a board means adding its
//! metadata *and* a firmware target; the resolver never guesses.

/// Pins the firmware hard-wires to one step channel.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ChannelPins {
    pub step: &'static str,
    pub dir: &'static str,
    pub enable: &'static str,
    pub endstop: &'static str,
}

/// How a built image gets onto the MCU.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FlashMethod {
    /// No firmware (virtual boards).
    None,
    /// SWD debug probe via `probe-rs download --chip <chip>`.
    ProbeRs { chip: &'static str },
    /// UF2 copied to the USB mass-storage bootloader (e.g. RP2040 BOOTSEL).
    Uf2 { family_id: u32, volume_label: &'static str },
}

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
    /// Firmware pin map, indexed by step channel. Empty = no fixed map.
    pub channel_pins: &'static [ChannelPins],
    pub flash: FlashMethod,
    /// Reference robot profile (under `robot-profiles/`) used by
    /// `motioncore firmware build --board <id>`.
    pub reference_profile: Option<&'static str>,
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
        channel_pins: &[],
        flash: FlashMethod::None,
        reference_profile: None,
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
            "PA0", "PA1", "PA2", "PA3", "PA4", "PA5", "PA6", "PA7", "PA8", "PA9", "PA10", "PA15", "PB0", "PB1", "PB3", "PB4", "PB5", "PB6", "PB7", "PB8",
            "PB9", "PB10", "PB12", "PB13", "PB14", "PB15",
        ],
        io_voltage: Some(3.3),
        provisional: true,
        channel_pins: &[ChannelPins { step: "PA0", dir: "PA1", enable: "PA2", endstop: "PB0" }],
        flash: FlashMethod::ProbeRs { chip: "STM32F411CEUx" },
        reference_profile: Some("single-axis-stm32f411.toml"),
    },
    Board {
        id: "rp2040-pico",
        description: "Raspberry Pi Pico (RP2040), PIO-timed STEP/DIR (provisional)",
        firmware_target: Some("rp2040"), // boards/rp2040
        rust_target: Some("thumbv6m-none-eabi"),
        is_virtual: false,
        // Six PIO state machines (PIO0 SM0-3, PIO1 SM0-1); see docs/rp2040.md.
        // Six-axis CPU timing is not yet measured on hardware.
        step_channels: 6,
        // Conservative limit: 10 steps per 200 µs window. The PIO timing
        // itself allows 250 kHz; see docs/rp2040.md before raising it.
        max_step_rate: 50_000,
        pins: &[
            "GP0", "GP1", "GP2", "GP3", "GP4", "GP5", "GP6", "GP7", "GP8", "GP9", "GP10", "GP11", "GP12", "GP13", "GP14", "GP15", "GP16", "GP17", "GP18",
            "GP19", "GP20", "GP21", "GP22", "GP26", "GP27", "GP28",
        ],
        io_voltage: Some(3.3),
        provisional: true,
        channel_pins: &[
            ChannelPins { step: "GP0", dir: "GP1", enable: "GP12", endstop: "GP18" },
            ChannelPins { step: "GP2", dir: "GP3", enable: "GP13", endstop: "GP19" },
            ChannelPins { step: "GP4", dir: "GP5", enable: "GP14", endstop: "GP20" },
            ChannelPins { step: "GP6", dir: "GP7", enable: "GP15", endstop: "GP21" },
            ChannelPins { step: "GP8", dir: "GP9", enable: "GP16", endstop: "GP22" },
            ChannelPins { step: "GP10", dir: "GP11", enable: "GP17", endstop: "GP26" },
        ],
        flash: FlashMethod::Uf2 { family_id: 0xE48B_FF56, volume_label: "RPI-RP2" },
        reference_profile: Some("six-axis-rp2040-pico.toml"),
    },
];

pub fn find(id: &str) -> Option<&'static Board> {
    BOARDS.iter().find(|b| b.id == id)
}
