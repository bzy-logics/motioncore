//! MotionCore firmware for the RP2040 (Raspberry Pi Pico), with STEP/DIR
//! pulses timed by PIO.
//!
//! * `TIMER` alarm 0 fires every [`board::TICK_US`] and drives
//!   [`McuCore::tick`]: the shared stepper runtime decides which steps are
//!   due, and [`board::BoardHal`] hands them to one PIO state machine per
//!   axis, spread evenly over the next tick window.
//! * The main loop services USB CDC-ACM, decodes host frames, sends replies
//!   and periodic status, and feeds the hardware watchdog.
//! * Time base: the RP2040's 64-bit 1 MHz `TIMER`.
//!
//! Single core on purpose: one tick of six axes plus USB fits comfortably in
//! core 0 (to be confirmed by the hardware measurements in docs/rp2040.md),
//! and the shared `McuCore` keeps commands and motion in one owner, so no
//! cross-core queues are needed. Core 1 stays parked.
//!
//! Safety notes:
//! * Driver ENABLE is active-low and must have an external pull-up so that a
//!   reset, watchdog reset or unprogrammed MCU leaves the drives disabled.
//! * This firmware's software E-stop does **not** replace a hardware E-stop
//!   circuit that removes driver power independently of all software.
#![no_std]
#![no_main]

use core::cell::RefCell;

use critical_section::Mutex;
use motioncore_firmware_core::{BoardInfo, McuCore};
use motioncore_protocol::{encode_frame, FrameDecoder, HostFrame, MAX_FRAME};
use panic_halt as _;
use rp2040_hal::{
    self as hal,
    clocks::Clock as _,
    fugit::MicrosDurationU32,
    pac::{self, interrupt},
    timer::{Alarm, Alarm0, Instant},
    usb::UsbBus,
    Timer, Watchdog,
};
use usb_device::{bus::UsbBusAllocator, prelude::*};
use usbd_serial::SerialPort;

mod board;
mod pio;

use board::{BoardHal, AXES, SYS_CLOCK_HZ, TICK_US};

/// Second-stage bootloader for the Pico's W25Q080 flash.
#[link_section = ".boot2"]
#[used]
pub static BOOT2_FIRMWARE: [u8; 256] = rp2040_boot2::BOOT_LOADER_W25Q080;

const XTAL_HZ: u32 = 12_000_000;
const QUEUE: usize = 32;
const STATUS_PERIOD_US: u64 = 50_000;
/// Segments due more than this late, or a tick gap this long during motion,
/// fault with `SegmentLate`. Ten tick periods.
const LATE_TOLERANCE_US: u64 = 10 * TICK_US as u64;

const FIRMWARE_VERSION: (u8, u8, u8) = (
    parse_u8(env!("CARGO_PKG_VERSION_MAJOR")),
    parse_u8(env!("CARGO_PKG_VERSION_MINOR")),
    parse_u8(env!("CARGO_PKG_VERSION_PATCH")),
);

const fn parse_u8(s: &str) -> u8 {
    let b = s.as_bytes();
    let mut v: u8 = 0;
    let mut i = 0;
    while i < b.len() {
        v = v * 10 + (b[i] - b'0');
        i += 1;
    }
    v
}

fn build_hash() -> u32 {
    match option_env!("MOTIONCORE_BUILD_HASH") {
        Some(s) => parse_hex(s),
        None => 0,
    }
}

const fn parse_hex(s: &str) -> u32 {
    let b = s.as_bytes();
    let mut v: u32 = 0;
    let mut i = 0;
    while i < b.len() && i < 8 {
        let d = match b[i] {
            c @ b'0'..=b'9' => c - b'0',
            c @ b'a'..=b'f' => c - b'a' + 10,
            c @ b'A'..=b'F' => c - b'A' + 10,
            _ => 0,
        };
        v = (v << 4) | d as u32;
        i += 1;
    }
    v
}

struct Shared {
    core: McuCore<AXES, QUEUE>,
    hal: BoardHal,
    timer: Timer,
    alarm: Alarm0,
    next_tick_us: u64,
}

static SHARED: Mutex<RefCell<Option<Shared>>> = Mutex::new(RefCell::new(None));

#[hal::entry]
fn main() -> ! {
    let mut pac = pac::Peripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);

    // 12 MHz crystal -> 125 MHz system clock, 48 MHz USB clock.
    let clocks = hal::clocks::init_clocks_and_plls(
        XTAL_HZ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();
    // The PIO divider and pulse timing assume this clock exactly.
    assert_eq!(clocks.system_clock.freq().to_Hz(), SYS_CLOCK_HZ);

    // Bring the PIO blocks out of reset; the step driver programs them at
    // register level, so the HAL's state-machine handles are not used.
    let _ = hal::pio::PIOExt::split(pac.PIO0, &mut pac.RESETS);
    let _ = hal::pio::PIOExt::split(pac.PIO1, &mut pac.RESETS);

    let sio = hal::Sio::new(pac.SIO);
    let pins = hal::gpio::Pins::new(pac.IO_BANK0, pac.PADS_BANK0, sio.gpio_bank0, &mut pac.RESETS);
    let board_hal = BoardHal::new(pins); // drives disabled

    // Hardware watchdog: if the main loop stalls, the MCU resets and the
    // external pull-ups disable the drivers.
    watchdog.pause_on_debug(true);
    watchdog.start(MicrosDurationU32::millis(100));

    // Timer before USB: the USB bus takes ownership of the USB clock.
    let mut timer = Timer::new(pac.TIMER, &mut pac.RESETS, &clocks);
    let mut alarm = timer.alarm_0().unwrap();

    // USB CDC-ACM.
    let usb_bus: &'static UsbBusAllocator<UsbBus> = cortex_m::singleton!(: UsbBusAllocator<UsbBus> = UsbBusAllocator::new(UsbBus::new(
        pac.USBCTRL_REGS,
        pac.USBCTRL_DPRAM,
        clocks.usb_clock,
        true,
        &mut pac.RESETS,
    )))
    .unwrap();
    let mut serial = SerialPort::new(usb_bus);
    let mut usb_dev = UsbDeviceBuilder::new(usb_bus, UsbVidPid(0x16c0, 0x27dd))
        .strings(&[StringDescriptors::default()
            .manufacturer("MotionCore")
            .product("MotionCore MCU (rp2040)")
            .serial_number("MC-RP2040")])
        .unwrap()
        .device_class(usbd_serial::USB_CLASS_CDC)
        .build();

    let core = McuCore::new(BoardInfo {
        target: "rp2040",
        board: "rp2040-pico",
        firmware_version: FIRMWARE_VERSION,
        build_hash: build_hash(),
        capabilities: board::CAPABILITIES,
        late_tolerance_us: LATE_TOLERANCE_US,
    });

    let next_tick_us = timer.get_counter().ticks() + TICK_US as u64;
    let _ = alarm.schedule_at(Instant::from_ticks(next_tick_us));
    alarm.enable_interrupt();

    critical_section::with(|cs| {
        SHARED.borrow(cs).replace(Some(Shared { core, hal: board_hal, timer, alarm, next_tick_us }));
    });
    // SAFETY: the shared state is initialised before the interrupt is unmasked.
    unsafe { pac::NVIC::unmask(pac::Interrupt::TIMER_IRQ_0) };

    let mut decoder = FrameDecoder::new();
    let mut rx = [0u8; 64];
    let mut tx = [0u8; 512];
    let mut tx_len = 0usize;
    let mut next_status = 0u64;

    loop {
        watchdog.feed();

        if usb_dev.poll(&mut [&mut serial]) {
            if let Ok(n) = serial.read(&mut rx) {
                for &b in &rx[..n] {
                    if let Some(Ok(frame)) = decoder.push::<HostFrame>(b) {
                        critical_section::with(|cs| {
                            if let Some(s) = SHARED.borrow(cs).borrow_mut().as_mut() {
                                let now = s.timer.get_counter().ticks();
                                s.core.handle(frame, now, &mut s.hal);
                            }
                        });
                    }
                }
            }
        }

        // Periodic status + collect outgoing frames.
        critical_section::with(|cs| {
            if let Some(s) = SHARED.borrow(cs).borrow_mut().as_mut() {
                let now = s.timer.get_counter().ticks();
                if now >= next_status {
                    s.core.report_status(now, &mut s.hal);
                    next_status = now + STATUS_PERIOD_US;
                }
                while tx_len + MAX_FRAME <= tx.len() {
                    let Some(frame) = s.core.poll_outgoing() else { break };
                    if let Ok(n) = encode_frame(&frame, &mut tx[tx_len..]) {
                        tx_len += n;
                    }
                }
            }
        });

        if tx_len > 0 {
            match serial.write(&tx[..tx_len]) {
                Ok(n) => {
                    tx.copy_within(n..tx_len, 0);
                    tx_len -= n;
                }
                Err(UsbError::WouldBlock) => {}
                // Host not attached: discard; the MCU watchdog handles safety.
                Err(_) => tx_len = 0,
            }
        }
    }
}

#[interrupt]
fn TIMER_IRQ_0() {
    critical_section::with(|cs| {
        if let Some(s) = SHARED.borrow(cs).borrow_mut().as_mut() {
            s.alarm.clear_interrupt();
            let now = s.timer.get_counter().ticks();
            // Absolute schedule (no drift); if the interrupt ran late, realign.
            // The core faults on a tick gap during motion.
            s.next_tick_us += TICK_US as u64;
            if s.next_tick_us <= now {
                s.next_tick_us = now + TICK_US as u64;
            }
            let _ = s.alarm.schedule_at(Instant::from_ticks(s.next_tick_us));
            s.core.tick(now, &mut s.hal);
        }
    });
}
