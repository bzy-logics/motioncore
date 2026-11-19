//! MotionCore firmware for the STM32F411CEU6 "Black Pill" (provisional MVP board).
//!
//! * `TIM3` update interrupt at [`TICK_HZ`] drives [`McuCore::tick`] (step generation,
//!   homing, controlled stop, communication watchdog).
//! * The main loop services USB CDC-ACM, decodes host frames, sends replies
//!   and periodic status, and feeds the hardware independent watchdog (IWDG).
//! * Time base: the DWT cycle counter extended to 64 bits.
//!
//! Safety notes:
//! * Driver ENABLE is active-low and must have an external pull-up so that a
//!   reset, hang (IWDG reset) or unprogrammed MCU leaves the drives disabled.
//! * This firmware's software E-stop does **not** replace a hardware E-stop
//!   circuit that removes driver power independently of all software.
#![no_std]
#![no_main]

use core::cell::RefCell;

use cortex_m_rt::entry;
use critical_section::Mutex;
use motioncore_firmware_core::{BoardInfo, McuCore, DEFAULT_LATE_TOLERANCE_US};
use motioncore_protocol::{encode_frame, FrameDecoder, HostFrame, MAX_FRAME};
use stm32f4xx_hal::{
    otg_fs::{UsbBus, USB},
    pac::{self, interrupt, TIM3},
    prelude::*,
    timer::{CounterHz, Event, Flag},
    watchdog::IndependentWatchdog,
};
use usb_device::{bus::UsbBusAllocator, prelude::*};
use usbd_serial::SerialPort;

mod board;

use board::{BoardHal, AXES};

const SYSCLK_MHZ: u32 = 96;
const FIRMWARE_VERSION: (u8, u8, u8) =
    (parse_u8(env!("CARGO_PKG_VERSION_MAJOR")), parse_u8(env!("CARGO_PKG_VERSION_MINOR")), parse_u8(env!("CARGO_PKG_VERSION_PATCH")));

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
const TICK_HZ: u32 = 20_000;
const QUEUE: usize = 32;
const STATUS_PERIOD_US: u64 = 50_000;

/// 64-bit microsecond clock from the 32-bit DWT cycle counter.
struct Clock {
    last: u32,
    high: u64,
}

impl Clock {
    fn now_us(&mut self) -> u64 {
        let cyc = cortex_m::peripheral::DWT::cycle_count();
        if cyc < self.last {
            self.high += 1 << 32;
        }
        self.last = cyc;
        (self.high | cyc as u64) / SYSCLK_MHZ as u64
    }
}

struct Shared {
    core: McuCore<AXES, QUEUE>,
    hal: BoardHal,
    clock: Clock,
    timer: CounterHz<TIM3>,
}

static SHARED: Mutex<RefCell<Option<Shared>>> = Mutex::new(RefCell::new(None));

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

#[entry]
fn main() -> ! {
    let dp = pac::Peripherals::take().unwrap();
    let mut cp = cortex_m::Peripherals::take().unwrap();

    // Black Pill: 25 MHz HSE -> 96 MHz SYSCLK, 48 MHz for USB.
    let rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr.use_hse(25.MHz()).sysclk(SYSCLK_MHZ.MHz()).require_pll48clk().freeze();

    cp.DCB.enable_trace();
    cp.DWT.enable_cycle_counter();

    let gpioa = dp.GPIOA.split();
    let gpiob = dp.GPIOB.split();
    let hal = BoardHal::new(
        gpioa.pa0,
        gpioa.pa1,
        gpioa.pa2,
        gpiob.pb0,
        #[cfg(any(feature = "bench-panic", feature = "bench-isr-probe"))]
        gpiob.pb1,
    );

    // Hardware watchdog: if the main loop stalls, the MCU resets and the
    // external pull-up disables the drivers.
    let mut iwdg = IndependentWatchdog::new(dp.IWDG);
    iwdg.start(100.millis());

    // USB CDC-ACM.
    let usb = USB::new((dp.OTG_FS_GLOBAL, dp.OTG_FS_DEVICE, dp.OTG_FS_PWRCLK), (gpioa.pa11, gpioa.pa12), &clocks);
    let ep_memory = cortex_m::singleton!(: [u32; 1024] = [0; 1024]).unwrap();
    let usb_bus: &'static UsbBusAllocator<UsbBus<USB>> = cortex_m::singleton!(: UsbBusAllocator<UsbBus<USB>> = UsbBus::new(usb, ep_memory)).unwrap();
    let mut serial = SerialPort::new(usb_bus);
    let mut usb_dev = UsbDeviceBuilder::new(usb_bus, UsbVidPid(0x16c0, 0x27dd))
        .strings(&[StringDescriptors::default().manufacturer("MotionCore").product("MotionCore MCU (stm32f411)").serial_number("MC-F411")])
        .unwrap()
        .device_class(usbd_serial::USB_CLASS_CDC)
        .build();

    let core = McuCore::new(BoardInfo {
        target: "stm32f4",
        board: "stm32f411-blackpill",
        firmware_version: FIRMWARE_VERSION,
        build_hash: build_hash(),
        capabilities: board::CAPABILITIES,
        late_tolerance_us: DEFAULT_LATE_TOLERANCE_US,
    });

    let mut timer = dp.TIM3.counter_hz(&clocks);
    timer.start(TICK_HZ.Hz()).unwrap();
    timer.listen(Event::Update);

    critical_section::with(|cs| {
        SHARED.borrow(cs).replace(Some(Shared { core, hal, clock: Clock { last: 0, high: 0 }, timer }));
    });
    // SAFETY: the shared state is initialised before the interrupt is unmasked.
    unsafe { cortex_m::peripheral::NVIC::unmask(pac::Interrupt::TIM3) };

    let mut decoder = FrameDecoder::new();
    let mut rx = [0u8; 64];
    let mut tx = [0u8; 512];
    let mut tx_len = 0usize;
    let mut next_status = 0u64;

    loop {
        iwdg.feed();

        if usb_dev.poll(&mut [&mut serial]) {
            if let Ok(n) = serial.read(&mut rx) {
                for &b in &rx[..n] {
                    if let Some(Ok(frame)) = decoder.push::<HostFrame>(b) {
                        critical_section::with(|cs| {
                            if let Some(s) = SHARED.borrow(cs).borrow_mut().as_mut() {
                                let now = s.clock.now_us();
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
                let now = s.clock.now_us();
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
fn TIM3() {
    critical_section::with(|cs| {
        if let Some(s) = SHARED.borrow(cs).borrow_mut().as_mut() {
            #[cfg(feature = "bench-isr-probe")]
            s.hal.isr_probe(true);
            s.timer.clear_flags(Flag::Update);
            let now = s.clock.now_us();
            s.core.tick(now, &mut s.hal);
            #[cfg(feature = "bench-panic")]
            if s.hal.panic_requested() {
                panic!("bench-panic trigger (PB1)");
            }
            #[cfg(feature = "bench-isr-probe")]
            s.hal.isr_probe(false);
        }
    });
}

/// Outputs go to their safe state immediately (driver disabled, STEP/DIR
/// low); the independent watchdog then resets the MCU.
#[panic_handler]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    board::safe_shutdown();
    loop {
        cortex_m::asm::nop();
    }
}
