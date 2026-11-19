//! Register-level driver for the STEP/DIR state machines.
//!
//! Each axis owns one state machine (see
//! [`sm_for_axis`](motioncore_firmware_pio_step::sm_for_axis)). The program
//! from `motioncore-firmware-pio-step` is loaded at offset 0 of every PIO
//! block in use. The TX FIFO is joined (8 words); the RX FIFO is unused.
//!
//! All register access happens during initialisation or inside the shared
//! critical section (tick interrupt / command handling), so the
//! read-modify-write accesses to the per-block `CTRL` and `SHIFTCTRL`
//! registers never race.

use motioncore_firmware_pio_step::{relocate, Program, PROGRAM_LEN};
use rp2040_hal::pac;

type Block = pac::pio0::RegisterBlock;

const PROGRAM_OFFSET: u8 = 0;

// PIO instruction encodings used for forced execution.
const SET_PINDIRS_OUT: u32 = 0xE081; // set pindirs, 1
const SET_PINS_LOW: u32 = 0xE000; // set pins, 0
const NOP_SIDE_LOW: u32 = 0xA042; // mov y, y  side 0
const fn jmp(addr: u8) -> u32 {
    addr as u32 & 0x1F
}

// Register bit positions (RP2040 datasheet section 3.7).
const CTRL_SM_ENABLE: u32 = 0;
const CTRL_SM_RESTART: u32 = 4;
const CTRL_CLKDIV_RESTART: u32 = 8;
const FSTAT_TXFULL: u32 = 16;
const FSTAT_TXEMPTY: u32 = 24;
const SHIFTCTRL_FJOIN_TX: u32 = 1 << 30;
const SHIFTCTRL_OUT_SHIFTDIR_RIGHT: u32 = 1 << 19;

fn block(b: u8) -> &'static Block {
    // SAFETY: the PIO register blocks are always mapped. Ownership of the
    // step state machines is exclusive to `StepSm` (constructed only by
    // `BoardHal::new`), and all accesses are serialised as described above.
    unsafe {
        if b == 0 {
            &*pac::PIO0::ptr()
        } else {
            &*pac::PIO1::ptr()
        }
    }
}

/// Write `program` into the instruction memory of PIO block `b`.
pub fn load_program(b: u8, program: &Program) {
    let blk = block(b);
    for (i, &ins) in program.code.iter().enumerate().take(PROGRAM_LEN) {
        let ins = relocate(ins, PROGRAM_OFFSET) as u32;
        blk.instr_mem(PROGRAM_OFFSET as usize + i).write(|w| unsafe { w.bits(ins) });
    }
}

/// One STEP/DIR state machine.
pub struct StepSm {
    block: u8,
    sm: u8,
}

impl StepSm {
    /// Configure and start the state machine. STEP and DIR are driven low.
    /// The GPIOs must already be switched to the PIO function.
    pub fn new(block_id: u8, sm: u8, program: &Program, step_pin: u8, dir_pin: u8, clkdiv: u16) -> Self {
        let s = Self { block: block_id, sm };
        let blk = block(block_id);
        let r = blk.sm(sm as usize);
        s.set_enabled(false);
        r.sm_clkdiv().write(|w| unsafe { w.bits((clkdiv as u32) << 16) });
        let wrap_top = (PROGRAM_OFFSET + program.wrap_source) as u32;
        let wrap_bottom = (PROGRAM_OFFSET + program.wrap_target) as u32;
        r.sm_execctrl().write(|w| unsafe { w.bits((wrap_top << 12) | (wrap_bottom << 7)) });
        // OSR shifts right, no autopull, TX FIFO joined (8 deep). Writing
        // FJOIN also clears both FIFOs.
        r.sm_shiftctrl().write(|w| unsafe { w.bits(SHIFTCTRL_FJOIN_TX | SHIFTCTRL_OUT_SHIFTDIR_RIGHT) });
        // Pin directions and initial low level via forced SET instructions.
        for pin in [step_pin, dir_pin] {
            r.sm_pinctrl().write(|w| unsafe { w.bits((1 << 26) | ((pin as u32) << 5)) });
            s.exec(SET_PINS_LOW);
            s.exec(SET_PINDIRS_OUT);
        }
        // SIDESET_COUNT = 1 (STEP), OUT_COUNT = 1 (DIR).
        let pinctrl = (program.side_set_bits as u32) << 29 | (1 << 20) | ((step_pin as u32) << 10) | dir_pin as u32;
        r.sm_pinctrl().write(|w| unsafe { w.bits(pinctrl) });
        s.restart_at_start();
        s.set_enabled(true);
        s
    }

    fn exec(&self, ins: u32) {
        block(self.block).sm(self.sm as usize).sm_instr().write(|w| unsafe { w.bits(ins) });
    }

    fn set_enabled(&self, on: bool) {
        let bit = 1 << (CTRL_SM_ENABLE + self.sm as u32);
        block(self.block).ctrl().modify(|r, w| unsafe { w.bits(if on { r.bits() | bit } else { r.bits() & !bit }) });
    }

    fn restart_at_start(&self) {
        let bits = (1 << (CTRL_SM_RESTART + self.sm as u32)) | (1 << (CTRL_CLKDIV_RESTART + self.sm as u32));
        block(self.block).ctrl().modify(|r, w| unsafe { w.bits(r.bits() | bits) });
        self.exec(jmp(PROGRAM_OFFSET));
    }

    /// `true` if every queued command has been taken by the state machine.
    pub fn tx_empty(&self) -> bool {
        block(self.block).fstat().read().bits() & (1 << (FSTAT_TXEMPTY + self.sm as u32)) != 0
    }

    /// Queue one command word. `false` if the FIFO is full (nothing written).
    pub fn push(&self, word: u32) -> bool {
        let blk = block(self.block);
        if blk.fstat().read().bits() & (1 << (FSTAT_TXFULL + self.sm as u32)) != 0 {
            return false;
        }
        blk.txf(self.sm as usize).write(|w| unsafe { w.bits(word) });
        true
    }

    /// Immediate output inhibit: stop the state machine mid-pulse, force
    /// STEP low, drop every queued command and park at the PULL again.
    pub fn abort(&self) {
        self.set_enabled(false);
        self.exec(NOP_SIDE_LOW);
        let r = block(self.block).sm(self.sm as usize);
        // Toggling FJOIN_TX clears both FIFOs.
        let shift = r.sm_shiftctrl().read().bits();
        r.sm_shiftctrl().write(|w| unsafe { w.bits(shift ^ SHIFTCTRL_FJOIN_TX) });
        r.sm_shiftctrl().write(|w| unsafe { w.bits(shift) });
        self.restart_at_start();
        self.set_enabled(true);
    }
}
