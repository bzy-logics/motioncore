//! Cycle-accurate model of one RP2040 PIO state machine, covering the
//! instructions the step program uses (PULL, OUT, MOV, JMP; 1 mandatory
//! side-set bit; delays). It executes the *encoded* program words, so the
//! tests check the instruction encoding as well as the timing arithmetic.
//!
//! Semantics follow the RP2040 datasheet section 3.4: side-set is asserted when an
//! instruction starts (also while it stalls), delay cycles follow a completed
//! instruction, `JMP X--`/`JMP Y--` test the register before decrementing,
//! and a blocking PULL on an empty FIFO stalls without consuming cycles of
//! delay.

use std::collections::VecDeque;

use crate::Program;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Edge {
    pub cycle: u64,
    pub step: bool,
    pub dir: bool,
}

pub struct Sm {
    code: [u16; 32],
    wrap_source: u8,
    wrap_target: u8,
    pc: u8,
    x: u32,
    y: u32,
    isr: u32,
    osr: u32,
    delay: u32,
    pub fifo: VecDeque<u32>,
    pub step: bool,
    pub dir: bool,
    pub cycle: u64,
    pub edges: Vec<Edge>,
    /// True while stalled on an empty FIFO at the PULL.
    pub stalled: bool,
    /// Net steps emitted (rising edge with DIR high = +1).
    pub position: i64,
}

impl Sm {
    pub fn new(p: &Program) -> Self {
        let mut code = [0u16; 32];
        code[..p.code.len()].copy_from_slice(&p.code);
        Self {
            code,
            wrap_source: p.wrap_source,
            wrap_target: p.wrap_target,
            pc: p.wrap_target,
            x: 0,
            y: 0,
            isr: 0,
            osr: 0,
            delay: 0,
            fifo: VecDeque::new(),
            step: false,
            dir: false,
            cycle: 0,
            edges: Vec::new(),
            stalled: false,
            position: 0,
        }
    }

    /// Forced stop as the board driver does it: drop queued commands, force
    /// STEP low and park at the program start.
    pub fn abort(&mut self) {
        self.fifo.clear();
        self.delay = 0;
        self.x = 0;
        self.y = 0;
        self.pc = self.wrap_target;
        self.stalled = false;
        let dir = self.dir;
        self.set_pins(false, dir);
    }

    fn set_pins(&mut self, step: bool, dir: bool) {
        if step && !self.step {
            self.position += if dir { 1 } else { -1 };
        }
        if step != self.step || dir != self.dir {
            self.step = step;
            self.dir = dir;
            self.edges.push(Edge { cycle: self.cycle, step, dir });
        }
    }

    fn advance(&mut self) {
        self.pc = if self.pc == self.wrap_source { self.wrap_target } else { (self.pc + 1) & 31 };
    }

    /// Run one state-machine clock cycle.
    pub fn clock(&mut self) {
        if self.delay > 0 {
            self.delay -= 1;
            self.cycle += 1;
            return;
        }
        let ins = self.code[self.pc as usize];
        let side = (ins >> 12) & 1 == 1;
        let delay = ((ins >> 8) & 0xF) as u32;
        let dir = self.dir;
        self.set_pins(side, dir);
        let mut jumped = false;
        match ins >> 13 {
            0b000 => {
                let addr = (ins & 0x1F) as u8;
                let take = match (ins >> 5) & 7 {
                    0b000 => true,
                    0b010 => {
                        let t = self.x != 0;
                        self.x = self.x.wrapping_sub(1);
                        t
                    }
                    0b100 => {
                        let t = self.y != 0;
                        self.y = self.y.wrapping_sub(1);
                        t
                    }
                    c => panic!("JMP condition {c:#b} not modelled"),
                };
                if take {
                    self.pc = addr;
                    jumped = true;
                }
            }
            0b011 => {
                let n = match ins & 0x1F {
                    0 => 32,
                    n => n as u32,
                };
                let v = if n == 32 { self.osr } else { self.osr & ((1u32 << n) - 1) };
                self.osr = if n == 32 { 0 } else { self.osr >> n };
                match (ins >> 5) & 7 {
                    0b000 => {
                        let step = self.step;
                        self.set_pins(step, v & 1 == 1);
                    }
                    0b001 => self.x = v,
                    0b110 => self.isr = v,
                    d => panic!("OUT destination {d:#b} not modelled"),
                }
            }
            0b100 => {
                assert!(ins & 0x80 != 0, "PUSH not modelled");
                assert!(ins & 0x20 != 0, "non-blocking PULL not modelled");
                match self.fifo.pop_front() {
                    Some(w) => {
                        self.osr = w;
                        self.stalled = false;
                    }
                    None => {
                        // Stall: retry next cycle, no delay, no advance.
                        self.stalled = true;
                        self.cycle += 1;
                        return;
                    }
                }
            }
            0b101 => {
                let src = match ins & 7 {
                    0b001 => self.x,
                    0b010 => self.y,
                    0b110 => self.isr,
                    s => panic!("MOV source {s:#b} not modelled"),
                };
                match (ins >> 5) & 7 {
                    0b001 => self.x = src,
                    0b010 => self.y = src,
                    d => panic!("MOV destination {d:#b} not modelled"),
                }
            }
            op => panic!("opcode {op:#b} not modelled"),
        }
        if !jumped {
            self.advance();
        }
        self.delay = delay;
        self.cycle += 1;
    }

    /// Run until the FIFO is empty and the program waits at the PULL again.
    pub fn run_until_idle(&mut self, limit: u64) {
        let start = self.cycle;
        loop {
            self.clock();
            if self.stalled && self.fifo.is_empty() {
                return;
            }
            assert!(self.cycle - start < limit, "state machine did not go idle");
        }
    }
}

/// One STEP pulse as seen on the pins.
#[derive(Debug, Clone, Copy)]
pub struct Pulse {
    pub rise: u64,
    pub fall: u64,
    pub dir: bool,
}

pub fn pulses(edges: &[Edge]) -> Vec<Pulse> {
    let mut out = Vec::new();
    let mut prev_step = false;
    let mut rise = None;
    for e in edges {
        if e.step && !prev_step {
            rise = Some((e.cycle, e.dir));
        }
        if !e.step && prev_step {
            if let Some((r, d)) = rise.take() {
                out.push(Pulse { rise: r, fall: e.cycle, dir: d });
            }
        }
        prev_step = e.step;
    }
    out
}

/// Cycles of every DIR change.
pub fn dir_changes(edges: &[Edge]) -> Vec<u64> {
    let mut out = Vec::new();
    let mut prev = false;
    for e in edges {
        if e.dir != prev {
            out.push(e.cycle);
            prev = e.dir;
        }
    }
    out
}

/// Every pin-timing guarantee: high time, minimum low time, DIR setup and
/// hold, DIR never changing while STEP is high.
pub fn check_constraints(t: &crate::Timing, edges: &[Edge]) {
    let ps = pulses(edges);
    for p in &ps {
        assert_eq!(p.fall - p.rise, t.high_cycles as u64, "high time");
    }
    for w in ps.windows(2) {
        assert!(w[1].rise - w[0].fall >= t.min_low_cycles as u64, "low time {}", w[1].rise - w[0].fall);
    }
    for &c in &dir_changes(edges) {
        if let Some(next) = ps.iter().find(|p| p.rise >= c) {
            assert!(next.rise - c >= t.dir_setup_cycles as u64, "dir setup {}", next.rise - c);
        }
        if let Some(prev) = ps.iter().rev().find(|p| p.fall <= c) {
            assert!(c - prev.fall >= t.min_low_cycles as u64, "dir hold {}", c - prev.fall);
        }
        assert!(ps.iter().all(|p| !(p.rise < c && c < p.fall)), "DIR changed while STEP high");
    }
}
