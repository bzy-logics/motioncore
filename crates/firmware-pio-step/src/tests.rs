use super::*;
use crate::emu::{self, pulses, Edge, Sm};

/// The RP2040 board configuration (5 MHz SM clock, 2 µs high/low/setup).
const T: Timing = Timing { sm_clock_hz: 5_000_000, high_cycles: 10, min_low_cycles: 10, dir_setup_cycles: 10, window_budget_pct: 90 };
const WINDOW_US: u32 = 200;

/// Run `words` through the emulated program; returns edges and busy cycles.
fn run(words: &[u32]) -> (Vec<Edge>, u64) {
    let p = program(&T).unwrap();
    let mut sm = Sm::new(&p);
    sm.clock(); // stall at PULL on the empty FIFO
    let start = sm.cycle;
    sm.fifo.extend(words.iter().copied());
    sm.run_until_idle(10_000_000);
    (sm.edges.clone(), sm.cycle - start - 1)
}

fn check_constraints(edges: &[Edge]) {
    emu::check_constraints(&T, edges)
}

#[test]
fn program_encoding_is_stable() {
    let p = program(&T).unwrap();
    // high 10 -> H = 9; setup 10 -> S = 7.
    assert_eq!(p.code, [0x80A0, 0x6701, 0x602F, 0x60D0, 0xB942, 0xA046, 0x0086, 0x0044]);
    assert_eq!((p.wrap_source, p.wrap_target, p.side_set_bits), (7, 0, 1));
}

#[test]
fn invalid_timing_is_rejected() {
    assert_eq!(program(&Timing { high_cycles: 0, ..T }), Err(TimingError::HighCycles));
    assert_eq!(program(&Timing { high_cycles: 17, ..T }), Err(TimingError::HighCycles));
    assert_eq!(program(&Timing { min_low_cycles: 2, ..T }), Err(TimingError::MinLowCycles));
    assert_eq!(program(&Timing { dir_setup_cycles: 19, ..T }), Err(TimingError::DirSetupCycles));
    assert_eq!(program(&Timing { window_budget_pct: 0, ..T }), Err(TimingError::WindowBudget));
}

#[test]
fn jmp_targets_relocate_other_instructions_do_not() {
    let p = program(&T).unwrap();
    assert_eq!(relocate(p.code[6], 8), 0x0080 | 14);
    assert_eq!(relocate(p.code[7], 8), 0x0040 | 12);
    assert_eq!(relocate(p.code[0], 8), p.code[0]);
    assert_eq!(relocate(p.code[4], 8), p.code[4]);
}

#[test]
fn exact_step_count_and_even_spacing() {
    for n in [1u32, 2, 5, 17, 44] {
        let span = T.window_budget_cycles(WINDOW_US);
        let (word, cycles) = command(&T, true, n, span).unwrap();
        let (edges, busy) = run(&[word]);
        let ps = pulses(&edges);
        assert_eq!(ps.len() as u32, n, "step count");
        assert!(ps.iter().all(|p| p.dir), "direction");
        assert_eq!(busy, cycles as u64, "modelled duration matches the formula");
        assert!(cycles <= span, "fits the window budget");
        let periods: Vec<u64> = ps.windows(2).map(|w| w[1].rise - w[0].rise).collect();
        assert!(periods.windows(2).all(|w| w[0] == w[1]), "even spacing");
        check_constraints(&edges);
    }
}

#[test]
fn step_frequency_matches_request() {
    // 10 steps in a 200 µs window at 90 % budget -> one step per ~18 µs.
    let span = T.window_budget_cycles(WINDOW_US);
    let (word, _) = command(&T, false, 10, span).unwrap();
    let (edges, _) = run(&[word]);
    let ps = pulses(&edges);
    let period = ps[1].rise - ps[0].rise;
    let ideal = (span - T.command_overhead_cycles()) as u64 / 10;
    assert_eq!(period, ideal);
}

#[test]
fn direction_changes_keep_setup_and_hold() {
    let mut r = StepRuns::new();
    for _ in 0..3 {
        r.push(true);
    }
    for _ in 0..2 {
        r.push(false);
    }
    r.push(true);
    let cmds = r.take_commands(&T, WINDOW_US).unwrap();
    assert_eq!(cmds.words().len(), 3);
    assert!(cmds.cycles <= T.window_budget_cycles(WINDOW_US));
    let (edges, busy) = run(cmds.words());
    assert_eq!(busy, cmds.cycles as u64);
    let dirs: Vec<bool> = pulses(&edges).iter().map(|p| p.dir).collect();
    assert_eq!(dirs, [true, true, true, false, false, true]);
    check_constraints(&edges);
    assert!(r.is_empty(), "buffer cleared");
}

#[test]
fn many_windows_back_to_back_keep_constraints() {
    // Deterministic pseudo-random step patterns over consecutive windows,
    // fed while the previous window drains (as the tick interrupt does).
    let p = program(&T).unwrap();
    let mut sm = Sm::new(&p);
    let window_cycles = (WINDOW_US as u64 * T.sm_clock_hz as u64) / 1_000_000;
    let mut seed = 0x1234_5678u32;
    let mut expected = Vec::new();
    for _ in 0..200 {
        seed = seed.wrapping_mul(1_103_515_245).wrapping_add(12_345);
        let n = (seed >> 16) % (T.max_steps_per_window(WINDOW_US) + 1);
        let fwd = seed & 0x100 != 0;
        let mut r = StepRuns::new();
        for _ in 0..n {
            r.push(fwd);
        }
        let cmds = r.take_commands(&T, WINDOW_US).unwrap();
        assert!(sm.fifo.is_empty(), "previous window finished in time");
        sm.fifo.extend(cmds.words().iter().copied());
        expected.extend(std::iter::repeat_n(fwd, n as usize));
        for _ in 0..window_cycles {
            sm.clock();
        }
    }
    sm.run_until_idle(1_000_000);
    let got: Vec<bool> = pulses(&sm.edges).iter().map(|p| p.dir).collect();
    assert_eq!(got, expected, "every step, in order, with its direction");
    check_constraints(&sm.edges);
}

#[test]
fn too_many_steps_for_the_window_is_an_error_not_a_burst() {
    let max = T.max_steps_per_window(WINDOW_US);
    assert_eq!(max, 44);
    let span = T.window_budget_cycles(WINDOW_US);
    assert!(command(&T, true, max, span).is_ok());
    assert_eq!(command(&T, true, max + 1, span), Err(PulseError::TooFast));
    let mut r = StepRuns::new();
    for _ in 0..=max {
        r.push(true);
    }
    assert_eq!(r.take_commands(&T, WINDOW_US), Err(PulseError::TooFast));
}

#[test]
fn too_many_direction_changes_overflow() {
    let mut r = StepRuns::new();
    for i in 0..=MAX_RUNS {
        r.push(i % 2 == 0);
    }
    assert_eq!(r.take_commands(&T, WINDOW_US), Err(PulseError::TooManyRuns));
    assert!(r.is_empty());
}

#[test]
fn long_windows_clamp_the_low_loop() {
    // One step in a 100 ms window would need L > 16 bits; it is clamped and
    // the pulse simply finishes early.
    let span = Timing { window_budget_pct: 100, ..T }.window_budget_cycles(100_000);
    let (word, cycles) = command(&T, true, 1, span).unwrap();
    assert_eq!(word >> 16, MAX_LOW_LOOP);
    assert!(cycles <= span);
}

#[test]
fn six_axes_fit_two_pio_blocks() {
    let mut used = std::collections::HashSet::new();
    for axis in 0..6u8 {
        let (block, sm) = sm_for_axis(axis);
        assert!(block < 2 && sm < 4);
        assert!(used.insert((block, sm)), "state machine shared");
    }
    // Program size per block leaves room for other programs.
    const _: () = assert!(PROGRAM_LEN <= 32);
}

#[test]
fn rate_limits() {
    assert_eq!(T.min_period_cycles(), 20);
    assert_eq!(T.max_step_rate(), 250_000);
}
