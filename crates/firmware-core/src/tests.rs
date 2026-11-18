use super::*;
use motioncore_protocol::{caps, AxisConfig, HomeParams, Segment};
use std::vec::Vec as StdVec;

#[derive(Default)]
struct SimHal {
    pos: [i64; 6],
    enabled: [bool; 6],
    /// Endstop triggers when physical position <= this value.
    endstop_at: [Option<i64>; 6],
    steps: [u64; 6],
    /// Simulated external safety interlock (E-stop chain) is open.
    interlock_open: bool,
    /// Number of `flush_pulses` calls.
    flushes: u64,
    /// Simulated pulse-hardware overrun.
    pulse_overrun: bool,
}

impl Hal for SimHal {
    fn step(&mut self, axis: u8, forward: bool) {
        assert!(self.enabled[axis as usize], "step on disabled axis {axis}");
        self.pos[axis as usize] += if forward { 1 } else { -1 };
        self.steps[axis as usize] += 1;
    }
    fn set_enable(&mut self, axis: u8, enabled: bool) {
        self.enabled[axis as usize] = enabled;
    }
    fn endstop_raw(&mut self, axis: u8) -> bool {
        matches!(self.endstop_at[axis as usize], Some(p) if self.pos[axis as usize] <= p)
    }
    fn interlock_ok(&mut self) -> bool {
        !self.interlock_open
    }
    fn flush_pulses(&mut self) -> bool {
        self.flushes += 1;
        !self.pulse_overrun
    }
}

type Core = McuCore<2, 16>;

fn core() -> Core {
    McuCore::new(BoardInfo {
        target: "test",
        board: "unit-test",
        firmware_version: (0, 1, 0),
        build_hash: 0xDEADBEEF,
        capabilities: caps::STEPPER_STEP_DIR | caps::ENDSTOP | caps::WATCHDOG,
        late_tolerance_us: DEFAULT_LATE_TOLERANCE_US,
    })
}

fn cfg(axis: u8) -> AxisConfig {
    AxisConfig { axis, min_steps: -10_000, max_steps: 10_000, max_rate: 5_000.0, stop_decel: 20_000.0, invert_dir: false, invert_endstop: false }
}

struct Rig {
    core: Core,
    hal: SimHal,
    now: u64,
    seq: u32,
}

impl Rig {
    fn new() -> Self {
        let mut r = Rig { core: core(), hal: SimHal::default(), now: 1_000, seq: 0 };
        r.cmd(HostMsg::ConfigureAxis(cfg(0)));
        r.cmd(HostMsg::Enable { axis: 0 });
        r.drain();
        r
    }

    fn cmd(&mut self, msg: HostMsg) {
        self.seq += 1;
        self.core.handle(HostFrame { seq: self.seq, msg }, self.now, &mut self.hal);
    }

    fn drain(&mut self) -> StdVec<McuMsg> {
        let mut v = StdVec::new();
        while let Some(f) = self.core.poll_outgoing() {
            v.push(f.msg);
        }
        v
    }

    /// Advance time in 100 µs ticks, sending heartbeats every 50 ms.
    fn run_for(&mut self, us: u64) {
        let end = self.now + us;
        while self.now < end {
            self.now += 100;
            if self.now % 50_000 < 100 {
                self.cmd(HostMsg::Heartbeat);
            }
            self.core.tick(self.now, &mut self.hal);
        }
    }

    fn last_ack_or_nack(&mut self) -> McuMsg {
        self.drain().into_iter().rev().find(|m| matches!(m, McuMsg::Ack { .. } | McuMsg::Nack { .. })).unwrap()
    }
}

/// Trapezoid-free single constant-velocity move split into contiguous segments.
fn const_move(start_us: u64, steps_per_seg: i32, n: usize, seg_us: u32) -> StdVec<Segment> {
    let v = steps_per_seg as f32 / (seg_us as f32 * 1e-6);
    (0..n)
        .map(|k| Segment {
            axis: 0,
            start_us: start_us + k as u64 * seg_us as u64,
            duration_us: seg_us,
            steps: steps_per_seg,
            v0: v,
            accel: 0.0,
            last: k == n - 1,
        })
        .collect()
}

#[test]
fn identify_reports_board() {
    let mut r = Rig::new();
    r.cmd(HostMsg::Identify);
    let msgs = r.drain();
    let McuMsg::Identity(id) = &msgs[0] else { panic!("{msgs:?}") };
    assert_eq!(id.protocol_version, PROTOCOL_VERSION);
    assert_eq!(id.axes, 2);
    assert_eq!(id.queue_capacity, 16);
    assert_eq!(id.target.as_str(), "test");
}

#[test]
fn executes_contiguous_segments_exactly() {
    let mut r = Rig::new();
    for s in const_move(r.now + 2_000, 20, 10, 10_000) {
        r.cmd(HostMsg::QueueSegment(s));
        assert!(matches!(r.last_ack_or_nack(), McuMsg::Ack { .. }));
    }
    r.run_for(150_000);
    assert_eq!(r.core.position(0), 200);
    assert_eq!(r.hal.pos[0], 200);
    assert_eq!(r.core.fault(), None);
    let st = r.core.status(r.now, &mut r.hal);
    assert!(!st.axes[0].moving);
    assert_eq!(st.queue_depth, 0);
}

#[test]
fn accelerating_segment_lands_on_integer_target() {
    let mut r = Rig::new();
    // a = 4000 steps/s², 0.1 s from rest: 20 steps.
    r.cmd(HostMsg::QueueSegment(Segment { axis: 0, start_us: r.now + 1_000, duration_us: 100_000, steps: 20, v0: 0.0, accel: 4000.0, last: true }));
    assert!(matches!(r.last_ack_or_nack(), McuMsg::Ack { .. }));
    r.run_for(50_000);
    let mid = r.core.position(0);
    assert!((4..=6).contains(&mid), "quadratic profile midpoint {mid}"); // 0.5*4000*0.049^2 ≈ 4.8
    r.run_for(100_000);
    assert_eq!(r.core.position(0), 20);
}

#[test]
fn rejects_inconsistent_or_unsafe_segments() {
    let mut r = Rig::new();
    let base = Segment { axis: 0, start_us: r.now + 1000, duration_us: 10_000, steps: 10, v0: 1000.0, accel: 0.0, last: true };

    r.cmd(HostMsg::QueueSegment(Segment { steps: 50, ..base }));
    assert!(matches!(r.last_ack_or_nack(), McuMsg::Nack { reason: NackReason::InvalidParams, .. }));

    r.cmd(HostMsg::QueueSegment(Segment { v0: 50_000.0, steps: 500, ..base }));
    assert!(matches!(r.last_ack_or_nack(), McuMsg::Nack { reason: NackReason::InvalidParams, .. }));

    r.cmd(HostMsg::QueueSegment(Segment { v0: f32::NAN, ..base }));
    assert!(matches!(r.last_ack_or_nack(), McuMsg::Nack { reason: NackReason::InvalidParams, .. }));

    r.cmd(HostMsg::SetPosition { axis: 0, steps: 9_995 });
    r.cmd(HostMsg::QueueSegment(base));
    assert!(matches!(r.last_ack_or_nack(), McuMsg::Nack { reason: NackReason::SoftLimit, .. }));

    r.cmd(HostMsg::QueueSegment(Segment { axis: 7, ..base }));
    assert!(matches!(r.last_ack_or_nack(), McuMsg::Nack { reason: NackReason::UnknownAxis, .. }));
}

#[test]
fn disabled_axis_rejects_motion() {
    let mut r = Rig::new();
    r.cmd(HostMsg::Disable { axis: 0 });
    r.cmd(HostMsg::QueueSegment(const_move(r.now + 1000, 10, 1, 10_000)[0]));
    assert!(matches!(r.last_ack_or_nack(), McuMsg::Nack { reason: NackReason::InvalidState, .. }));
}

#[test]
fn queue_underrun_faults_and_disables() {
    let mut r = Rig::new();
    let mut segs = const_move(r.now + 1000, 20, 3, 10_000);
    segs.pop(); // drop the `last` segment: motion would continue but data is missing
    for s in segs {
        r.cmd(HostMsg::QueueSegment(s));
    }
    r.run_for(50_000);
    assert_eq!(r.core.fault(), Some(FaultCode::QueueUnderrun));
    assert!(!r.hal.enabled[0]);
    assert!(r.drain().iter().any(|m| matches!(m, McuMsg::Fault { code: FaultCode::QueueUnderrun, axis: Some(0) })));
}

#[test]
fn watchdog_trips_without_heartbeat() {
    let mut r = Rig::new();
    let t0 = r.now;
    // Tick without heartbeats while the drive is enabled.
    while r.now < t0 + 600_000 {
        r.now += 1000;
        r.core.tick(r.now, &mut r.hal);
    }
    assert_eq!(r.core.fault(), Some(FaultCode::CommunicationLost));
    assert!(!r.hal.enabled[0]);
    // Faulted core refuses motion and enable until reset.
    r.cmd(HostMsg::Enable { axis: 0 });
    assert!(matches!(r.last_ack_or_nack(), McuMsg::Nack { reason: NackReason::Faulted, .. }));
    r.cmd(HostMsg::ResetFault);
    assert_eq!(r.core.fault(), None);
    assert!(!r.hal.enabled[0], "reset must not re-enable drives");
}

#[test]
fn watchdog_ignored_when_all_drives_disabled() {
    let mut r = Rig::new();
    r.cmd(HostMsg::Disable { axis: 0 });
    r.now += 2_000_000;
    r.core.tick(r.now, &mut r.hal);
    assert_eq!(r.core.fault(), None);
}

#[test]
fn late_segment_faults() {
    let mut r = Rig::new();
    let s = const_move(r.now + 1000, 20, 1, 10_000)[0];
    r.cmd(HostMsg::QueueSegment(s));
    // Jump far past the start time without ticking.
    r.now += 50_000;
    r.cmd(HostMsg::Heartbeat);
    r.core.tick(r.now, &mut r.hal);
    assert_eq!(r.core.fault(), Some(FaultCode::SegmentLate));
}

#[test]
fn homing_finds_endstop_and_sets_position() {
    let mut r = Rig::new();
    r.hal.pos[0] = 300;
    r.hal.endstop_at[0] = Some(0);
    r.cmd(HostMsg::Home(HomeParams { axis: 0, positive: false, speed: 2000.0, max_travel_steps: 1000, home_steps: -5000 }));
    assert!(matches!(r.last_ack_or_nack(), McuMsg::Ack { .. }));
    r.run_for(300_000);
    assert_eq!(r.core.fault(), None);
    assert!(r.drain().iter().any(|m| matches!(m, McuMsg::HomingDone { axis: 0, position_steps: -5000 })));
    assert_eq!(r.core.position(0), -5000);
    assert_eq!(r.hal.pos[0], 0);

    // Moving away from the switch is allowed.
    for s in const_move(r.now + 1000, 10, 2, 10_000) {
        r.cmd(HostMsg::QueueSegment(s));
    }
    r.run_for(50_000);
    assert_eq!(r.core.fault(), None);
    assert_eq!(r.core.position(0), -4980);
}

#[test]
fn homing_without_trigger_fails() {
    let mut r = Rig::new();
    r.cmd(HostMsg::Home(HomeParams { axis: 0, positive: false, speed: 4000.0, max_travel_steps: 100, home_steps: 0 }));
    r.run_for(200_000);
    assert_eq!(r.core.fault(), Some(FaultCode::HomingFailed));
    assert!(!r.hal.enabled[0]);
}

#[test]
fn endstop_hit_during_motion_faults() {
    let mut r = Rig::new();
    r.hal.endstop_at[0] = Some(-15);
    for s in const_move(r.now + 1000, -20, 3, 10_000) {
        r.cmd(HostMsg::QueueSegment(s));
    }
    r.run_for(60_000);
    assert_eq!(r.core.fault(), Some(FaultCode::LimitHit));
    assert_eq!(r.hal.pos[0], -15, "no step past the switch");
}

#[test]
fn controlled_stop_decelerates_and_stays_enabled() {
    let mut r = Rig::new();
    // 4000 steps/s cruise for 1 s.
    for s in const_move(r.now + 1000, 40, 100, 10_000) {
        r.cmd(HostMsg::QueueSegment(s));
        if r.core.status(r.now, &mut r.hal).queue_depth as usize >= 16 {
            break;
        }
    }
    r.run_for(50_000);
    let before = r.core.position(0);
    r.cmd(HostMsg::Stop);
    r.run_for(400_000);
    let after = r.core.position(0);
    // v²/(2a) = 4000² / 40000 = 400 steps of braking distance.
    let braking = after - before;
    assert!((380..=420).contains(&braking), "braking distance {braking}");
    assert_eq!(r.core.fault(), None);
    assert!(r.hal.enabled[0]);
    assert!(!r.core.status(r.now, &mut r.hal).axes[0].moving);
}

#[test]
fn emergency_stop_latches_and_disables() {
    let mut r = Rig::new();
    for s in const_move(r.now + 1000, 20, 5, 10_000) {
        r.cmd(HostMsg::QueueSegment(s));
    }
    r.run_for(20_000);
    let p = r.hal.pos[0];
    r.cmd(HostMsg::EmergencyStop);
    r.run_for(100_000);
    assert_eq!(r.hal.pos[0], p, "no steps after E-stop");
    assert_eq!(r.core.fault(), Some(FaultCode::EmergencyStop));
    assert!(!r.hal.enabled[0]);
}

#[test]
fn limit_fault_reset_requires_released_endstop() {
    let mut r = Rig::new();
    r.hal.endstop_at[0] = Some(-15);
    for s in const_move(r.now + 1000, -20, 3, 10_000) {
        r.cmd(HostMsg::QueueSegment(s));
    }
    r.run_for(60_000);
    assert_eq!(r.core.fault(), Some(FaultCode::LimitHit));
    r.drain();
    // Still on the switch and the safe direction is unknown: refused.
    r.cmd(HostMsg::ResetFault);
    assert!(matches!(r.last_ack_or_nack(), McuMsg::Nack { reason: NackReason::InvalidState, .. }));
    assert_eq!(r.core.fault(), Some(FaultCode::LimitHit));
    // Moved off the switch by hand: accepted, drive stays disabled.
    r.hal.pos[0] = 0;
    r.cmd(HostMsg::ResetFault);
    assert!(matches!(r.last_ack_or_nack(), McuMsg::Ack { .. }));
    assert_eq!(r.core.fault(), None);
    assert!(!r.hal.enabled[0]);
}

#[test]
fn open_interlock_trips_and_blocks_reset_and_enable() {
    let mut r = Rig::new();
    r.hal.interlock_open = true;
    r.run_for(1_000);
    assert_eq!(r.core.fault(), Some(FaultCode::EmergencyStop));
    assert!(!r.hal.enabled[0]);
    r.drain();
    r.cmd(HostMsg::ResetFault);
    assert!(matches!(r.last_ack_or_nack(), McuMsg::Nack { reason: NackReason::InvalidState, .. }));
    // Interlock restored: reset is accepted, enable works again.
    r.hal.interlock_open = false;
    r.cmd(HostMsg::ResetFault);
    assert!(matches!(r.last_ack_or_nack(), McuMsg::Ack { .. }));
    assert!(!r.hal.enabled[0], "reset must not re-enable drives");
    // Enable is refused (and latches a fault) while the interlock is open.
    r.hal.interlock_open = true;
    r.cmd(HostMsg::Enable { axis: 0 });
    assert!(matches!(r.last_ack_or_nack(), McuMsg::Nack { reason: NackReason::Faulted, .. }));
    assert!(!r.hal.enabled[0]);
}

#[test]
fn stalled_tick_during_motion_faults_instead_of_bursting() {
    let mut r = Rig::new();
    for s in const_move(r.now + 1000, 40, 5, 10_000) {
        r.cmd(HostMsg::QueueSegment(s));
    }
    r.run_for(15_000);
    assert_eq!(r.core.fault(), None);
    let p = r.hal.pos[0];
    // The tick timer stalls for longer than the late tolerance mid-move.
    r.now += DEFAULT_LATE_TOLERANCE_US + 1_000;
    r.cmd(HostMsg::Heartbeat);
    r.core.tick(r.now, &mut r.hal);
    assert_eq!(r.core.fault(), Some(FaultCode::SegmentLate));
    assert_eq!(r.hal.pos[0], p, "no catch-up burst");
    assert!(!r.hal.enabled[0]);
}

#[test]
fn pulses_are_flushed_once_per_tick() {
    let mut r = Rig::new();
    let before = r.hal.flushes;
    r.run_for(10_000); // 100 ticks of 100 µs
    assert_eq!(r.hal.flushes - before, 100);
}

#[test]
fn pulse_hardware_overrun_faults_and_disables() {
    let mut r = Rig::new();
    for s in const_move(r.now + 1000, 20, 3, 10_000) {
        r.cmd(HostMsg::QueueSegment(s));
    }
    r.run_for(5_000);
    assert_eq!(r.core.fault(), None);
    r.hal.pulse_overrun = true;
    r.run_for(200);
    assert_eq!(r.core.fault(), Some(FaultCode::StepRateExceeded));
    assert!(!r.hal.enabled[0]);
    assert!(r.drain().iter().any(|m| matches!(m, McuMsg::Fault { code: FaultCode::StepRateExceeded, axis: None })));
}

/// A six-axis coordinated move planned by the host planner, fed in time
/// order with at most 28 segments queued per axis (as the engine does), runs
/// every axis on the shared MCU clock: they start in the same tick, stay at
/// the same fraction of their distance, and finish in the same tick.
#[test]
fn six_axis_coordinated_move_runs_in_lockstep() {
    use motioncore_motion::{plan_synchronized, to_segments, AxisLimits};
    let mut core: McuCore<6, 32> = McuCore::new(BoardInfo {
        target: "test",
        board: "unit-test",
        firmware_version: (0, 1, 0),
        build_hash: 0,
        capabilities: caps::STEPPER_STEP_DIR,
        late_tolerance_us: DEFAULT_LATE_TOLERANCE_US,
    });
    let mut hal = SimHal::default();
    let mut now = 1_000u64;
    let mut seq = 0;
    let mut send = |core: &mut McuCore<6, 32>, hal: &mut SimHal, now: u64, msg: HostMsg| {
        seq += 1;
        core.handle(HostFrame { seq, msg }, now, hal);
        core.poll_outgoing().map(|f| f.msg)
    };
    let spu = [444.4, 266.7, 266.7, 213.3, 266.7, 444.4];
    for a in 0..6u8 {
        let c = AxisConfig { axis: a, min_steps: -400_000, max_steps: 400_000, max_rate: 60_000.0, ..cfg(a) };
        assert!(matches!(send(&mut core, &mut hal, now, HostMsg::ConfigureAxis(c)), Some(McuMsg::Ack { .. })));
        assert!(matches!(send(&mut core, &mut hal, now, HostMsg::Enable { axis: a }), Some(McuMsg::Ack { .. })));
    }
    let lim = |v: f64, a: f64| AxisLimits { min: -720.0, max: 720.0, max_velocity: v, max_acceleration: a };
    let limits = [lim(60.0, 150.0), lim(60.0, 100.0), lim(60.0, 200.0), lim(60.0, 200.0), lim(60.0, 200.0), lim(90.0, 200.0)];
    let start = [0.0; 6];
    let goal = [-30.0, 20.0, -60.0, 75.0, 40.0, -180.0];
    let plan = plan_synchronized(&start, &goal, &limits).unwrap();
    let t0 = now + 50_000;
    let mut segs: StdVec<Segment> = plan.iter().enumerate().flat_map(|(i, p)| to_segments(p, i as u8, spu[i], t0, 20_000)).collect();
    segs.sort_by_key(|s| (s.start_us, s.axis));
    let target: StdVec<i64> = (0..6).map(|i| (goal[i] * spu[i]).round() as i64).collect();
    let end_us = segs.iter().map(|s| s.end_us()).max().unwrap();

    let mut next = 0;
    let mut in_flight: StdVec<(u8, u64)> = StdVec::new();
    let mut first_step: [Option<u64>; 6] = [None; 6];
    let mut arrived: [Option<u64>; 6] = [None; 6];
    let mut worst = 0.0f64;
    while now < end_us + 20_000 {
        // Host side: refill every 5 ms.
        if now.is_multiple_of(5_000) {
            in_flight.retain(|&(_, e)| e >= now);
            while next < segs.len() {
                let s = segs[next];
                if in_flight.iter().filter(|(a, _)| *a == s.axis).count() >= 28 {
                    break;
                }
                assert!(matches!(send(&mut core, &mut hal, now, HostMsg::QueueSegment(s)), Some(McuMsg::Ack { .. })), "{s:?}");
                in_flight.push((s.axis, s.end_us()));
                next += 1;
            }
            send(&mut core, &mut hal, now, HostMsg::Heartbeat);
        }
        now += 100;
        core.tick(now, &mut hal);
        assert_eq!(core.fault(), None, "at {now}");
        let f: StdVec<f64> = (0..6).map(|i| hal.pos[i] as f64 / target[i] as f64).collect();
        for i in 0..6 {
            if first_step[i].is_none() && hal.pos[i] != 0 {
                first_step[i] = Some(now);
            }
            if arrived[i].is_none() && hal.pos[i] == target[i] {
                arrived[i] = Some(now);
            }
            // Fraction of the move, compared with the axis that has the most
            // steps; slack of two steps of this axis.
            worst = worst.max((f[i] - f[5]).abs() - 2.0 / target[i].abs() as f64);
        }
    }
    assert!(worst <= 0.0, "axes drifted apart by {worst}");
    assert_eq!((0..6).map(|i| hal.pos[i]).collect::<StdVec<_>>(), target);
    let span = |v: &[Option<u64>; 6]| v.iter().map(|t| t.unwrap()).max().unwrap() - v.iter().map(|t| t.unwrap()).min().unwrap();
    // Slowest axis needs a few ms for its first step under acceleration.
    assert!(span(&first_step) <= 15_000, "start spread {} us: {first_step:?}", span(&first_step));
    assert!(span(&arrived) <= 15_000, "finish spread {} us: {arrived:?}", span(&arrived));
    assert!(arrived.iter().all(|t| t.unwrap() <= end_us + 100));
}
