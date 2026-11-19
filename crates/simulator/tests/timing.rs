//! Executed step timing against the plan (#24).
//!
//! Drives the shared firmware runtime ([`McuCore`], the same code the virtual
//! MCU and the boards run) with a **simulated clock** in fixed ticks, feeds it
//! the segments the real planners produce for the six-axis desktop arm
//! (`robot-profiles/dummy-arm-virtual.toml`, configured exactly as the engine
//! configures the MCU), and records every step. Deterministic: no OS timers.
//!
//! For every move type (MoveJ, MoveJPath with corners and reversals, MoveL)
//! each axis must stay within **±1 step and one tick** of the plan: at every
//! tick τ the executed position lies between the planned positions at τ − T
//! and τ, widened by one step. A MoveL plan is a sampled trajectory, exactly
//! defined only at its knots (every 10 ms), so it is checked there. The final
//! position is exact.
//!
//! Fault paths mid-move:
//! * Stop: every axis decelerates at its configured stop deceleration (the
//!   stopping distance is v²/(2·a) within two steps), never reverses, and
//!   stops for good.
//! * E-stop: no step after the trip; drives disabled.
//! * Link loss (no frames from the host): a fault latches within the
//!   watchdog timeout; no step after it; drives disabled.
//!
//! Per-axis deviations are printed (`cargo test -p motioncore-simulator
//! --test timing -- --nocapture`). The MoveJ capture is also written as a
//! CSV of STEP edges and checked with `tools/step-skew` against the ground
//! truth (all axes run from one clock, so the skew must be within a tick).

use std::path::{Path, PathBuf};

use motioncore_device_graph::{AxisSection, JointKind, ResolvedRobot};
use motioncore_firmware_core::{Hal, McuCore};
use motioncore_kinematics::chain::{Chain, ToolPose};
use motioncore_kinematics::collision::JointSpec;
use motioncore_motion::blend::plan_blended;
use motioncore_motion::sampled::{time_scale, Sampled};
use motioncore_motion::{plan_synchronized_scaled, to_segments, AxisLimits, Profile};
use motioncore_protocol::{AxisConfig, FaultCode, HostFrame, HostMsg, McuMsg, Segment};

/// Simulated tick: the firmware emits every step that became due since the
/// previous tick, so "one tick" is the timing resolution being verified.
const TICK_US: u64 = 100;
/// Engine constants (crates/core/src/engine.rs).
const LEAD_US: u64 = 100_000;
const MAX_SEGMENT_US: u32 = 20_000;
const LINE_STEP_MM: f64 = 0.5;
const LINE_STEP_DEG: f64 = 0.25;
const LINE_SPEED_MM_S: f64 = 50.0;
const LINE_ACCEL_MM_S2: f64 = 500.0;
const HEARTBEAT_US: u64 = 100_000;
const AXES: usize = 6;

// ------------------------------------------------------------------ rig

/// Physical world: step counters, enables, and a log of every step.
#[derive(Default)]
struct RecordingHal {
    now: u64,
    pos: [i64; AXES],
    enabled: [bool; AXES],
    /// (time, axis, +1 / −1)
    steps: Vec<(u64, usize, i8)>,
}

impl Hal for RecordingHal {
    fn step(&mut self, axis: u8, forward: bool) {
        let i = axis as usize;
        assert!(self.enabled[i], "step on disabled axis {i} at {} µs", self.now);
        self.pos[i] += if forward { 1 } else { -1 };
        self.steps.push((self.now, i, if forward { 1 } else { -1 }));
    }
    fn set_enable(&mut self, axis: u8, enabled: bool) {
        self.enabled[axis as usize] = enabled;
    }
    fn endstop_raw(&mut self, _axis: u8) -> bool {
        false
    }
}

struct Rig {
    robot: ResolvedRobot,
    core: McuCore<AXES, 32>,
    hal: RecordingHal,
    now: u64,
    seq: u32,
    faults: Vec<(u64, FaultCode)>,
    /// Executed positions after every tick.
    trace: Vec<(u64, [i64; AXES])>,
    last_heartbeat: u64,
}

fn robot() -> ResolvedRobot {
    let path = Path::new(env!("CARGO_MANIFEST_DIR")).join("../../robot-profiles/dummy-arm-virtual.toml");
    motioncore_device_graph::load(&path).expect("profile loads")
}

fn axes(robot: &ResolvedRobot) -> &[AxisSection] {
    &robot.file.axes
}

fn limits(a: &AxisSection) -> AxisLimits {
    AxisLimits { min: a.min, max: a.max, max_velocity: a.max_velocity, max_acceleration: a.max_acceleration }
}

impl Rig {
    /// Configure the MCU as the engine does for homed axes and put the arm at
    /// `pose` (step-quantised).
    fn new(pose: &[f64; AXES]) -> Self {
        let robot = robot();
        let mut r = Rig {
            core: McuCore::new(motioncore_simulator::board_info()),
            hal: RecordingHal::default(),
            now: 7_000_000,
            seq: 0,
            faults: Vec::new(),
            trace: Vec::new(),
            last_heartbeat: 7_000_000,
            robot,
        };
        let watchdog_ms = r.robot.file.mcu["main"].watchdog_ms;
        r.send(HostMsg::SetWatchdog { timeout_ms: watchdog_ms });
        for (i, a) in axes(&r.robot).to_vec().iter().enumerate() {
            let spu = a.steps_per_unit;
            let cfg = AxisConfig {
                axis: a.channel,
                min_steps: (a.min * spu).floor() as i32,
                max_steps: (a.max * spu).ceil() as i32,
                max_rate: (a.max_velocity * spu * 1.05) as f32,
                stop_decel: (a.stop_deceleration() * spu) as f32,
                invert_dir: a.invert_dir,
                invert_endstop: false,
            };
            assert_eq!(a.channel as usize, i, "test assumes channel = axis index");
            r.send(HostMsg::ConfigureAxis(cfg));
            let steps = (pose[i] * spu).round() as i32;
            r.send(HostMsg::SetPosition { axis: a.channel, steps });
            r.hal.pos[i] = steps as i64;
            r.send(HostMsg::Enable { axis: a.channel });
        }
        r
    }

    fn spu(&self, i: usize) -> f64 {
        axes(&self.robot)[i].steps_per_unit
    }

    /// Current pose in axis units (from the step counters).
    fn pose(&self) -> [f64; AXES] {
        std::array::from_fn(|i| self.hal.pos[i] as f64 / self.spu(i))
    }

    /// Send one frame; returns `false` on a Nack.
    fn send(&mut self, msg: HostMsg) -> bool {
        self.seq += 1;
        let seq = self.seq;
        self.hal.now = self.now;
        self.core.handle(HostFrame { seq, msg }, self.now, &mut self.hal);
        let mut ok = true;
        while let Some(f) = self.core.poll_outgoing() {
            match f.msg {
                McuMsg::Nack { seq: s, .. } if s == seq => ok = false,
                McuMsg::Fault { code, .. } => self.faults.push((self.now, code)),
                _ => {}
            }
        }
        ok
    }

    /// Advance the clock to `until`, feeding `pending` segments in time order
    /// whenever the MCU queue has room (as the engine does) and sending
    /// heartbeats, unless `host_alive` says the host has gone.
    fn run(&mut self, until: u64, pending: &mut Vec<Segment>, host_alive: &dyn Fn(u64) -> bool) {
        while self.now < until {
            if host_alive(self.now) {
                while let Some(seg) = pending.first().copied() {
                    if self.send(HostMsg::QueueSegment(seg)) {
                        pending.remove(0);
                    } else {
                        break; // queue full: try again next tick
                    }
                }
                if self.now - self.last_heartbeat >= HEARTBEAT_US {
                    self.send(HostMsg::Heartbeat);
                    self.last_heartbeat = self.now;
                }
            }
            self.now += TICK_US;
            self.hal.now = self.now;
            self.core.tick(self.now, &mut self.hal);
            while let Some(f) = self.core.poll_outgoing() {
                if let McuMsg::Fault { code, .. } = f.msg {
                    self.faults.push((self.now, code));
                }
            }
            self.trace.push((self.now, self.hal.pos));
        }
    }
}

// ------------------------------------------------------------------ plans

/// A planned move: segments plus the planned position of every axis (axis
/// units) at a time since the move start.
struct Plan {
    start_us: u64,
    duration: f64,
    segs: Vec<Segment>,
    position: Box<dyn Fn(usize, f64) -> f64>,
    /// Times (µs since the move start) at which the plan is exactly defined,
    /// for sampled trajectories (their knots); `None` = everywhere.
    defined_at: Option<Vec<u64>>,
}

fn sorted(mut segs: Vec<Segment>) -> Vec<Segment> {
    segs.sort_by_key(|s| (s.start_us, s.axis));
    segs
}

fn movej(rig: &Rig, target: [f64; AXES], scale: f64) -> Plan {
    let axes = axes(&rig.robot).to_vec();
    let start = rig.pose();
    let l: Vec<AxisLimits> = axes.iter().map(limits).collect();
    let profiles = plan_synchronized_scaled(&start, &target, &l, scale).expect("MoveJ plans");
    let start_us = rig.now + LEAD_US;
    let segs = profiles
        .iter()
        .zip(&axes)
        .flat_map(|(p, a)| if p.distance == 0.0 { vec![] } else { to_segments(p, a.channel, a.steps_per_unit, start_us, MAX_SEGMENT_US) })
        .collect();
    let duration = profiles.iter().map(|p| p.duration()).fold(0.0, f64::max);
    Plan { start_us, duration, segs: sorted(segs), position: Box::new(move |j, t| profiles[j].position(t)), defined_at: None }
}

fn movej_path(rig: &Rig, waypoints: &[[f64; AXES]], scale: f64) -> Plan {
    let axes = axes(&rig.robot).to_vec();
    let mut w: Vec<Vec<f64>> = vec![rig.pose().to_vec()];
    w.extend(waypoints.iter().map(|p| p.to_vec()));
    let l: Vec<AxisLimits> = axes.iter().map(limits).collect();
    let profiles = plan_blended(&w, &l, scale).expect("MoveJPath plans");
    let start_us = rig.now + LEAD_US;
    let segs = profiles.iter().zip(&axes).flat_map(|(p, a)| to_segments(p, a.channel, a.steps_per_unit, start_us, MAX_SEGMENT_US)).collect();
    let duration = profiles[0].duration();
    Plan { start_us, duration, segs: sorted(segs), position: Box::new(move |j, t| profiles[j].position(t)), defined_at: None }
}

/// The engine's serial chain of the profile geometry (engine.rs `joint_specs`).
fn chain(robot: &ResolvedRobot) -> Chain {
    let axes = axes(robot);
    let joints = axes
        .iter()
        .map(|a| {
            let g = a.geometry.as_ref().expect("geometry");
            JointSpec {
                id: a.id.clone(),
                parent: g.parent.as_ref().and_then(|p| axes.iter().position(|x| &x.id == p)),
                origin: g.origin,
                rotation_deg: g.rotation,
                axis: g.axis,
                tip: g.tip,
                radius: g.radius.unwrap_or(0.0),
                prismatic: a.kind == JointKind::Prismatic,
            }
        })
        .collect();
    Chain::new(joints, axes.iter().map(|a| (a.min, a.max)).collect()).expect("serial chain")
}

/// MoveL by `delta_mm`, orientation held: IK path, time-scaling and segments
/// exactly as the engine computes them.
fn movel(rig: &Rig, delta_mm: [f64; 3]) -> Plan {
    let axes = axes(&rig.robot).to_vec();
    let chain = chain(&rig.robot);
    let q0 = rig.pose().to_vec();
    let start = chain.forward(&q0);
    let position = [0, 1, 2].map(|k| start.position[k] + delta_mm[k]);
    let target = ToolPose { position, rotation: start.rotation };
    let path = chain.linear_path(&q0, &target, LINE_STEP_MM, LINE_STEP_DEG).expect("line is reachable");
    let dist = delta_mm.iter().map(|d| d * d).sum::<f64>().sqrt();
    let l: Vec<AxisLimits> = axes.iter().map(limits).collect();
    let traj = time_scale(&path, &l, 1.0, LINE_SPEED_MM_S / dist, LINE_ACCEL_MM_S2 / dist).expect("MoveL time-scales");
    let start_us = rig.now + LEAD_US;
    let segs = axes.iter().enumerate().flat_map(|(j, a)| traj.segments(j, a.channel, a.steps_per_unit, start_us)).collect();
    let duration = traj.duration();
    let knots = traj.t_us.clone();
    Plan { start_us, duration, segs: sorted(segs), position: Box::new(move |j, t| hermite(&traj, j, t)), defined_at: Some(knots) }
}

/// The planned position of a sampled trajectory: exact at the knots (cubic
/// Hermite through the knot positions and velocities in between, used only
/// for the planned velocity in reports).
fn hermite(s: &Sampled, j: usize, t: f64) -> f64 {
    let t_us = (t.max(0.0) * 1e6).min(*s.t_us.last().unwrap() as f64);
    let k = s.t_us.partition_point(|&x| (x as f64) <= t_us).saturating_sub(1).min(s.t_us.len() - 2);
    let (t0, t1) = (s.t_us[k] as f64 * 1e-6, s.t_us[k + 1] as f64 * 1e-6);
    let h = t1 - t0;
    let u = ((t_us * 1e-6 - t0) / h).clamp(0.0, 1.0);
    let (p0, p1, v0, v1) = (s.q[k][j], s.q[k + 1][j], s.v[k][j] * h, s.v[k + 1][j] * h);
    let (u2, u3) = (u * u, u * u * u);
    (2.0 * u3 - 3.0 * u2 + 1.0) * p0 + (u3 - 2.0 * u2 + u) * v0 + (-2.0 * u3 + 3.0 * u2) * p1 + (u3 - u2) * v1
}

// ------------------------------------------------------------------ checks

#[derive(Debug, Default, Clone, Copy)]
struct AxisReport {
    steps: u64,
    /// Largest |executed − planned| at a tick, steps.
    max_dev: f64,
    /// Largest excursion beyond "±1 step and one tick", steps (0 = within).
    worst_excess: f64,
    /// Time of the largest deviation (s since the move start) and the
    /// planned velocity there (steps/s).
    at_s: f64,
    v_at: f64,
}

/// Compare the executed positions with the plan up to `until` (MCU time):
/// at every tick, or for a sampled trajectory at the first tick at or after
/// each knot (where its plan is exactly defined).
fn compare(rig: &Rig, plan: &Plan, until: u64) -> [AxisReport; AXES] {
    let mut rep = [AxisReport::default(); AXES];
    let tick = TICK_US as f64 * 1e-6;
    let checked = |now: u64| match &plan.defined_at {
        None => true,
        Some(knots) => knots.iter().any(|&k| {
            let k = plan.start_us + k;
            now >= k && now < k + TICK_US
        }),
    };
    for &(now, pos) in rig.trace.iter().filter(|(t, _)| *t <= until && checked(*t)) {
        let t = (now as f64 - plan.start_us as f64) * 1e-6;
        for j in 0..AXES {
            let spu = rig.spu(j);
            let at = |t: f64| (plan.position)(j, t.clamp(0.0, plan.duration)) * spu;
            let (a, b) = (at(t - tick), at(t));
            let e = pos[j] as f64;
            if (e - b).abs() > rep[j].max_dev {
                rep[j].max_dev = (e - b).abs();
                rep[j].at_s = t;
                rep[j].v_at = (at(t + 1e-5) - at(t - 1e-5)) / 2e-5;
            }
            let excess = (a.min(b) - 1.0 - e).max(e - a.max(b) - 1.0).max(0.0);
            rep[j].worst_excess = rep[j].worst_excess.max(excess);
        }
    }
    for &(t, j, _) in &rig.hal.steps {
        if t <= until {
            rep[j].steps += 1;
        }
    }
    rep
}

fn print_report(name: &str, rep: &[AxisReport; AXES]) {
    eprintln!("{name}: tick {TICK_US} µs");
    eprintln!("  axis    steps   max |dev| (steps)   beyond ±1 step + 1 tick   at (s)   planned v there (steps/s)");
    for (j, r) in rep.iter().enumerate() {
        eprintln!("  j{}   {:>8}   {:>17.3}   {:>23.3}   {:>6.3}   {:>25.1}", j + 1, r.steps, r.max_dev, r.worst_excess, r.at_s, r.v_at);
    }
}

fn assert_within(name: &str, rep: &[AxisReport; AXES]) {
    print_report(name, rep);
    for (j, r) in rep.iter().enumerate() {
        assert!(r.worst_excess <= 1e-6, "{name}: j{} leaves ±1 step and one tick of the plan by {:.3} steps ({rep:?})", j + 1, r.worst_excess);
    }
}

/// Run a full move and check it against the plan; returns the rig.
fn execute(mut rig: Rig, plan: &Plan, name: &str) -> Rig {
    let mut pending = plan.segs.clone();
    let end = plan.start_us + (plan.duration * 1e6) as u64 + 200_000;
    rig.run(end, &mut pending, &|_| true);
    assert!(rig.faults.is_empty(), "{name}: faults {:?}", rig.faults);
    assert!(pending.is_empty(), "{name}: {} segments never accepted", pending.len());
    let rep = compare(&rig, plan, u64::MAX);
    assert_within(name, &rep);
    for j in 0..AXES {
        let want = ((plan.position)(j, plan.duration) * rig.spu(j)).round() as i64;
        assert_eq!(rig.hal.pos[j], want, "{name}: j{} ends at step {} not {want}", j + 1, rig.hal.pos[j]);
    }
    rig
}

// Poses of the six-axis arm (deg), inside every soft limit.
const START: [f64; AXES] = [0.0, 20.0, 120.0, 0.0, -40.0, 0.0];
const MOVEJ_TARGET: [f64; AXES] = [60.0, 50.0, 80.0, 90.0, 30.0, 180.0];

// ------------------------------------------------------------------ scenarios

#[test]
fn movej_six_axes_follows_the_plan() {
    for scale in [1.0, 0.3] {
        let rig = Rig::new(&START);
        let plan = movej(&rig, MOVEJ_TARGET, scale);
        let rig = execute(rig, &plan, &format!("MoveJ 6 axes, speed_scale {scale}"));
        if scale == 1.0 {
            check_step_skew_tool(&rig, &plan);
        }
    }
}

#[test]
fn movej_path_with_corners_and_reversals_follows_the_plan() {
    let rig = Rig::new(&START);
    // j1 and j4 reverse twice, j2/j3 turn corners, j6 reverses once.
    let waypoints = [
        [40.0, 40.0, 100.0, 60.0, -20.0, 90.0],
        [-30.0, 30.0, 140.0, -30.0, 10.0, 180.0],
        [20.0, 55.0, 90.0, 45.0, -30.0, 45.0],
        [0.0, 20.0, 120.0, 0.0, -40.0, 0.0],
    ];
    let plan = movej_path(&rig, &waypoints, 1.0);
    execute(rig, &plan, "MoveJPath, corners and reversals");
}

#[test]
fn movel_follows_the_plan() {
    let rig = Rig::new(&START);
    let plan = movel(&rig, [40.0, -30.0, -25.0]);
    execute(rig, &plan, "MoveL 56 mm, orientation held");
}

/// A line that passes the base: the reach shrinks and grows again, so the
/// shoulder and elbow reverse in the middle of the line.
#[test]
fn movel_with_joint_reversals_follows_the_plan() {
    let rig = Rig::new(&START);
    let chain = chain(&rig.robot);
    let p = chain.forward(&rig.pose()).position;
    // To the near side of the line first (joint move), then along +y.
    let mut rig = rig;
    let side = movel(&rig, [0.0, -60.0 - p[1], 0.0]);
    rig = execute(rig, &side, "MoveL to the start of the pass");
    let pass = movel(&rig, [0.0, 120.0, 0.0]);
    let reversing: Vec<usize> = (0..AXES)
        .filter(|&j| {
            let v: Vec<f64> = (0..=100).map(|k| (pass.position)(j, pass.duration * k as f64 / 100.0)).collect();
            v.windows(3).any(|w| (w[1] - w[0]) * (w[2] - w[1]) < 0.0)
        })
        .collect();
    assert!(!reversing.is_empty(), "the pass should reverse a joint");
    eprintln!("joints reversing during the pass: {:?}", reversing.iter().map(|j| format!("j{}", j + 1)).collect::<Vec<_>>());
    execute(rig, &pass, "MoveL past the base (joint reversals)");
}

#[test]
fn stop_mid_move_decelerates_within_the_stop_limit() {
    let mut rig = Rig::new(&START);
    let plan = movej(&rig, MOVEJ_TARGET, 1.0);
    let mut pending = plan.segs.clone();
    let stop_at = plan.start_us + (plan.duration * 0.5 * 1e6) as u64;
    rig.run(stop_at, &mut pending, &|_| true);
    let before = rig.hal.pos;
    let t = (stop_at - plan.start_us) as f64 * 1e-6;
    assert!(rig.send(HostMsg::Stop), "Stop accepted");
    pending.clear(); // the engine discards what it has not sent
    rig.run(stop_at + 3_000_000, &mut pending, &|_| true);
    assert!(rig.faults.is_empty(), "Stop must not fault: {:?}", rig.faults);
    assert_within("Stop: before the stop", &compare(&rig, &plan, stop_at));
    eprintln!("Stop mid-move at t = {t:.3} s:");
    eprintln!("  axis   v (steps/s)   stop decel   expected dist   actual dist   stopped after (ms)");
    for j in 0..AXES {
        let a = &axes(&rig.robot)[j];
        let spu = a.steps_per_unit;
        let v = ((plan.position)(j, t + 1e-4) - (plan.position)(j, t - 1e-4)) / 2e-4 * spu;
        let decel = a.stop_deceleration() * spu;
        let expect = v * v / (2.0 * decel);
        let after: Vec<&(u64, usize, i8)> = rig.hal.steps.iter().filter(|(tt, jj, _)| *tt > stop_at && *jj == j).collect();
        let dist = (rig.hal.pos[j] - before[j]).abs() as f64;
        let last = after.last().map_or(0.0, |s| (s.0 - stop_at) as f64 * 1e-3);
        eprintln!("  j{}   {:>11.1}   {:>10.0}   {:>13.2}   {:>11}   {:>18.1}", j + 1, v, decel, expect, dist, last);
        // Never reverses.
        assert!(after.iter().all(|s| s.2 as f64 * v >= 0.0), "j{}: reverses while stopping", j + 1);
        // Decelerates at the configured rate: not harder, not softer.
        assert!((dist - expect).abs() <= 2.0, "j{}: stopped in {dist} steps, expected {expect:.2} ± 2", j + 1);
        assert!(last <= (v.abs() / decel) * 1e3 + 2.0 * TICK_US as f64 * 1e-3, "j{}: still stepping {last} ms after Stop", j + 1);
        assert!(rig.hal.enabled[j], "Stop keeps drives enabled");
    }
    // Stopped for good: nothing more after another second.
    let settled = rig.hal.pos;
    rig.run(rig.now + 1_000_000, &mut pending, &|_| true);
    assert_eq!(settled, rig.hal.pos, "axes moved after the stop finished");
}

#[test]
fn estop_mid_move_emits_no_further_steps() {
    let mut rig = Rig::new(&START);
    let plan = movej(&rig, MOVEJ_TARGET, 1.0);
    let mut pending = plan.segs.clone();
    let trip = plan.start_us + (plan.duration * 0.4 * 1e6) as u64;
    rig.run(trip, &mut pending, &|_| true);
    rig.send(HostMsg::EmergencyStop);
    rig.run(trip + 1_000_000, &mut pending, &|_| true);
    assert_within("E-stop: before the trip", &compare(&rig, &plan, trip));
    let late: Vec<_> = rig.hal.steps.iter().filter(|s| s.0 > trip).collect();
    assert!(late.is_empty(), "{} steps after the E-stop, first {:?}", late.len(), late.first());
    assert!(rig.hal.enabled.iter().all(|e| !e), "drives disabled");
    assert_eq!(rig.faults.first().map(|f| f.1), Some(FaultCode::EmergencyStop));
}

#[test]
fn link_loss_mid_move_faults_and_stops_stepping() {
    let mut rig = Rig::new(&START);
    let plan = movej(&rig, MOVEJ_TARGET, 0.3);
    let mut pending = plan.segs.clone();
    let lost = plan.start_us + (plan.duration * 0.3 * 1e6) as u64;
    let watchdog_us = rig.robot.file.mcu["main"].watchdog_ms as u64 * 1000;
    rig.run(lost + watchdog_us + 1_000_000, &mut pending, &move |now| now < lost);
    let &(at, code) = rig.faults.first().expect("a fault latches when the host is gone");
    eprintln!("link loss at {} ms into the move: {code:?} after {} ms", (lost - plan.start_us) / 1000, (at - lost) / 1000);
    assert!(matches!(code, FaultCode::CommunicationLost | FaultCode::QueueUnderrun), "{code:?}");
    assert!(at <= lost + watchdog_us + 2 * TICK_US, "fault {} ms after the link loss, watchdog {} ms", (at - lost) / 1000, watchdog_us / 1000);
    assert_within("Link loss: until the fault", &compare(&rig, &plan, at - TICK_US));
    let late: Vec<_> = rig.hal.steps.iter().filter(|s| s.0 >= at).collect();
    assert!(late.is_empty(), "{} steps after the fault", late.len());
    assert!(rig.hal.enabled.iter().all(|e| !e), "drives disabled");
}

// ------------------------------------------------------------------ step-skew

/// Write the MoveJ STEP edges as a logic-analyser CSV and check the
/// `tools/step-skew` analysis against the ground truth: every axis ran from
/// the same clock, so the reported skew must be within one tick, and the
/// jitter within the tool's default limits (one window plus one step at the
/// local rate: in the ramps j2 deviates 271 µs at 2817 steps/s, 0.79 step).
/// Several steps due in one tick are spread evenly over that tick (a CSV row
/// cannot hold two edges of one channel at the same instant). Skipped when
/// Python is not available.
fn check_step_skew_tool(rig: &Rig, plan: &Plan) {
    // `PYTHON` names the interpreter when it is not on PATH as an executable
    // (e.g. a pyenv-win .bat shim).
    let candidates: Vec<String> = std::env::var("PYTHON").into_iter().chain(["python3".into(), "python".into()]).collect();
    let Some(python) = candidates.into_iter().find(|p| std::process::Command::new(p).arg("--version").output().is_ok_and(|o| o.status.success())) else {
        eprintln!("step-skew check skipped: no Python (set PYTHON to the interpreter)");
        return;
    };
    // MoveJ moves every axis in one direction, as the tool requires.
    let mut edges: Vec<(f64, usize)> = Vec::new();
    for j in 0..AXES {
        let mut by_tick: std::collections::BTreeMap<u64, u32> = Default::default();
        for s in rig.hal.steps.iter().filter(|s| s.1 == j && s.0 >= plan.start_us) {
            *by_tick.entry(s.0).or_default() += 1;
        }
        for (t, n) in by_tick {
            for k in 0..n {
                // Steps are emitted at the tick that found them due, i.e.
                // somewhere in the preceding tick: spread them over it.
                edges.push(((t - TICK_US) as f64 + TICK_US as f64 * (k + 1) as f64 / n as f64, j));
            }
        }
    }
    edges.sort_by(|a, b| a.0.total_cmp(&b.0));
    let mut csv = String::from("Time [s],D0,D1,D2,D3,D4,D5\n");
    let mut state = [0u8; AXES];
    let row = |t: f64, s: &[u8; AXES]| format!("{:.7},{}\n", t * 1e-6, s.map(|x| x.to_string()).join(","));
    for (t, j) in edges {
        state[j] = 1;
        csv += &row(t - 0.4, &state);
        state[j] = 0;
        csv += &row(t + 0.4, &state);
    }
    let dir = PathBuf::from(env!("CARGO_TARGET_TMPDIR"));
    let file = dir.join("movej6-steps.csv");
    std::fs::write(&file, csv).unwrap();
    let tool = Path::new(env!("CARGO_MANIFEST_DIR")).join("../../tools/step-skew/step_skew.py");
    let out = std::process::Command::new(&python)
        .arg(&tool)
        .arg(&file)
        .args(["--step", "D0,D1,D2,D3,D4,D5", "--max-skew-us", &TICK_US.to_string()])
        .output()
        .unwrap();
    let text = String::from_utf8_lossy(&out.stdout);
    eprintln!("tools/step-skew on the simulated MoveJ capture:\n{text}{}", String::from_utf8_lossy(&out.stderr));
    assert!(out.status.success(), "step-skew rejects simulated ground truth (skew above one tick, or jitter above its defaults):\n{text}");
}
