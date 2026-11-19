//! Property-based tests of the planners (#23).
//!
//! Every planner is run on random limits, step resolutions, starts, targets,
//! waypoints and speed scales, and its output is checked against:
//!
//! * the kinematic limits: |v| ≤ max_velocity · speed_scale and
//!   |a| ≤ max_acceleration, with the tolerances documented per property;
//! * the soft limits, and the exact start and end positions;
//! * the rules the MCU applies to every segment it accepts
//!   (`firmware-stepper`, `Stepper::enqueue`): contiguous, step count
//!   consistent with v0·t + ½·a·t² within 1.5 steps (+1e-4 relative),
//!   |v0|, |v1| ≤ max_rate · 1.01 + 1 with the engine's
//!   max_rate = max_velocity · steps_per_unit · 1.05, every end position
//!   inside the MCU soft limits [floor(min·spu), ceil(max·spu)], and step
//!   totals that match the rounded absolute positions exactly.
//!
//! Starts are **step-quantised** the way the engine derives them from the MCU
//! step counter (`round(x · spu) / spu`), so a start at a soft limit can sit a
//! fraction of a step outside it. That is a normal state, and every planner
//! must accept it.
//!
//! Each property runs `CASES` cases (at least 1000, as #23 requires).

use motioncore_motion::blend::plan_blended;
use motioncore_motion::sampled::time_scale;
use motioncore_motion::{plan_axis_move, plan_synchronized_scaled, to_segments, AxisLimits, MotionError, Profile};
use motioncore_protocol::Segment;
use proptest::prelude::*;

const CASES: u32 = 1000;
/// MCU segment length used by the engine (`MAX_SEGMENT_US`).
const MAX_SEGMENT_US: u32 = 20_000;

// ------------------------------------------------------------------ inputs

/// One axis as a valid robot profile can describe it:
///
/// * at least 100 steps over the soft-limit range;
/// * a top step rate of 50 to 50 000 steps/s (profiles are refused above the
///   board's `max_step_rate`, at most 50 000);
/// * acceleration from 0.5× to 100× the velocity limit (per second), so
///   ramps range from 2 s to 10 ms;
/// * a full traverse at the scaled speed within a minute (keeps cases
///   fast).
#[derive(Debug, Clone, Copy)]
struct Axis {
    limits: AxisLimits,
    spu: f64,
}

const MAX_STEP_RATE: f64 = 50_000.0;

fn axis(scale: f64) -> impl Strategy<Value = Axis> {
    (-1000.0..0.0f64, 1.0..2000.0f64, 1.0..2000.0f64, 0.0..=1.0f64, 0.5..100.0f64).prop_map(move |(min, span, spu, u, k)| {
        let span = span.min(60.0 * MAX_STEP_RATE * scale / spu).max(100.0 / spu);
        let rate_min = (span * spu / (60.0 * scale)).max(50.0);
        let rate = rate_min + u * (MAX_STEP_RATE - rate_min);
        let max_velocity = rate / spu;
        Axis { limits: AxisLimits { min, max: min + span, max_velocity, max_acceleration: max_velocity * k }, spu }
    })
}

fn quantised(x: f64, spu: f64) -> f64 {
    (x * spu).round() / spu
}

/// A position the engine could report: step-quantised, anywhere in the
/// range, or exactly at a soft limit (and then possibly a fraction of a
/// step outside it).
fn start_in(a: Axis) -> impl Strategy<Value = f64> {
    let l = a.limits;
    prop_oneof![(l.min..=l.max).prop_map(move |x| quantised(x, a.spu)), Just(quantised(l.min, a.spu)), Just(quantised(l.max, a.spu)),]
}

/// A requested target: anywhere in the range, exactly at a limit, or a tiny
/// move (less than one step) away from `start`.
fn target_in(a: Axis, start: f64) -> impl Strategy<Value = f64> {
    let l = a.limits;
    let step = 1.0 / a.spu;
    prop_oneof![
        4 => l.min..=l.max,
        1 => Just(l.min),
        1 => Just(l.max),
        1 => (-step..step).prop_map(move |d| (start + d).clamp(l.min, l.max)),
    ]
}

// ------------------------------------------------------------------ checks

/// The MCU's acceptance rules for the segments of one axis, plus exact step
/// totals.
fn check_segments(segs: &[Segment], a: Axis, start: f64, end: f64, max_segment_us: Option<u32>) -> Result<(), TestCaseError> {
    let to_steps = |x: f64| (x * a.spu).round() as i64;
    let (s0, s1) = (to_steps(start), to_steps(end));
    if s0 == s1 && segs.iter().all(|s| s.steps == 0) {
        return Ok(());
    }
    prop_assert!(!segs.is_empty(), "no segments for a move of {} steps", s1 - s0);
    let max_rate = (a.limits.max_velocity * a.spu * 1.05) as f32;
    let rate_limit = max_rate * 1.01 + 1.0;
    let (lo, hi) = ((a.limits.min * a.spu).floor() as i64, (a.limits.max * a.spu).ceil() as i64);
    let mut pos = s0;
    for (i, s) in segs.iter().enumerate() {
        prop_assert!(s.duration_us > 0, "segment {i}: zero duration");
        if let Some(m) = max_segment_us {
            prop_assert!(s.duration_us <= m, "segment {i}: {} µs > {m}", s.duration_us);
        }
        if i > 0 {
            prop_assert_eq!(segs[i - 1].end_us(), s.start_us, "segment {} not contiguous", i);
        }
        prop_assert_eq!(s.last, i + 1 == segs.len(), "segment {}: wrong `last` flag", i);
        let t = s.duration_us as f32 * 1e-6;
        let ideal = s.v0 * t + 0.5 * s.accel * t * t;
        prop_assert!((ideal - s.steps as f32).abs() <= 1.5 + ideal.abs() * 1e-4, "segment {i}: {} steps but v0·t + ½·a·t² = {ideal} ({s:?})", s.steps);
        prop_assert!(s.v0.abs() <= rate_limit && s.end_velocity().abs() <= rate_limit, "segment {i}: rate above {rate_limit} steps/s ({s:?})");
        pos += s.steps as i64;
        // The start itself may be outside after quantisation; the MCU only
        // checks where each segment ends.
        prop_assert!(pos >= lo && pos <= hi, "segment {i} ends at step {pos}, outside MCU limits [{lo}, {hi}]");
    }
    prop_assert_eq!(pos, s1, "step total does not reach the target");
    Ok(())
}

/// Velocity and acceleration of a profile at its breakpoints and at
/// `samples` points in between, against `vmax` and `amax` with a relative
/// tolerance `tol`.
fn check_profile(p: &impl Profile, vmax: f64, amax: f64, tol: f64, samples: usize) -> Result<(), TestCaseError> {
    let d = p.duration();
    prop_assert!(d.is_finite() && d >= 0.0, "duration {d}");
    let mut times = p.breakpoints();
    times.extend((0..=samples).map(|k| d * k as f64 / samples as f64));
    for &t in &times {
        let v = p.velocity(t);
        let a = p.accel_at(t);
        prop_assert!(v.abs() <= vmax * (1.0 + tol) + 1e-12, "|v({t})| = {} > {vmax}", v.abs());
        prop_assert!(a.abs() <= amax * (1.0 + tol) + 1e-12, "|a({t})| = {} > {amax}", a.abs());
    }
    Ok(())
}

fn close(a: f64, b: f64, rel: f64) -> bool {
    (a - b).abs() <= rel * (1.0 + a.abs().max(b.abs()))
}

// ------------------------------------------------------------------ properties

proptest! {
    #![proptest_config(ProptestConfig::with_cases(CASES))]

    /// Single-axis moves (jog, move_to): within limits, exact endpoint,
    /// monotonic, and MCU-valid segments; with or without a requested speed.
    #[test]
    fn single_axis_moves(
        (a, start, target, speed) in axis(1.0).prop_flat_map(|a| start_in(a).prop_flat_map(move |s| (Just(a), Just(s), target_in(a, s), prop::option::of(0.01..=1.0f64))))
    ) {
        let l = a.limits;
        let p = plan_axis_move(start, target, speed.map(|f| f * l.max_velocity), &l)
            .map_err(|e| TestCaseError::fail(format!("refused {start} -> {target}: {e}")))?;
        check_profile(&p, speed.map_or(l.max_velocity, |f| f * l.max_velocity), l.max_acceleration, 1e-9, 64)?;
        prop_assert!(close(p.position(p.duration()), target, 1e-9), "ends at {} not {target}", p.position(p.duration()));
        prop_assert!(close(p.position(0.0), start, 1e-12));
        let (lo, hi) = (start.min(target), start.max(target));
        for k in 0..=32 {
            let x = p.position(p.duration() * k as f64 / 32.0);
            prop_assert!(x >= lo - 1e-9 * (1.0 + lo.abs()) && x <= hi + 1e-9 * (1.0 + hi.abs()), "{x} leaves [{lo}, {hi}]");
        }
        let segs = to_segments(&p, 0, a.spu, 1_000, MAX_SEGMENT_US);
        check_segments(&segs, a, start, target, Some(MAX_SEGMENT_US))?;
    }

    /// Targets outside the soft limits are refused, never planned.
    #[test]
    fn targets_outside_limits_are_refused(
        (a, start, beyond) in axis(1.0).prop_flat_map(|a| (Just(a), start_in(a), 1e-6..1000.0f64, any::<bool>()))
            .prop_map(|(a, s, d, up)| (a, s, if up { a.limits.max + d } else { a.limits.min - d }))
    ) {
        let r = plan_axis_move(start, beyond, None, &a.limits);
        prop_assert!(matches!(r, Err(MotionError::OutOfLimits { .. })), "{beyond} accepted: {r:?}");
    }

    /// MoveJ: every axis within its scaled limits, all start and finish
    /// together on a straight joint-space line, MCU-valid segments on one
    /// shared time grid.
    #[test]
    fn synchronised_moves(
        (scale, axes, starts, targets) in (0.01..=1.0f64).prop_flat_map(|scale| (Just(scale), prop::collection::vec(axis(scale), 1..=6)))
            .prop_flat_map(|(scale, axes)| {
                let starts: Vec<_> = axes.iter().map(|&a| start_in(a)).collect();
                (Just(scale), Just(axes), starts)
            })
            .prop_flat_map(|(scale, axes, starts)| {
                let targets: Vec<_> = axes.iter().zip(&starts).map(|(&a, &s)| target_in(a, s)).collect();
                (Just(scale), Just(axes), Just(starts), targets)
            })
    ) {
        let limits: Vec<AxisLimits> = axes.iter().map(|a| a.limits).collect();
        let profiles = plan_synchronized_scaled(&starts, &targets, &limits, scale)
            .map_err(|e| TestCaseError::fail(format!("refused: {e}")))?;
        let duration = profiles.iter().map(|p| p.duration()).fold(0.0, f64::max);
        let mut grid: Option<Vec<(u64, u32)>> = None;
        for (j, (p, a)) in profiles.iter().zip(&axes).enumerate() {
            check_profile(p, a.limits.max_velocity * scale, a.limits.max_acceleration, 1e-9, 64)?;
            prop_assert!(close(p.position(p.duration()), targets[j], 1e-9), "axis {j} ends at {} not {}", p.position(p.duration()), targets[j]);
            if p.distance != 0.0 {
                prop_assert!(close(p.duration(), duration, 1e-12), "axis {j} takes {} s, the move {duration} s", p.duration());
            }
            let segs = to_segments(p, j as u8, a.spu, 0, MAX_SEGMENT_US);
            check_segments(&segs, *a, starts[j], targets[j], Some(MAX_SEGMENT_US))?;
            if p.distance != 0.0 {
                let g: Vec<(u64, u32)> = segs.iter().map(|s| (s.start_us, s.duration_us)).collect();
                match &grid {
                    None => grid = Some(g),
                    Some(first) => prop_assert_eq!(first, &g, "axis {} segment times differ from the others", j),
                }
            }
        }
        // Straight line in joint space: at every instant each axis is at the
        // same fraction of its distance as the axis that moves farthest.
        // (Compared as positions: a fraction of a tiny distance amplifies
        // rounding errors.)
        let lead = profiles.iter().max_by(|a, b| a.distance.abs().total_cmp(&b.distance.abs())).unwrap();
        if lead.distance != 0.0 {
            for k in 0..=16 {
                let t = duration * k as f64 / 16.0;
                let f = (lead.position(t) - lead.start) / lead.distance;
                for (j, p) in profiles.iter().enumerate() {
                    let expect = p.start + f * p.distance;
                    prop_assert!(close(p.position(t), expect, 1e-9), "axis {j} off the straight line at t = {t}: {} vs {expect}", p.position(t));
                }
            }
        }
    }

    /// MoveJPath: continuous position and velocity, within scaled limits,
    /// exact first and last waypoint, never beyond the soft limits, MCU-valid
    /// segments on one shared time grid.
    #[test]
    fn blended_paths(
        (scale, axes, start, rest) in (0.01..=1.0f64).prop_flat_map(|scale| (Just(scale), prop::collection::vec(axis(scale), 1..=4)))
            .prop_flat_map(|(scale, axes)| {
                let start: Vec<_> = axes.iter().map(|&a| start_in(a)).collect();
                let point: Vec<_> = axes.iter().map(|a| a.limits.min..=a.limits.max).collect();
                (Just(scale), Just(axes.clone()), start, prop::collection::vec(point, 1..=12))
            })
    ) {
        let limits: Vec<AxisLimits> = axes.iter().map(|a| a.limits).collect();
        let mut waypoints = vec![start.clone()];
        waypoints.extend(rest);
        let profiles = plan_blended(&waypoints, &limits, scale).map_err(|e| TestCaseError::fail(format!("refused: {e}")))?;
        let end = waypoints.last().unwrap();
        let times = profiles[0].breakpoints();
        for (j, (p, a)) in profiles.iter().zip(&axes).enumerate() {
            prop_assert_eq!(&p.breakpoints(), &times, "axis {} has its own piece times", j);
            check_profile(p, a.limits.max_velocity * scale, a.limits.max_acceleration, 1e-6, 64)?;
            prop_assert!(close(p.position(0.0), start[j], 1e-9));
            prop_assert!(close(p.position(p.duration()), end[j], 1e-6), "axis {j} ends at {} not {}", p.position(p.duration()), end[j]);
            for w in p.pieces.windows(2) {
                let (x0, v0) = (w[0].p0 + w[0].v0 * w[0].duration + 0.5 * w[0].accel * w[0].duration.powi(2), w[0].v0 + w[0].accel * w[0].duration);
                prop_assert!(close(x0, w[1].p0, 1e-6), "axis {j}: position jumps {x0} -> {}", w[1].p0);
                prop_assert!((v0 - w[1].v0).abs() <= 1e-6 * (1.0 + a.limits.max_velocity), "axis {j}: velocity jumps {v0} -> {}", w[1].v0);
            }
            let (lo, hi) = p.range();
            let eps = 1e-9 * (1.0 + a.limits.max.abs().max(a.limits.min.abs()));
            prop_assert!(lo >= a.limits.min.min(start[j]) - eps && hi <= a.limits.max.max(start[j]) + eps, "axis {j} reaches [{lo}, {hi}]");
            let segs = to_segments(p, j as u8, a.spu, 0, MAX_SEGMENT_US);
            check_segments(&segs, *a, start[j], end[j], Some(MAX_SEGMENT_US))?;
        }
    }

    /// MoveL time-scaling: a smooth joint path is accepted from a quantised
    /// start, stays within the velocity limit (documented tolerance 2 %),
    /// starts and ends exactly on the path, and yields MCU-valid segments.
    ///
    /// Paths are smooth and finely sampled, like the engine's MoveL paths
    /// (IK every 0.5 mm / 0.25° of tool motion; lines through singularities
    /// are refused): here at most 1 unit and 0.5 % of the range of joint
    /// motion per sample, at most 20 000 samples.
    #[test]
    fn sampled_paths(
        (scale, axes, start, mid, end, s_speed, s_accel) in (0.05..=1.0f64).prop_flat_map(|scale| (Just(scale), prop::collection::vec(axis(scale), 1..=4)))
            .prop_flat_map(|(scale, axes)| {
                let start: Vec<_> = axes.iter().map(|&a| start_in(a)).collect();
                let mid: Vec<_> = axes.iter().map(|a| a.limits.min..=a.limits.max).collect();
                let end: Vec<_> = axes.iter().map(|a| a.limits.min..=a.limits.max).collect();
                (Just(scale), Just(axes), start, mid, end, 0.05..=10.0f64, 0.05..=100.0f64)
            })
    ) {
        let limits: Vec<AxisLimits> = axes.iter().map(|a| a.limits).collect();
        // A smooth curve, like the IK of a straight tool line away from
        // singularities: the quadratic Bézier q(s) = (1−s)²·start + 2s(1−s)·mid
        // + s²·end. It is a convex combination of in-limit points, so it stays
        // inside the limits, except near a quantised start a fraction of a
        // step outside, where it is clamped (by less than a step).
        // Samples: each moves a joint at most 1 unit and at most 0.5 % of its
        // range (|q′| ≤ 2·max(|mid − start|, |end − mid|)).
        let n = axes
            .iter()
            .enumerate()
            .map(|(j, a)| {
                let per_sample = (0.005 * (a.limits.max - a.limits.min)).min(1.0);
                (2.0 * (mid[j] - start[j]).abs().max((end[j] - mid[j]).abs()) / per_sample).ceil() as usize
            })
            .max()
            .unwrap()
            .clamp(2, 20_000);
        let path: Vec<Vec<f64>> = (0..=n)
            .map(|i| {
                let s = i as f64 / n as f64;
                (0..axes.len())
                    .map(|j| {
                        let l = axes[j].limits;
                        let x = (1.0 - s) * (1.0 - s) * start[j] + 2.0 * s * (1.0 - s) * mid[j] + s * s * end[j];
                        // Like the engine's IK, which clamps into the limits:
                        // only the start itself may be outside.
                        if i == 0 { x } else if i == n { end[j] } else { x.clamp(l.min, l.max) }
                    })
                    .collect()
            })
            .collect();
        let traj = time_scale(&path, &limits, scale, s_speed, s_accel)
            .map_err(|e| TestCaseError::fail(format!("refused: {e}")))?;
        prop_assert_eq!(&traj.q[0], &path[0]);
        prop_assert_eq!(traj.q.last().unwrap(), path.last().unwrap());
        prop_assert!(traj.v.last().unwrap().iter().all(|v| *v == 0.0), "does not end at rest");
        for (k, v) in traj.v.iter().enumerate() {
            for (j, a) in axes.iter().enumerate() {
                prop_assert!(v[j].abs() <= a.limits.max_velocity * scale * 1.02 + 1e-12, "knot {k}, axis {j}: |v| = {} > {}", v[j].abs(), a.limits.max_velocity * scale);
            }
        }
        for (j, a) in axes.iter().enumerate() {
            let segs = traj.segments(j, j as u8, a.spu, 0);
            check_segments(&segs, *a, path[0][j], path[n][j], None)?;
        }
    }
}
