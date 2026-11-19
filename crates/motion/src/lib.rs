//! Joint-space motion planning: trapezoidal profiles, synchronised multi-axis
//! moves, validation and conversion to MCU step segments.
//!
//! **Units.** This crate is unit-agnostic: every quantity of one axis is in
//! that axis' configured unit (e.g. `deg`, `deg/s`, `deg/s²` for a revolute
//! joint, `mm`… for a prismatic one). `steps_per_unit` converts to MCU steps.

use motioncore_protocol::Segment;
use thiserror::Error;

pub mod blend;
pub mod sampled;

/// A one-axis motion profile made of constant-acceleration pieces, as the
/// MCU executes them: `breakpoints` are the times where the acceleration
/// may change, so a segment between two of them is exactly quadratic.
pub trait Profile {
    fn duration(&self) -> f64;
    /// Position at `t` (clamped to the profile).
    fn position(&self, t: f64) -> f64;
    /// Signed velocity at `t`.
    fn velocity(&self, t: f64) -> f64;
    /// Signed acceleration in the piece that contains `t`.
    fn accel_at(&self, t: f64) -> f64;
    /// Piece boundaries, from 0 to the duration, ascending.
    fn breakpoints(&self) -> Vec<f64>;
}

impl Profile for Trapezoid {
    fn duration(&self) -> f64 {
        Trapezoid::duration(self)
    }
    fn position(&self, t: f64) -> f64 {
        Trapezoid::position(self, t)
    }
    fn velocity(&self, t: f64) -> f64 {
        Trapezoid::velocity(self, t)
    }
    fn accel_at(&self, t: f64) -> f64 {
        Trapezoid::accel_at(self, t)
    }
    fn breakpoints(&self) -> Vec<f64> {
        self.phase_times().to_vec()
    }
}

/// Kinematic limits of one axis, in axis units.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AxisLimits {
    pub min: f64,
    pub max: f64,
    pub max_velocity: f64,
    pub max_acceleration: f64,
}

#[derive(Debug, Clone, PartialEq, Error)]
pub enum MotionError {
    #[error("non-finite value in {0}")]
    NonFinite(&'static str),
    #[error("target {target} outside limits [{min}, {max}]")]
    OutOfLimits { target: f64, min: f64, max: f64 },
    #[error("velocity {requested} exceeds limit {limit}")]
    VelocityLimit { requested: f64, limit: f64 },
    #[error("{0} must be positive")]
    NonPositive(&'static str),
    #[error("profile violates {0}")]
    Infeasible(&'static str),
    #[error("axis count mismatch: {starts} starts, {targets} targets, {limits} limits")]
    InvalidAxisCount { starts: usize, targets: usize, limits: usize },
}

/// A symmetric trapezoidal (or triangular) velocity profile.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Trapezoid {
    pub start: f64,
    /// Signed distance.
    pub distance: f64,
    /// Peak velocity magnitude.
    pub v_peak: f64,
    /// Acceleration magnitude.
    pub accel: f64,
    pub t_accel: f64,
    pub t_cruise: f64,
}

impl Trapezoid {
    pub fn duration(&self) -> f64 {
        2.0 * self.t_accel + self.t_cruise
    }

    fn sign(&self) -> f64 {
        if self.distance < 0.0 {
            -1.0
        } else {
            1.0
        }
    }

    /// Position at time `t` (clamped to the profile).
    pub fn position(&self, t: f64) -> f64 {
        let t = t.clamp(0.0, self.duration());
        let (ta, tc, a, v) = (self.t_accel, self.t_cruise, self.accel, self.v_peak);
        let d = if t <= ta {
            0.5 * a * t * t
        } else if t <= ta + tc {
            0.5 * a * ta * ta + v * (t - ta)
        } else {
            let td = t - ta - tc;
            0.5 * a * ta * ta + v * tc + v * td - 0.5 * a * td * td
        };
        self.start + self.sign() * d
    }

    /// Signed velocity at time `t`.
    pub fn velocity(&self, t: f64) -> f64 {
        let (ta, tc) = (self.t_accel, self.t_cruise);
        let v = if t <= 0.0 || t >= self.duration() {
            0.0
        } else if t <= ta {
            self.accel * t
        } else if t <= ta + tc {
            self.v_peak
        } else {
            self.v_peak - self.accel * (t - ta - tc)
        };
        self.sign() * v
    }

    /// Signed acceleration in the phase that contains `t`.
    fn accel_at(&self, t: f64) -> f64 {
        let (ta, tc) = (self.t_accel, self.t_cruise);
        let a = if t < ta {
            self.accel
        } else if t < ta + tc {
            0.0
        } else {
            -self.accel
        };
        self.sign() * a
    }

    /// Phase boundary times (start, end of accel, end of cruise, end).
    fn phase_times(&self) -> [f64; 4] {
        let ta = self.t_accel;
        let tc = self.t_cruise;
        [0.0, ta, ta + tc, 2.0 * ta + tc]
    }
}

fn finite(name: &'static str, v: f64) -> Result<f64, MotionError> {
    if v.is_finite() {
        Ok(v)
    } else {
        Err(MotionError::NonFinite(name))
    }
}

/// Time-optimal trapezoid from `start` to `target`, respecting `v_max`/`a_max`.
pub fn plan_trapezoid(start: f64, target: f64, v_max: f64, a_max: f64) -> Result<Trapezoid, MotionError> {
    finite("start", start)?;
    finite("target", target)?;
    if finite("velocity", v_max)? <= 0.0 {
        return Err(MotionError::NonPositive("velocity"));
    }
    if finite("acceleration", a_max)? <= 0.0 {
        return Err(MotionError::NonPositive("acceleration"));
    }
    let distance = target - start;
    let d = distance.abs();
    let (v_peak, t_accel, t_cruise) = if d * a_max >= v_max * v_max {
        (v_max, v_max / a_max, (d - v_max * v_max / a_max) / v_max)
    } else {
        let v = (d * a_max).sqrt();
        (v, v / a_max, 0.0)
    };
    Ok(Trapezoid { start, distance, v_peak, accel: a_max, t_accel, t_cruise })
}

/// Validate and plan a single-axis point-to-point move.
///
/// `velocity` is an optional requested cruise speed (must be ≤ limit).
pub fn plan_axis_move(start: f64, target: f64, velocity: Option<f64>, limits: &AxisLimits) -> Result<Trapezoid, MotionError> {
    finite("target", target)?;
    if target < limits.min || target > limits.max {
        return Err(MotionError::OutOfLimits { target, min: limits.min, max: limits.max });
    }
    let v = match velocity {
        Some(v) => {
            finite("velocity", v)?;
            if v <= 0.0 {
                return Err(MotionError::NonPositive("velocity"));
            }
            if v > limits.max_velocity * (1.0 + 1e-9) {
                return Err(MotionError::VelocityLimit { requested: v, limit: limits.max_velocity });
            }
            v
        }
        None => limits.max_velocity,
    };
    let p = plan_trapezoid(start, target, v, limits.max_acceleration)?;
    validate(&p, limits)?;
    Ok(p)
}

/// Re-check a profile against limits (defence in depth for externally
/// proposed trajectories, e.g. from plugins).
pub fn validate(p: &Trapezoid, limits: &AxisLimits) -> Result<(), MotionError> {
    for (n, v) in [("start", p.start), ("distance", p.distance), ("v_peak", p.v_peak), ("accel", p.accel), ("t_accel", p.t_accel), ("t_cruise", p.t_cruise)] {
        finite(n, v)?;
    }
    if p.t_accel < 0.0 || p.t_cruise < 0.0 {
        return Err(MotionError::Infeasible("timing"));
    }
    let tol = 1e-9;
    if p.v_peak > limits.max_velocity * (1.0 + tol) + tol {
        return Err(MotionError::Infeasible("max_velocity"));
    }
    if p.accel > limits.max_acceleration * (1.0 + tol) + tol {
        return Err(MotionError::Infeasible("max_acceleration"));
    }
    // Profiles are monotonic, so an in-limit endpoint means the whole move
    // stays in limits or moves back into them. (The start may sit a fraction
    // of a step outside after step-quantised position feedback.)
    let end = p.position(p.duration());
    if end < limits.min - 1e-9 || end > limits.max + 1e-9 {
        return Err(MotionError::OutOfLimits { target: end, min: limits.min, max: limits.max });
    }
    if (end - (p.start + p.distance)).abs() > 1e-6 * (1.0 + p.distance.abs()) {
        return Err(MotionError::Infeasible("endpoint"));
    }
    Ok(())
}

/// Plan a synchronised joint-space move: all axes start and finish together,
/// sharing the phase timing of the slowest axis.
///
/// The three slices must have the same length, otherwise
/// [`MotionError::InvalidAxisCount`] is returned. A zero-axis move is valid
/// and plans to an empty trajectory.
pub fn plan_synchronized(starts: &[f64], targets: &[f64], limits: &[AxisLimits]) -> Result<Vec<Trapezoid>, MotionError> {
    plan_synchronized_scaled(starts, targets, limits, 1.0)
}

/// [`plan_synchronized`] with every axis' velocity limit scaled by
/// `speed_scale` (0 < scale ≤ 1). Acceleration limits are not scaled.
///
/// Because every axis shares the same phase times, each one follows
/// `start + distance · s(t)` with the same normalised `s(t)`: the path is a
/// straight line in joint space, and all axes start, cruise, decelerate and
/// stop together.
pub fn plan_synchronized_scaled(starts: &[f64], targets: &[f64], limits: &[AxisLimits], speed_scale: f64) -> Result<Vec<Trapezoid>, MotionError> {
    if starts.len() != targets.len() || targets.len() != limits.len() {
        return Err(MotionError::InvalidAxisCount { starts: starts.len(), targets: targets.len(), limits: limits.len() });
    }
    let scale = finite("speed_scale", speed_scale)?;
    if !(scale > 0.0 && scale <= 1.0) {
        return Err(MotionError::NonPositive("speed_scale (0 < scale <= 1)"));
    }
    if starts.is_empty() {
        return Ok(Vec::new());
    }
    let scaled: Vec<AxisLimits> = limits.iter().map(|l| AxisLimits { max_velocity: l.max_velocity * speed_scale, ..*l }).collect();
    let individual: Vec<Trapezoid> = starts.iter().zip(targets).zip(&scaled).map(|((&s, &t), l)| plan_axis_move(s, t, None, l)).collect::<Result<_, _>>()?;
    let lead = individual.iter().copied().max_by(|a, b| a.duration().total_cmp(&b.duration())).expect("at least one axis");
    let (mut ta, mut tc) = (lead.t_accel, lead.t_cruise);
    if ta + tc == 0.0 {
        // Nothing moves.
        return Ok(individual);
    }

    // Stretch the shared timing by the smallest factor k ≥ 1 that keeps every
    // axis within its limits. With phase times k·ta and k·tc an axis moving d
    // needs v = d / (k·(ta+tc)) and a = d / (k²·ta·(ta+tc)).
    let mut k: f64 = 1.0;
    for (p, l) in individual.iter().zip(&scaled) {
        let d = p.distance.abs();
        if d == 0.0 {
            continue;
        }
        k = k.max(d / (l.max_velocity * (ta + tc)));
        if ta > 0.0 {
            k = k.max((d / (l.max_acceleration * ta * (ta + tc))).sqrt());
        }
    }
    // Rounding margin, so v and a never exceed a limit by an ulp.
    k *= 1.0 + 1e-12;
    ta *= k;
    tc *= k;

    let out: Vec<Trapezoid> = individual
        .iter()
        .map(|p| {
            let d = p.distance.abs();
            let (v, a) = if d == 0.0 {
                (0.0, 0.0)
            } else if ta == 0.0 {
                (d / tc, 0.0)
            } else {
                let v = d / (ta + tc);
                (v, v / ta)
            };
            Trapezoid { start: p.start, distance: p.distance, v_peak: v, accel: a, t_accel: ta, t_cruise: tc }
        })
        .collect();
    for (p, l) in out.iter().zip(&scaled) {
        validate(p, l)?;
    }
    Ok(out)
}

/// Split a profile into MCU segments.
///
/// * Segments never straddle a phase boundary, so each one is exactly
///   quadratic and the MCU's `v0·t + ½·a·t²` model is exact.
/// * Segments never straddle a velocity reversal either: the MCU keeps a
///   segment's position within `[0, steps]`, so the backward part of a
///   segment that turns around would be dropped (the axis then lags the
///   plan by the depth of the turn). Blended corners (MoveJPath) reverse
///   inside a constant-acceleration piece.
/// * Step counts are differences of rounded **absolute** step positions, so
///   rounding error never accumulates across segments.
pub fn to_segments(p: &impl Profile, axis: u8, steps_per_unit: f64, start_us: u64, max_segment_us: u32) -> Vec<Segment> {
    let to_steps = |x: f64| (x * steps_per_unit).round() as i64;
    let mut times_us: Vec<u64> = Vec::new();
    let mut phases = p.breakpoints();
    let mut reversals = Vec::new();
    for w in phases.windows(2) {
        let (a, b) = (w[0], w[1]);
        let acc = p.accel_at(0.5 * (a + b));
        if b > a && acc != 0.0 {
            // Velocity is linear within a piece: v(t) = v(mid) + acc·(t − mid).
            let mid = 0.5 * (a + b);
            let tz = mid - p.velocity(mid) / acc;
            if tz > a + 1e-6 && tz < b - 1e-6 {
                reversals.push(tz);
            }
        }
    }
    phases.extend(reversals);
    phases.sort_by(f64::total_cmp);
    for w in phases.windows(2) {
        let (a, b) = ((w[0] * 1e6).round() as u64, (w[1] * 1e6).round() as u64);
        if b <= a {
            continue;
        }
        let n = (b - a).div_ceil(max_segment_us as u64);
        for k in 0..n {
            times_us.push(a + (b - a) * k / n);
        }
    }
    let end_us = (p.duration() * 1e6).round() as u64;
    times_us.push(end_us);
    times_us.dedup();

    let mut out = Vec::new();
    for (i, w) in times_us.windows(2).enumerate() {
        let (t0, t1) = (w[0] as f64 * 1e-6, w[1] as f64 * 1e-6);
        let mid = 0.5 * (t0 + t1);
        let s0 = to_steps(p.position(t0));
        let s1 = to_steps(p.position(t1));
        out.push(Segment {
            axis,
            start_us: start_us + w[0],
            duration_us: (w[1] - w[0]) as u32,
            steps: (s1 - s0) as i32,
            v0: (p.velocity(t0) * steps_per_unit) as f32,
            accel: (p.accel_at(mid) * steps_per_unit) as f32,
            last: i + 2 == times_us.len(),
        });
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    const L: AxisLimits = AxisLimits { min: -170.0, max: 170.0, max_velocity: 90.0, max_acceleration: 360.0 };

    #[test]
    fn trapezoid_reaches_cruise() {
        let p = plan_trapezoid(0.0, 90.0, 90.0, 360.0).unwrap();
        assert!((p.t_accel - 0.25).abs() < 1e-12);
        assert!((p.duration() - 1.25).abs() < 1e-12);
        assert!((p.position(p.duration()) - 90.0).abs() < 1e-9);
        assert!((p.velocity(0.5) - 90.0).abs() < 1e-9);
    }

    #[test]
    fn triangle_for_short_moves() {
        let p = plan_trapezoid(10.0, 9.0, 90.0, 360.0).unwrap();
        assert_eq!(p.t_cruise, 0.0);
        assert!(p.v_peak < 90.0);
        assert!((p.position(p.duration()) - 9.0).abs() < 1e-9);
        assert!(p.velocity(p.t_accel) < 0.0);
    }

    #[test]
    fn rejects_bad_requests() {
        assert!(matches!(plan_axis_move(0.0, 200.0, None, &L), Err(MotionError::OutOfLimits { .. })));
        assert!(matches!(plan_axis_move(0.0, f64::NAN, None, &L), Err(MotionError::NonFinite(_))));
        assert!(matches!(plan_axis_move(0.0, 10.0, Some(1000.0), &L), Err(MotionError::VelocityLimit { .. })));
        assert!(matches!(plan_axis_move(0.0, 10.0, Some(-1.0), &L), Err(MotionError::NonPositive(_))));
        assert!(plan_axis_move(0.0, 10.0, Some(f64::INFINITY), &L).is_err());
    }

    #[test]
    fn validate_catches_tampered_profiles() {
        let mut p = plan_axis_move(0.0, 90.0, None, &L).unwrap();
        p.accel *= 2.0;
        assert!(validate(&p, &L).is_err());
        let mut p = plan_axis_move(0.0, 90.0, None, &L).unwrap();
        p.distance = 500.0;
        assert!(validate(&p, &L).is_err());
    }

    #[test]
    fn zero_length_move_is_empty() {
        let p = plan_axis_move(5.0, 5.0, None, &L).unwrap();
        assert_eq!(p.duration(), 0.0);
        assert!(to_segments(&p, 0, 44.4, 0, 20_000).is_empty());
    }

    #[test]
    fn segments_are_contiguous_and_exact() {
        let spu = 44.444;
        for (s, t) in [(0.0, 90.0), (12.3, -150.7), (0.0, 0.5), (-3.0, 3.0)] {
            let p = plan_axis_move(s, t, None, &L).unwrap();
            let segs = to_segments(&p, 0, spu, 1_000_000, 20_000);
            assert!(segs.last().unwrap().last);
            assert!(segs[..segs.len() - 1].iter().all(|s| !s.last));
            let total: i64 = segs.iter().map(|s| s.steps as i64).sum();
            assert_eq!(total, (t * spu).round() as i64 - (s * spu).round() as i64);
            for w in segs.windows(2) {
                assert_eq!(w[0].end_us(), w[1].start_us, "segments must be contiguous");
            }
            for seg in &segs {
                assert!(seg.duration_us <= 20_000);
                // The MCU consistency rule.
                let tt = seg.duration_us as f32 * 1e-6;
                let ideal = seg.v0 * tt + 0.5 * seg.accel * tt * tt;
                assert!((ideal - seg.steps as f32).abs() <= 1.5, "{seg:?} ideal {ideal}");
                assert!(seg.v0.abs() as f64 <= L.max_velocity * spu * 1.0001);
            }
        }
    }

    #[test]
    fn synchronized_axes_finish_together() {
        let lim = [L, AxisLimits { max_velocity: 30.0, ..L }, L];
        let ps = plan_synchronized(&[0.0, 0.0, 0.0], &[90.0, 30.0, -10.0], &lim).unwrap();
        let d = ps[0].duration();
        for (p, l) in ps.iter().zip(&lim) {
            assert!((p.duration() - d).abs() < 1e-9);
            validate(p, l).unwrap();
        }
        assert!((ps[2].position(d) + 10.0).abs() < 1e-9);
    }
}

#[cfg(test)]
mod sync_tests {
    use super::*;

    const L: AxisLimits = AxisLimits { min: -720.0, max: 720.0, max_velocity: 60.0, max_acceleration: 100.0 };

    fn six() -> [AxisLimits; 6] {
        [L, AxisLimits { max_velocity: 30.0, ..L }, L, AxisLimits { max_acceleration: 40.0, ..L }, L, AxisLimits { max_velocity: 90.0, ..L }]
    }

    #[test]
    fn six_axes_share_one_timing_and_a_straight_joint_path() {
        let starts = [0.0, -73.0, 180.0, -180.0, 120.0, -720.0];
        let targets = [35.0, 0.0, 90.0, 30.0, -50.0, 90.0];
        let ps = plan_synchronized(&starts, &targets, &six()).unwrap();
        let d = ps[0].duration();
        for (p, l) in ps.iter().zip(six()) {
            assert_eq!(p.t_accel, ps[0].t_accel);
            assert_eq!(p.t_cruise, ps[0].t_cruise);
            validate(p, &l).unwrap();
        }
        // Straight line in joint space: every axis is at the same fraction.
        for k in 0..=20 {
            let t = d * k as f64 / 20.0;
            let f0 = (ps[0].position(t) - starts[0]) / (targets[0] - starts[0]);
            for i in 1..6 {
                let f = (ps[i].position(t) - starts[i]) / (targets[i] - starts[i]);
                assert!((f - f0).abs() < 1e-9, "axis {i} at {t}: {f} vs {f0}");
            }
        }
        // Some axis is exactly at a limit: the timing is not padded.
        let tight = ps.iter().zip(six()).any(|(p, l)| (p.v_peak / l.max_velocity - 1.0).abs() < 1e-6 || (p.accel / l.max_acceleration - 1.0).abs() < 1e-6);
        assert!(tight, "the stretch must be minimal");
    }

    #[test]
    fn segments_of_all_axes_share_the_time_grid() {
        let ps = plan_synchronized(&[0.0, 0.0, 0.0], &[90.0, -20.0, 5.0], &[L, L, L]).unwrap();
        let grids: Vec<Vec<(u64, u32)>> =
            ps.iter().enumerate().map(|(i, p)| to_segments(p, i as u8, 100.0, 1_000, 20_000).iter().map(|s| (s.start_us, s.duration_us)).collect()).collect();
        assert!(grids.iter().all(|g| g == &grids[0]));
    }

    #[test]
    fn speed_scale_slows_the_move_down() {
        let full = plan_synchronized(&[0.0, 0.0], &[300.0, 100.0], &[L, L]).unwrap();
        let half = plan_synchronized_scaled(&[0.0, 0.0], &[300.0, 100.0], &[L, L], 0.5).unwrap();
        assert!(half[0].duration() > full[0].duration() * 1.5);
        assert!(half[0].v_peak <= 30.0 + 1e-9);
        for bad in [0.0, -0.5, 1.5, f64::NAN] {
            assert!(plan_synchronized_scaled(&[0.0], &[1.0], &[L], bad).is_err(), "{bad}");
        }
    }

    #[test]
    fn axes_that_do_not_move_stay_put() {
        let ps = plan_synchronized(&[10.0, 5.0], &[50.0, 5.0], &[L, L]).unwrap();
        assert_eq!(ps[1].distance, 0.0);
        assert!((ps[1].position(ps[0].duration() / 2.0) - 5.0).abs() < 1e-12);
        let none = plan_synchronized(&[1.0, 2.0], &[1.0, 2.0], &[L, L]).unwrap();
        assert!(none.iter().all(|p| p.duration() == 0.0));
    }
}

#[cfg(test)]
mod sync_input_tests {
    use super::*;

    const L: AxisLimits = AxisLimits { min: -100.0, max: 100.0, max_velocity: 10.0, max_acceleration: 50.0 };

    #[test]
    fn mismatched_lengths_are_an_error_not_a_panic() {
        let err = plan_synchronized(&[0.0, 0.0], &[1.0], &[L, L]).unwrap_err();
        assert_eq!(err, MotionError::InvalidAxisCount { starts: 2, targets: 1, limits: 2 });
        assert!(plan_synchronized(&[0.0], &[1.0], &[]).is_err());
    }

    #[test]
    fn zero_axes_plan_to_empty_trajectory() {
        assert_eq!(plan_synchronized(&[], &[], &[]), Ok(Vec::new()));
    }
}

#[cfg(test)]
mod reversal_tests {
    use super::*;

    const L: AxisLimits = AxisLimits { min: -180.0, max: 180.0, max_velocity: 60.0, max_acceleration: 200.0 };

    /// Regression (#24, found by the simulator timing tests): a blended
    /// corner reverses a joint inside a constant-acceleration piece. The MCU
    /// keeps a segment's position within [0, steps], so a segment that turns
    /// around lost its backward part and the axis lagged the plan by the
    /// depth of the turn. Every segment must now move in one direction.
    #[test]
    fn segments_never_straddle_a_velocity_reversal() {
        let spu = 266.666667;
        // Corners of different shapes; in an asymmetric one the turn falls
        // between segment boundaries. The first is j3 of the simulator's
        // MoveJPath scenario.
        let paths: [&[f64]; 4] =
            [&[120.0, 100.0, 140.0, 90.0, 120.0], &[0.0, 40.0, -30.0, 20.0], &[0.0, 7.0, -50.0, 3.0, 90.0], &[10.0, -3.0, 60.0, 59.0, -20.0]];
        for path in paths {
            let w: Vec<Vec<f64>> = path.iter().map(|&x| vec![x]).collect();
            let ps = blend::plan_blended(&w, &[L], 1.0).unwrap();
            let segs = to_segments(&ps[0], 0, spu, 0, 20_000);
            for s in &segs {
                let v1 = s.end_velocity();
                // A segment ending exactly at the turn has an end velocity of
                // ±rounding; a real reversal has both ends clearly moving.
                assert!(!(s.v0 * v1 < 0.0 && s.v0.abs().min(v1.abs()) > 1.0), "{path:?}: segment reverses: {s:?}");
            }
            let total: i64 = segs.iter().map(|s| s.steps as i64).sum();
            assert_eq!(total, (path.last().unwrap() * spu).round() as i64 - (path[0] * spu).round() as i64);
        }
    }
}
