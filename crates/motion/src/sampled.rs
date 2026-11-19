//! Time-scaling of a geometric joint path (e.g. the IK samples of a
//! Cartesian straight line) and its conversion to MCU segments.
//!
//! The path is given as joint vectors at uniform steps of a path parameter
//! s ∈ [0, 1]. A trapezoidal profile s(t) is chosen so that every joint stays
//! within its velocity and acceleration limits:
//! q̇ = q′(s)·ṡ and q̈ = q″(s)·ṡ² + q′(s)·s̈. The result is sampled into knots
//! every [`KNOT_US`]; each knot interval becomes one MCU segment whose step
//! count comes from rounded absolute positions (no drift) and whose start
//! velocity is the exact joint velocity at the knot.

use crate::{finite, plan_trapezoid, AxisLimits, MotionError, Trapezoid};
use motioncore_protocol::Segment;

/// Knot spacing (MCU segment length) of sampled trajectories.
pub const KNOT_US: u32 = 10_000;

/// A time-scaled joint path, sampled at knots.
#[derive(Debug, Clone, PartialEq)]
pub struct Sampled {
    /// Knot times, µs from the start; the first is 0, the last the duration.
    pub t_us: Vec<u64>,
    /// Joint values at each knot: `q[knot][joint]`.
    pub q: Vec<Vec<f64>>,
    /// Joint velocities at each knot (units/s).
    pub v: Vec<Vec<f64>>,
}

impl Sampled {
    pub fn duration(&self) -> f64 {
        *self.t_us.last().unwrap_or(&0) as f64 * 1e-6
    }

    /// MCU segments of joint `j`, one per knot interval; empty when the joint
    /// does not move by a step.
    pub fn segments(&self, j: usize, axis: u8, steps_per_unit: f64, start_us: u64) -> Vec<Segment> {
        let steps: Vec<i64> = self.q.iter().map(|q| (q[j] * steps_per_unit).round() as i64).collect();
        if steps.iter().all(|&s| s == steps[0]) {
            return Vec::new();
        }
        let n = self.t_us.len() - 1;
        (0..n)
            .map(|k| {
                let dur = (self.t_us[k + 1] - self.t_us[k]) as u32;
                let t = dur as f64 * 1e-6;
                let ds = steps[k + 1] - steps[k];
                let v0 = self.v[k][j] * steps_per_unit;
                let v1 = self.v[k + 1][j] * steps_per_unit;
                // Smooth acceleration between the knot velocities while the
                // MCU's consistency rule (|v0·t + ½·a·t² − steps| ≤ 1.5)
                // holds with margin; otherwise the acceleration that lands
                // exactly on the integer step count.
                let smooth = (v1 - v0) / t;
                let accel = if (v0 * t + 0.5 * smooth * t * t - ds as f64).abs() <= 1.0 { smooth } else { 2.0 * (ds as f64 - v0 * t) / (t * t) };
                Segment { axis, start_us: start_us + self.t_us[k], duration_us: dur, steps: ds as i32, v0: v0 as f32, accel: accel as f32, last: k + 1 == n }
            })
            .collect()
    }
}

/// Time-scale `path` (joint vectors at uniform s, first = current position)
/// within `limits` (velocity limits scaled by `speed_scale`) and at most
/// `s_speed` / `s_accel` in path units (1/s, 1/s²; e.g. Cartesian speed
/// divided by the path length).
pub fn time_scale(path: &[Vec<f64>], limits: &[AxisLimits], speed_scale: f64, s_speed: f64, s_accel: f64) -> Result<Sampled, MotionError> {
    let m = limits.len();
    if path.len() < 2 || path.iter().any(|q| q.len() != m) {
        return Err(MotionError::InvalidAxisCount { starts: path.first().map_or(0, |q| q.len()), targets: path.len(), limits: m });
    }
    let scale = finite("speed_scale", speed_scale)?;
    if !(scale > 0.0 && scale <= 1.0) {
        return Err(MotionError::NonPositive("speed_scale (0 < scale <= 1)"));
    }
    if !(finite("path speed", s_speed)? > 0.0 && finite("path acceleration", s_accel)? > 0.0) {
        return Err(MotionError::NonPositive("path speed"));
    }
    // The start (the current position) may sit a fraction of a step outside
    // the limits after step-quantised position feedback, e.g. a joint resting
    // at its soft limit; every other point must be inside. (Same rule as
    // `blend::plan_blended`.)
    for (k, q) in path.iter().enumerate() {
        for (&x, l) in q.iter().zip(limits) {
            finite("path", x)?;
            if k > 0 && (x < l.min - 1e-9 || x > l.max + 1e-9) {
                return Err(MotionError::OutOfLimits { target: x, min: l.min, max: l.max });
            }
        }
    }
    let n = path.len() - 1;
    let ds = 1.0 / n as f64;
    // Derivatives along the path (central differences inside).
    let d1: Vec<Vec<f64>> = (0..=n)
        .map(|i| {
            let (a, b) = (i.saturating_sub(1), (i + 1).min(n));
            (0..m).map(|j| (path[b][j] - path[a][j]) / ((b - a) as f64 * ds)).collect()
        })
        .collect();
    let d2: Vec<Vec<f64>> = (0..=n)
        .map(|i| {
            let c = i.clamp(1, n.saturating_sub(1).max(1));
            if n < 2 {
                return vec![0.0; m];
            }
            (0..m).map(|j| (path[c + 1][j] - 2.0 * path[c][j] + path[c - 1][j]) / (ds * ds)).collect()
        })
        .collect();
    let vmax: Vec<f64> = limits.iter().map(|l| l.max_velocity * scale).collect();
    let amax: Vec<f64> = limits.iter().map(|l| l.max_acceleration).collect();

    // Path speed limited by joint velocities.
    let mut v_s = s_speed;
    for row in &d1 {
        for j in 0..m {
            if row[j].abs() > 1e-12 {
                v_s = v_s.min(vmax[j] / row[j].abs());
            }
        }
    }
    let mut a_s = s_accel;
    for attempt in 0..60 {
        // Path acceleration limited by joint accelerations at this speed.
        let mut a_lim = a_s;
        let mut feasible = true;
        for (r1, r2) in d1.iter().zip(&d2) {
            for j in 0..m {
                let room = amax[j] - r2[j].abs() * v_s * v_s;
                if room <= 0.0 {
                    feasible = false;
                } else if r1[j].abs() > 1e-12 {
                    a_lim = a_lim.min(room / r1[j].abs());
                }
            }
        }
        if !feasible || a_lim <= 0.0 {
            v_s *= 0.8;
            continue;
        }
        let profile = plan_trapezoid(0.0, 1.0, v_s, a_lim)?;
        let out = sample(path, &d1, &profile);
        if within(&out, &vmax, &amax) {
            return Ok(out);
        }
        // The bounds above are estimates between samples: back off.
        v_s *= 0.9;
        a_s = a_lim * 0.85;
        if attempt == 59 {
            break;
        }
    }
    Err(MotionError::Infeasible("path timing"))
}

fn interp(rows: &[Vec<f64>], s: f64) -> Vec<f64> {
    let n = rows.len() - 1;
    let x = (s.clamp(0.0, 1.0) * n as f64).min(n as f64);
    let i = (x.floor() as usize).min(n.saturating_sub(1));
    let f = x - i as f64;
    rows[i].iter().zip(&rows[(i + 1).min(n)]).map(|(a, b)| a + (b - a) * f).collect()
}

fn sample(path: &[Vec<f64>], d1: &[Vec<f64>], s: &Trapezoid) -> Sampled {
    let total_us = (s.duration() * 1e6).ceil().max(1.0) as u64;
    // Knots every KNOT_US, plus the phase boundaries of s(t) (end of the
    // ramp up, start of the ramp down). An MCU segment has one constant
    // acceleration; a knot interval that contains a phase boundary cannot
    // be represented by one, and the step-count correction then overshoots
    // the velocity limit (found by the property tests, #23). Grid knots
    // closer than KNOT_US / 4 to a boundary or the end are dropped, so no
    // interval is very short.
    let phase: Vec<u64> = [s.t_accel, s.t_accel + s.t_cruise].iter().map(|t| (t * 1e6).round() as u64).filter(|&t| t > 0 && t < total_us).collect();
    let near = |t: u64, b: u64| t.abs_diff(b) < KNOT_US as u64 / 4;
    let mut t_us: Vec<u64> = (0..)
        .map(|k| k * KNOT_US as u64)
        .take_while(|&t| t < total_us)
        .filter(|&t| t == 0 || !(phase.iter().any(|&b| near(t, b)) || near(t, total_us)))
        .collect();
    t_us.extend(phase);
    t_us.push(total_us);
    t_us.sort_unstable();
    t_us.dedup();
    let mut q = Vec::with_capacity(t_us.len());
    let mut v = Vec::with_capacity(t_us.len());
    for (k, &t) in t_us.iter().enumerate() {
        let last = k + 1 == t_us.len();
        let tt = t as f64 * 1e-6;
        let (sp, sv) = if last { (1.0, 0.0) } else { (s.position(tt), s.velocity(tt)) };
        q.push(if last { path.last().unwrap().clone() } else { interp(path, sp) });
        v.push(interp(d1, sp).iter().map(|d| d * sv).collect());
    }
    Sampled { t_us, q, v }
}

fn within(s: &Sampled, vmax: &[f64], amax: &[f64]) -> bool {
    for k in 0..s.q.len() {
        for j in 0..vmax.len() {
            if s.v[k][j].abs() > vmax[j] * 1.02 {
                return false;
            }
            if k > 0 {
                let dt = (s.t_us[k] - s.t_us[k - 1]) as f64 * 1e-6;
                // Average velocity over the interval must also respect the
                // limit (the MCU runs at the interval's mean rate).
                let mean = (s.q[k][j] - s.q[k - 1][j]) / dt;
                if mean.abs() > vmax[j] * 1.02 || ((s.v[k][j] - s.v[k - 1][j]) / dt).abs() > amax[j] * 1.1 {
                    return false;
                }
            }
        }
    }
    true
}

#[cfg(test)]
mod tests {
    use super::*;

    const L: AxisLimits = AxisLimits { min: -720.0, max: 720.0, max_velocity: 60.0, max_acceleration: 150.0 };

    /// A curved joint path: joint 0 linear, joint 1 a half sine.
    fn curve(n: usize) -> Vec<Vec<f64>> {
        (0..=n)
            .map(|i| {
                let s = i as f64 / n as f64;
                vec![100.0 * s, 40.0 * (std::f64::consts::PI * s).sin()]
            })
            .collect()
    }

    #[test]
    fn respects_joint_limits_and_the_requested_path_speed() {
        let p = curve(400);
        let slow = time_scale(&p, &[L, L], 1.0, 0.1, 1.0).unwrap();
        // 1 / 0.1 s at cruise plus one 0.1 s acceleration ramp.
        assert!((slow.duration() - 10.1).abs() < 0.02, "{}", slow.duration());
        let fast = time_scale(&p, &[L, L], 1.0, 100.0, 1000.0).unwrap();
        // Joint 0 needs 100 deg at <= 60 deg/s.
        assert!(fast.duration() > 100.0 / 60.0);
        for v in &fast.v {
            assert!(v.iter().all(|v| v.abs() <= 60.0 * 1.02));
        }
        assert_eq!(fast.q.last().unwrap(), p.last().unwrap());
        assert!(fast.v.last().unwrap().iter().all(|v| *v == 0.0));
    }

    #[test]
    fn segments_are_contiguous_consistent_and_exact() {
        let p = curve(400);
        let s = time_scale(&p, &[L, L], 0.8, 100.0, 1000.0).unwrap();
        let spu = 444.444;
        for j in 0..2 {
            let segs = s.segments(j, j as u8, spu, 5_000);
            let total: i64 = segs.iter().map(|x| x.steps as i64).sum();
            assert_eq!(total, (p.last().unwrap()[j] * spu).round() as i64 - (p[0][j] * spu).round() as i64);
            for w in segs.windows(2) {
                assert_eq!(w[0].end_us(), w[1].start_us);
                // Velocity jumps between segments stay within step
                // quantisation (2 × 1.5 steps over a 10 ms segment).
                assert!((w[0].end_velocity() - w[1].v0).abs() <= 301.0, "{w:?}");
            }
            for x in &segs {
                let t = x.duration_us as f32 * 1e-6;
                assert!((x.v0 * t + 0.5 * x.accel * t * t - x.steps as f32).abs() <= 1.01);
                assert!(x.v0.abs() <= (60.0 * 0.8 * 1.05 * spu) as f32 && x.end_velocity().abs() <= (60.0 * 0.8 * 1.05 * spu) as f32);
            }
            assert!(segs.last().unwrap().last);
        }
    }

    /// Regression (#23): a joint resting at its soft limit reports a
    /// step-quantised position a fraction of a step outside (180° at
    /// 213.333333 steps/° is 38400 steps = 180.00000028°). A straight line
    /// that starts there was refused with "outside limits".
    #[test]
    fn accepts_a_start_a_fraction_of_a_step_outside_the_limits() {
        let l = AxisLimits { min: -180.0, max: 180.0, max_velocity: 60.0, max_acceleration: 200.0 };
        let spu: f64 = 213.333333;
        let start = (180.0 * spu).round() / spu;
        assert!(start > 180.0);
        // The IK clamps its seed into the limits, so later points are inside.
        let path = vec![vec![start, 0.0], vec![180.0, 10.0], vec![179.0, 20.0]];
        let s = time_scale(&path, &[l, L], 1.0, 1.0, 10.0).unwrap();
        assert_eq!(s.q[0], path[0]);
        // Points after the start must still be inside.
        let out = vec![vec![0.0, 0.0], vec![180.5, 10.0]];
        assert!(matches!(time_scale(&out, &[l, L], 1.0, 1.0, 10.0), Err(MotionError::OutOfLimits { .. })));
    }

    /// Regression (#23, found by the property tests): with a ramp shorter
    /// than two knots, the end of the ramp fell inside a 10 ms knot interval.
    /// One constant-acceleration segment cannot follow "accelerate, then
    /// cruise", and the step-count correction pushed the segment's end
    /// velocity above the MCU's rate limit (max_rate · 1.01 + 1), which the
    /// MCU refuses mid-move.
    #[test]
    fn knots_follow_the_phase_boundaries_of_short_ramps() {
        let l = AxisLimits { min: 0.0, max: 584.4817204181895, max_velocity: 9.911503097229474, max_acceleration: 793.4603022113835 };
        let spu = 1551.1399690023861;
        let scale = 0.9828339769870822;
        let n = 315;
        let path: Vec<Vec<f64>> = (0..=n).map(|i| vec![314.80589099515936 * (1.0 - i as f64 / n as f64)]).collect();
        let s = time_scale(&path, &[l], scale, 0.05, 71.59503936452782).unwrap();
        let limit = (l.max_velocity * spu * 1.05) as f32 * 1.01 + 1.0;
        for x in s.segments(0, 0, spu, 0) {
            assert!(x.v0.abs() <= limit && x.end_velocity().abs() <= limit, "{x:?} exceeds {limit} steps/s");
        }
    }

    #[test]
    fn rejects_bad_input() {
        assert!(time_scale(&[vec![0.0]], &[L], 1.0, 1.0, 1.0).is_err());
        assert!(time_scale(&curve(10), &[L, L], 0.0, 1.0, 1.0).is_err());
        assert!(time_scale(&[vec![0.0], vec![900.0]], &[L], 1.0, 1.0, 1.0).is_err());
        let still = time_scale(&[vec![3.0], vec![3.0]], &[L], 1.0, 1.0, 1.0).unwrap();
        assert!(still.segments(0, 0, 100.0, 0).is_empty());
    }
}
