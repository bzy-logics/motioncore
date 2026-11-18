//! Blended multi-waypoint joint moves (MoveJPath).
//!
//! The waypoints are joined by straight joint-space segments travelled at
//! constant velocity; every corner is rounded by a constant-acceleration
//! blend shared by all joints, so the robot never stops at an intermediate
//! waypoint. The path passes *near* intermediate waypoints (the blend cuts
//! the corner) and starts and ends exactly at the first and last one, at
//! rest. Every joint follows the same piece times, and each piece has
//! constant acceleration, so MCU segments are exact.
//!
//! Timing, with τₖ the nominal time of waypoint k and dₖ = τₖ₊₁ − τₖ:
//! segment k moves joint j at vⱼₖ = Δⱼₖ / dₖ; the blend at waypoint k lasts
//! tₖ = maxⱼ |vⱼₖ − vⱼ,ₖ₋₁| / aⱼ (velocity 0 before the first and after the
//! last waypoint) and is centred on τₖ, except that the first blend ends at
//! t₀ and the last one starts at T − tₙ so the path starts and ends exactly
//! on the waypoints. Durations start at the time-optimal dₖ = maxⱼ |Δⱼₖ| / vⱼ
//! and are stretched until every blend fits: dₖ ≥ (tₖ + tₖ₊₁) / 2.

use crate::{finite, AxisLimits, MotionError, Profile};

/// One constant-acceleration piece of a [`Piecewise`] profile.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Piece {
    pub t0: f64,
    pub duration: f64,
    pub p0: f64,
    pub v0: f64,
    pub accel: f64,
}

impl Piece {
    fn end(&self) -> f64 {
        self.t0 + self.duration
    }
    fn at(&self, t: f64) -> (f64, f64) {
        let dt = (t - self.t0).clamp(0.0, self.duration);
        (self.p0 + self.v0 * dt + 0.5 * self.accel * dt * dt, self.v0 + self.accel * dt)
    }
}

/// A one-axis profile of consecutive constant-acceleration pieces.
#[derive(Debug, Clone, PartialEq)]
pub struct Piecewise {
    pub pieces: Vec<Piece>,
    pub end: f64,
}

impl Piecewise {
    fn piece(&self, t: f64) -> Option<&Piece> {
        self.pieces.iter().find(|p| t < p.end()).or(self.pieces.last())
    }

    /// Lowest and highest position reached (including turning points).
    pub fn range(&self) -> (f64, f64) {
        let mut lo = f64::INFINITY;
        let mut hi = f64::NEG_INFINITY;
        let mut see = |x: f64| {
            lo = lo.min(x);
            hi = hi.max(x);
        };
        for p in &self.pieces {
            see(p.p0);
            see(p.at(p.end()).0);
            if p.accel != 0.0 {
                let tz = -p.v0 / p.accel;
                if tz > 0.0 && tz < p.duration {
                    see(p.at(p.t0 + tz).0);
                }
            }
        }
        if self.pieces.is_empty() {
            see(self.end);
        }
        (lo, hi)
    }
}

impl Profile for Piecewise {
    fn duration(&self) -> f64 {
        self.pieces.last().map_or(0.0, |p| p.end())
    }
    fn position(&self, t: f64) -> f64 {
        self.piece(t).map_or(self.end, |p| p.at(t).0)
    }
    fn velocity(&self, t: f64) -> f64 {
        if t <= 0.0 || t >= self.duration() {
            return 0.0;
        }
        self.piece(t).map_or(0.0, |p| p.at(t).1)
    }
    fn accel_at(&self, t: f64) -> f64 {
        self.piece(t).map_or(0.0, |p| p.accel)
    }
    fn breakpoints(&self) -> Vec<f64> {
        let mut b: Vec<f64> = self.pieces.iter().map(|p| p.t0).collect();
        b.push(self.duration());
        b
    }
}

/// Plan a blended path through `waypoints` (each one joint vector, the first
/// being the current position) for joints with `limits`, every velocity
/// limit scaled by `speed_scale`. Returns one profile per joint, all with the
/// same piece times.
pub fn plan_blended(waypoints: &[Vec<f64>], limits: &[AxisLimits], speed_scale: f64) -> Result<Vec<Piecewise>, MotionError> {
    let m = limits.len();
    if waypoints.len() < 2 || waypoints.iter().any(|w| w.len() != m) {
        return Err(MotionError::InvalidAxisCount { starts: waypoints.first().map_or(0, |w| w.len()), targets: waypoints.len(), limits: m });
    }
    let scale = finite("speed_scale", speed_scale)?;
    if !(scale > 0.0 && scale <= 1.0) {
        return Err(MotionError::NonPositive("speed_scale (0 < scale <= 1)"));
    }
    // The start may sit a fraction of a step outside the limits after
    // step-quantised position feedback; every target must be inside.
    for (k, w) in waypoints.iter().enumerate() {
        for (&x, l) in w.iter().zip(limits) {
            finite("waypoint", x)?;
            if k > 0 && (x < l.min || x > l.max) {
                return Err(MotionError::OutOfLimits { target: x, min: l.min, max: l.max });
            }
        }
    }
    for l in limits {
        if finite("velocity", l.max_velocity)? <= 0.0 || finite("acceleration", l.max_acceleration)? <= 0.0 {
            return Err(MotionError::NonPositive("limits"));
        }
    }
    // Drop waypoints equal to the previous one.
    let mut pts: Vec<&Vec<f64>> = vec![&waypoints[0]];
    for w in &waypoints[1..] {
        if w.iter().zip(*pts.last().unwrap()).any(|(a, b)| a != b) {
            pts.push(w);
        }
    }
    let start = waypoints[0].clone();
    if pts.len() < 2 {
        return Ok(start.iter().map(|&p| Piecewise { pieces: Vec::new(), end: p }).collect());
    }
    let n = pts.len() - 1; // segments
    let vmax: Vec<f64> = limits.iter().map(|l| l.max_velocity * scale).collect();
    let amax: Vec<f64> = limits.iter().map(|l| l.max_acceleration).collect();
    let delta: Vec<Vec<f64>> = (0..n).map(|k| (0..m).map(|j| pts[k + 1][j] - pts[k][j]).collect()).collect();
    let mut d: Vec<f64> = delta.iter().map(|dk| (0..m).map(|j| dk[j].abs() / vmax[j]).fold(0.0, f64::max)).collect();

    let blends = |d: &[f64]| -> (Vec<Vec<f64>>, Vec<f64>) {
        let v: Vec<Vec<f64>> = (0..n).map(|k| (0..m).map(|j| delta[k][j] / d[k]).collect()).collect();
        let tb: Vec<f64> = (0..=n)
            .map(|k| {
                (0..m)
                    .map(|j| {
                        let before = if k == 0 { 0.0 } else { v[k - 1][j] };
                        let after = if k == n { 0.0 } else { v[k][j] };
                        (after - before).abs() / amax[j]
                    })
                    .fold(0.0, f64::max)
            })
            .collect();
        (v, tb)
    };
    let mut ok = false;
    for _ in 0..2000 {
        let (_, tb) = blends(&d);
        let mut stretched = false;
        for k in 0..n {
            let need = 0.5 * (tb[k] + tb[k + 1]);
            if d[k] < need * (1.0 - 1e-12) {
                // Blend times scale like 1/d: stretching to √(d·need) moves
                // both sides toward each other.
                d[k] = (d[k] * need).sqrt().max(d[k] * 1.0001);
                stretched = true;
            }
        }
        if !stretched {
            ok = true;
            break;
        }
    }
    if !ok {
        return Err(MotionError::Infeasible("blending"));
    }
    let (v, tb) = blends(&d);

    // Nominal waypoint times and the shared piece boundaries.
    let mut tau = vec![0.5 * tb[0]];
    for k in 0..n {
        tau.push(tau[k] + d[k]);
    }
    let total = tau[n] + 0.5 * tb[n];
    let mut out = Vec::with_capacity(m);
    for j in 0..m {
        let vel = |k: isize| if k < 0 || k as usize >= n { 0.0 } else { v[k as usize][j] };
        let mut pieces: Vec<Piece> = Vec::new();
        let mut t = 0.0;
        let mut p = start[j];
        let mut push = |t0: f64, dur: f64, v0: f64, a: f64, p: &mut f64, t: &mut f64| {
            if dur > 1e-12 {
                pieces.push(Piece { t0, duration: dur, p0: *p, v0, accel: a });
                *p += v0 * dur + 0.5 * a * dur * dur;
                *t = t0 + dur;
            }
        };
        for k in 0..=n {
            let (b0, b1) = if k == 0 {
                (0.0, tb[0])
            } else if k == n {
                (total - tb[n], total)
            } else {
                (tau[k] - 0.5 * tb[k], tau[k] + 0.5 * tb[k])
            };
            let (vin, vout) = (vel(k as isize - 1), vel(k as isize));
            // Cruise from the previous blend to this one.
            push(t, b0 - t, vin, 0.0, &mut p, &mut t);
            if b1 - b0 > 1e-12 {
                push(b0, b1 - b0, vin, (vout - vin) / (b1 - b0), &mut p, &mut t);
            } else {
                t = b1;
            }
        }
        let end = *pts.last().unwrap();
        if (p - end[j]).abs() > 1e-6 * (1.0 + end[j].abs()) {
            return Err(MotionError::Infeasible("blended endpoint"));
        }
        out.push(Piecewise { pieces, end: end[j] });
    }
    Ok(out)
}

#[cfg(test)]
mod tests {
    use super::*;

    const L: AxisLimits = AxisLimits { min: -720.0, max: 720.0, max_velocity: 60.0, max_acceleration: 150.0 };

    fn check(ps: &[Piecewise], wps: &[Vec<f64>], limits: &[AxisLimits]) {
        let t_end = ps[0].duration();
        for (j, p) in ps.iter().enumerate() {
            assert_eq!(p.breakpoints(), ps[0].breakpoints(), "shared piece times");
            assert!((p.position(0.0) - wps[0][j]).abs() < 1e-9);
            assert!((p.position(t_end) - wps.last().unwrap()[j]).abs() < 1e-6);
            assert_eq!(p.velocity(0.0), 0.0);
            // Continuous position and velocity across pieces.
            for w in p.pieces.windows(2) {
                let (pe, ve) = w[0].at(w[0].end());
                assert!((pe - w[1].p0).abs() < 1e-9 && (ve - w[1].v0).abs() < 1e-9);
            }
            let (_, v_end) = p.pieces.last().unwrap().at(t_end);
            assert!(v_end.abs() < 1e-9, "ends at rest");
            for piece in &p.pieces {
                assert!(piece.accel.abs() <= limits[j].max_acceleration * (1.0 + 1e-9));
                assert!(piece.v0.abs() <= limits[j].max_velocity * (1.0 + 1e-9));
            }
        }
    }

    #[test]
    fn two_waypoints_is_a_synchronised_trapezoid() {
        let w = vec![vec![0.0, 0.0], vec![90.0, -30.0]];
        let ps = plan_blended(&w, &[L, L], 1.0).unwrap();
        check(&ps, &w, &[L, L]);
        let tr = crate::plan_synchronized(&w[0], &w[1], &[L, L]).unwrap();
        assert!((ps[0].duration() - tr[0].duration()).abs() < 1e-6);
    }

    #[test]
    fn does_not_stop_at_intermediate_waypoints() {
        let w = vec![vec![0.0, 0.0, 0.0], vec![60.0, 30.0, -20.0], vec![120.0, 0.0, 20.0], vec![60.0, -40.0, 0.0]];
        let lim = [L, AxisLimits { max_velocity: 30.0, ..L }, L];
        let ps = plan_blended(&w, &lim, 1.0).unwrap();
        check(&ps, &w, &lim);
        // Joint 0 keeps moving through the first waypoint (60): its speed
        // there is well above zero.
        let t = ps[0].pieces.iter().find(|p| p.p0 <= 60.0 && p.at(p.end()).0 >= 60.0).unwrap();
        let tz = t.t0 + (0.0..t.duration).step_by_f(1e-4).find(|&dt| t.at(t.t0 + dt).0 >= 60.0).unwrap();
        assert!(ps[0].velocity(tz) > 20.0, "{}", ps[0].velocity(tz));
        // Faster than three separate stop-and-go moves.
        let stops: f64 = w.windows(2).map(|p| crate::plan_synchronized(&p[0], &p[1], &lim).unwrap()[0].duration()).sum();
        assert!(ps[0].duration() < 0.85 * stops, "{} vs {stops}", ps[0].duration());
    }

    #[test]
    fn corners_are_cut_and_stay_within_the_neighbouring_waypoints() {
        // A reversal: 0 → 50 → 0. The blend turns around before 50.
        let w = vec![vec![0.0], vec![50.0], vec![0.0]];
        let ps = plan_blended(&w, &[L], 1.0).unwrap();
        check(&ps, &w, &[L]);
        let (lo, hi) = ps[0].range();
        assert!(lo >= -1e-9 && hi <= 50.0 + 1e-9 && hi > 20.0, "{lo} {hi}");
    }

    #[test]
    fn speed_scale_and_input_errors() {
        let w = vec![vec![0.0], vec![300.0], vec![100.0]];
        let full = plan_blended(&w, &[L], 1.0).unwrap()[0].duration();
        let half = plan_blended(&w, &[L], 0.5).unwrap()[0].duration();
        assert!(half > full * 1.4);
        assert!(plan_blended(&w, &[L], 0.0).is_err());
        assert!(plan_blended(&[vec![0.0]], &[L], 1.0).is_err());
        assert!(matches!(plan_blended(&[vec![0.0], vec![800.0]], &[L], 1.0), Err(MotionError::OutOfLimits { .. })));
        // Repeated waypoints are skipped; all-equal plans to nothing.
        assert_eq!(plan_blended(&[vec![5.0], vec![5.0], vec![5.0]], &[L], 1.0).unwrap()[0].duration(), 0.0);
        let rep = plan_blended(&[vec![0.0], vec![10.0], vec![10.0], vec![20.0]], &[L], 1.0).unwrap();
        check(&rep, &[vec![0.0], vec![20.0]], &[L]);
    }

    #[test]
    fn segments_are_exact() {
        let w = vec![vec![0.0, 0.0], vec![40.0, 25.0], vec![-30.0, 60.0], vec![10.0, 0.0]];
        let ps = plan_blended(&w, &[L, L], 1.0).unwrap();
        let spu = 266.67;
        for (j, p) in ps.iter().enumerate() {
            let segs = crate::to_segments(p, j as u8, spu, 1_000, 20_000);
            let total: i64 = segs.iter().map(|s| s.steps as i64).sum();
            assert_eq!(total, (w.last().unwrap()[j] * spu).round() as i64);
            for s in &segs {
                let t = s.duration_us as f32 * 1e-6;
                assert!((s.v0 * t + 0.5 * s.accel * t * t - s.steps as f32).abs() <= 1.5, "{s:?}");
            }
            assert!(segs.last().unwrap().last);
        }
    }

    trait StepBy {
        fn step_by_f(self, step: f64) -> Box<dyn Iterator<Item = f64>>;
    }
    impl StepBy for std::ops::Range<f64> {
        fn step_by_f(self, step: f64) -> Box<dyn Iterator<Item = f64>> {
            let (a, b) = (self.start, self.end);
            Box::new((0..).map(move |k| a + k as f64 * step).take_while(move |x| *x < b))
        }
    }
}
