//! One trajectory type for every planner's output (ADR 0003, section 2).
//!
//! A [`Trajectory`] is a set of per-joint piecewise polynomials of degree at
//! most 3 on **shared breakpoints**, with metadata: what kind of motion it
//! is, the limits and speed scale it was planned with. Degree 3 is enough for
//! today's constant-acceleration profiles, for the cubic Hermite form of
//! time-scaled paths, and for jerk-limited (S-curve) profiles (#80).
//!
//! On piece `k` (from `breaks[k]` to `breaks[k + 1]`), joint `j` follows
//! `q(t) = c₀ + c₁·τ + c₂·τ² + c₃·τ³` with `τ = t − breaks[k]`.
//!
//! The type is hardware-independent: it knows nothing about steps, MCU
//! segments or boards. Converting it to segments is the executor's job
//! (stage 4, #93).
//!
//! Conversions from the existing planner outputs are lossless:
//!
//! - [`Trajectory::from_profiles`] for constant-acceleration profiles
//!   ([`crate::Trapezoid`], [`crate::blend::Piecewise`]): every piece is the
//!   original quadratic, split at the union of all joints' breakpoints.
//! - [`Trajectory::from_sampled`] for time-scaled paths
//!   ([`crate::sampled::Sampled`]): the cubic Hermite polynomial through the
//!   knot positions and velocities, exact at every knot.

use crate::sampled::Sampled;
use crate::{AxisLimits, MotionError, Profile};

/// What a trajectory moves.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[non_exhaustive]
pub enum MotionKind {
    /// One axis (jog, MoveTo).
    SingleAxis,
    /// Coordinated joint move (MoveJ).
    Joint,
    /// Blended joint path through waypoints (MoveJPath).
    JointPath,
    /// Straight line of the tool (MoveL).
    CartesianLine,
}

/// `c₀ + c₁·τ + c₂·τ² + c₃·τ³`, with τ measured from the piece's start.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Poly(pub [f64; 4]);

impl Poly {
    /// Position, velocity, acceleration and jerk at `tau`.
    pub fn eval(&self, tau: f64) -> JointState {
        let [c0, c1, c2, c3] = self.0;
        JointState {
            position: c0 + tau * (c1 + tau * (c2 + tau * c3)),
            velocity: c1 + tau * (2.0 * c2 + tau * 3.0 * c3),
            acceleration: 2.0 * c2 + 6.0 * c3 * tau,
            jerk: 6.0 * c3,
        }
    }

    /// A joint at rest at `position`.
    pub fn hold(position: f64) -> Self {
        Poly([position, 0.0, 0.0, 0.0])
    }
}

/// One joint's state at an instant, in its axis units.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct JointState {
    pub position: f64,
    pub velocity: f64,
    pub acceleration: f64,
    pub jerk: f64,
}

/// Per-joint piecewise polynomials on shared breakpoints, plus metadata.
#[derive(Debug, Clone, PartialEq)]
pub struct Trajectory {
    kind: MotionKind,
    /// Piece boundaries in seconds: `breaks[0] == 0`, ascending, the last is
    /// the duration. A trajectory of zero duration has `[0, 0]`.
    breaks: Vec<f64>,
    /// `polys[joint][piece]`.
    polys: Vec<Vec<Poly>>,
    /// Limits each joint was planned with.
    limits: Vec<AxisLimits>,
    /// Velocity scale the trajectory was planned with (0 < scale ≤ 1).
    speed_scale: f64,
}

/// Breakpoints closer than this (s) are one breakpoint.
const SAME_TIME: f64 = 1e-12;

fn bad(what: &'static str) -> MotionError {
    MotionError::Infeasible(what)
}

impl Trajectory {
    /// Build a trajectory from its pieces. `polys[j]` must have one
    /// polynomial per piece (`breaks.len() - 1`), and `limits` one entry per
    /// joint.
    pub fn new(kind: MotionKind, breaks: Vec<f64>, polys: Vec<Vec<Poly>>, limits: Vec<AxisLimits>, speed_scale: f64) -> Result<Self, MotionError> {
        if breaks.len() < 2 || breaks[0] != 0.0 {
            return Err(bad("trajectory breakpoints (must start at 0 with at least one piece)"));
        }
        if breaks.iter().any(|b| !b.is_finite()) {
            return Err(MotionError::NonFinite("trajectory breakpoints"));
        }
        let zero_length = breaks.len() == 2 && breaks[1] == 0.0;
        if !zero_length && breaks.windows(2).any(|w| w[1] <= w[0]) {
            return Err(bad("trajectory breakpoints (must increase)"));
        }
        if polys.len() != limits.len() || polys.iter().any(|p| p.len() != breaks.len() - 1) {
            return Err(MotionError::InvalidAxisCount { starts: polys.len(), targets: breaks.len() - 1, limits: limits.len() });
        }
        if polys.iter().flatten().flat_map(|p| p.0).any(|c| !c.is_finite()) {
            return Err(MotionError::NonFinite("trajectory coefficients"));
        }
        if !(speed_scale > 0.0 && speed_scale <= 1.0) {
            return Err(MotionError::NonPositive("speed_scale (0 < scale <= 1)"));
        }
        Ok(Self { kind, breaks, polys, limits, speed_scale })
    }

    /// Lossless conversion of constant-acceleration profiles (one per joint,
    /// all starting at t = 0): each is split at the union of every profile's
    /// breakpoints, so every piece is the profile's own quadratic. A profile
    /// shorter than the others holds its end position.
    pub fn from_profiles<P: Profile>(kind: MotionKind, profiles: &[P], limits: &[AxisLimits], speed_scale: f64) -> Result<Self, MotionError> {
        if profiles.len() != limits.len() {
            return Err(MotionError::InvalidAxisCount { starts: profiles.len(), targets: profiles.len(), limits: limits.len() });
        }
        let mut times: Vec<f64> = vec![0.0];
        for p in profiles {
            times.extend(p.breakpoints());
            times.push(p.duration());
        }
        times.sort_by(f64::total_cmp);
        let mut breaks: Vec<f64> = Vec::with_capacity(times.len());
        for t in times {
            match breaks.last() {
                Some(&last) if t - last <= SAME_TIME * (1.0 + t.abs()) => {}
                _ => breaks.push(t),
            }
        }
        if breaks.len() == 1 {
            breaks.push(0.0); // nothing moves: one piece of zero length
        }
        let polys = profiles
            .iter()
            .map(|p| {
                let end = p.duration();
                breaks
                    .windows(2)
                    .map(|w| {
                        let (a, b) = (w[0], w[1]);
                        if a >= end - SAME_TIME * (1.0 + end.abs()) {
                            return Poly::hold(p.position(end));
                        }
                        // Constant acceleration inside the piece; velocity is
                        // continuous at `a` (and zero at the start).
                        let acc = p.accel_at(0.5 * (a + b));
                        Poly([p.position(a), p.velocity(a), 0.5 * acc, 0.0])
                    })
                    .collect()
            })
            .collect();
        Self::new(kind, breaks, polys, limits.to_vec(), speed_scale)
    }

    /// Lossless conversion of a time-scaled path: on each knot interval, the
    /// cubic Hermite polynomial through the knot positions and velocities.
    pub fn from_sampled(kind: MotionKind, s: &Sampled, limits: &[AxisLimits], speed_scale: f64) -> Result<Self, MotionError> {
        let joints = limits.len();
        if s.t_us.len() < 2 || s.q.len() != s.t_us.len() || s.v.len() != s.t_us.len() || s.q.iter().chain(&s.v).any(|r| r.len() != joints) {
            return Err(MotionError::InvalidAxisCount { starts: s.q.first().map_or(0, |q| q.len()), targets: s.t_us.len(), limits: joints });
        }
        let breaks: Vec<f64> = s.t_us.iter().map(|&t| t as f64 * 1e-6).collect();
        let polys = (0..joints)
            .map(|j| {
                (0..breaks.len() - 1)
                    .map(|k| {
                        let h = breaks[k + 1] - breaks[k];
                        let (p0, p1, v0, v1) = (s.q[k][j], s.q[k + 1][j], s.v[k][j], s.v[k + 1][j]);
                        let slope = (p1 - p0) / h;
                        Poly([p0, v0, (3.0 * slope - 2.0 * v0 - v1) / h, (v0 + v1 - 2.0 * slope) / (h * h)])
                    })
                    .collect()
            })
            .collect();
        Self::new(kind, breaks, polys, limits.to_vec(), speed_scale)
    }

    pub fn kind(&self) -> MotionKind {
        self.kind
    }

    pub fn joint_count(&self) -> usize {
        self.polys.len()
    }

    /// Duration in seconds.
    pub fn duration(&self) -> f64 {
        *self.breaks.last().expect("at least one piece")
    }

    /// Piece boundaries, from 0 to the duration.
    pub fn breakpoints(&self) -> &[f64] {
        &self.breaks
    }

    /// Polynomials of joint `j`, one per piece.
    pub fn pieces(&self, j: usize) -> &[Poly] {
        &self.polys[j]
    }

    /// Limits each joint was planned with.
    pub fn limits(&self) -> &[AxisLimits] {
        &self.limits
    }

    pub fn speed_scale(&self) -> f64 {
        self.speed_scale
    }

    /// Index of the piece that contains `t` (clamped to the trajectory).
    fn piece_at(&self, t: f64) -> usize {
        let n = self.breaks.len() - 1;
        self.breaks.partition_point(|&b| b <= t).saturating_sub(1).min(n - 1)
    }

    /// State of joint `j` at time `t`. Times outside the trajectory are
    /// clamped to it; at a breakpoint the following piece is used.
    pub fn state(&self, j: usize, t: f64) -> JointState {
        let t = t.clamp(0.0, self.duration());
        let k = self.piece_at(t);
        self.polys[j][k].eval(t - self.breaks[k])
    }

    /// Every joint's position at `t`.
    pub fn positions(&self, t: f64) -> Vec<f64> {
        (0..self.joint_count()).map(|j| self.state(j, t).position).collect()
    }

    /// Joint vector at the start.
    pub fn start(&self) -> Vec<f64> {
        self.positions(0.0)
    }

    /// Joint vector at the end.
    pub fn end(&self) -> Vec<f64> {
        self.positions(self.duration())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::blend::plan_blended;
    use crate::sampled::time_scale;
    use crate::{plan_axis_move, plan_synchronized};

    const L: AxisLimits = AxisLimits { min: -720.0, max: 720.0, max_velocity: 60.0, max_acceleration: 150.0 };

    fn close(a: f64, b: f64) -> bool {
        (a - b).abs() <= 1e-9 * (1.0 + a.abs().max(b.abs()))
    }

    #[test]
    fn a_trapezoid_converts_exactly() {
        let p = plan_axis_move(10.0, 130.0, None, &L).unwrap();
        let t = Trajectory::from_profiles(MotionKind::SingleAxis, &[p], &[L], 1.0).unwrap();
        assert_eq!(t.breakpoints().len(), 4); // accelerate, cruise, decelerate
        assert!(close(t.duration(), p.duration()));
        for k in 0..=200 {
            let x = p.duration() * k as f64 / 200.0;
            let s = t.state(0, x);
            assert!(close(s.position, p.position(x)) && close(s.velocity, p.velocity(x)), "t = {x}: {s:?}");
            assert_eq!(s.jerk, 0.0);
        }
        assert_eq!(t.start(), vec![10.0]);
        assert!(close(t.end()[0], 130.0));
    }

    #[test]
    fn a_joint_that_finishes_early_holds_its_end() {
        // Two independent single-axis profiles of different lengths.
        let a = plan_axis_move(0.0, 100.0, None, &L).unwrap();
        let b = plan_axis_move(0.0, 5.0, None, &L).unwrap();
        let t = Trajectory::from_profiles(MotionKind::Joint, &[a, b], &[L, L], 1.0).unwrap();
        assert!(close(t.duration(), a.duration()));
        let late = t.state(1, a.duration() * 0.9);
        assert!(close(late.position, 5.0) && late.velocity == 0.0 && late.acceleration == 0.0);
    }

    #[test]
    fn nothing_moving_is_one_piece_of_zero_length() {
        let p = plan_synchronized(&[3.0, 4.0], &[3.0, 4.0], &[L, L]).unwrap();
        let t = Trajectory::from_profiles(MotionKind::Joint, &p, &[L, L], 1.0).unwrap();
        assert_eq!(t.duration(), 0.0);
        assert_eq!(t.start(), vec![3.0, 4.0]);
        assert_eq!(t.end(), vec![3.0, 4.0]);
    }

    #[test]
    fn a_blended_path_converts_exactly() {
        let wps = vec![vec![0.0, 0.0], vec![90.0, 30.0], vec![40.0, -60.0], vec![0.0, 0.0]];
        let profiles = plan_blended(&wps, &[L, L], 0.8).unwrap();
        let t = Trajectory::from_profiles(MotionKind::JointPath, &profiles, &[L, L], 0.8).unwrap();
        for (j, p) in profiles.iter().enumerate() {
            for k in 0..=500 {
                let x = p.duration() * k as f64 / 500.0;
                let s = t.state(j, x);
                assert!(close(s.position, p.position(x)) && close(s.velocity, p.velocity(x)) && close(s.acceleration, p.accel_at(x)), "joint {j}, t = {x}");
            }
        }
    }

    #[test]
    fn a_sampled_path_is_exact_at_every_knot() {
        let path: Vec<Vec<f64>> = (0..=300).map(|i| i as f64 / 300.0).map(|s| vec![100.0 * s, 40.0 * (std::f64::consts::PI * s).sin()]).collect();
        let s = time_scale(&path, &[L, L], 1.0, 100.0, 1000.0).unwrap();
        let t = Trajectory::from_sampled(MotionKind::CartesianLine, &s, &[L, L], 1.0).unwrap();
        assert_eq!(t.breakpoints().len(), s.t_us.len());
        for (k, &us) in s.t_us.iter().enumerate() {
            let x = us as f64 * 1e-6;
            for j in 0..2 {
                // From the piece ending here and from the one starting here.
                let after = t.state(j, x);
                assert!(close(after.position, s.q[k][j]) && close(after.velocity, s.v[k][j]), "knot {k}, joint {j}");
                if k > 0 {
                    let h = x - t.breakpoints()[k - 1];
                    let before = t.pieces(j)[k - 1].eval(h);
                    assert!(close(before.position, s.q[k][j]) && close(before.velocity, s.v[k][j]), "knot {k}, joint {j} (end of the piece before)");
                }
            }
        }
    }

    #[test]
    fn evaluation_clamps_to_the_trajectory() {
        let p = plan_axis_move(0.0, 50.0, None, &L).unwrap();
        let t = Trajectory::from_profiles(MotionKind::SingleAxis, &[p], &[L], 1.0).unwrap();
        assert_eq!(t.state(0, -1.0).position, 0.0);
        assert!(close(t.state(0, t.duration() + 5.0).position, 50.0));
    }

    #[test]
    fn malformed_trajectories_are_refused() {
        let one = vec![vec![Poly::hold(0.0)]];
        assert!(Trajectory::new(MotionKind::Joint, vec![0.0], vec![vec![]], vec![L], 1.0).is_err());
        assert!(Trajectory::new(MotionKind::Joint, vec![0.5, 1.0], one.clone(), vec![L], 1.0).is_err());
        assert!(Trajectory::new(MotionKind::Joint, vec![0.0, 1.0, 1.0], vec![vec![Poly::hold(0.0); 2]], vec![L], 1.0).is_err());
        assert!(Trajectory::new(MotionKind::Joint, vec![0.0, 1.0], one.clone(), vec![L, L], 1.0).is_err());
        assert!(Trajectory::new(MotionKind::Joint, vec![0.0, 1.0], vec![vec![Poly([f64::NAN, 0.0, 0.0, 0.0])]], vec![L], 1.0).is_err());
        assert!(Trajectory::new(MotionKind::Joint, vec![0.0, 1.0], one, vec![L], 0.0).is_err());
    }
}
