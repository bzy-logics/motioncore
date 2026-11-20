//! Property tests: every planner's output converts to [`Trajectory`] without
//! loss (#91).
//!
//! * Constant-acceleration profiles (single-axis moves, MoveJ, MoveJPath):
//!   position and velocity match the original profile at every breakpoint
//!   and at evenly spread times, for every joint; acceleration matches inside
//!   every piece (it jumps at breakpoints).
//! * Time-scaled paths (MoveL): position and velocity match at every knot,
//!   from both sides, and positions stay continuous.
//!
//! Tolerance: 1e-9 relative (to 1 + the magnitude); the conversion only
//! re-expands each polynomial around a new origin.

use motioncore_motion::blend::plan_blended;
use motioncore_motion::sampled::time_scale;
use motioncore_motion::{plan_axis_move, plan_synchronized_scaled, AxisLimits, MotionKind, Profile, Trajectory};
use proptest::prelude::*;

const CASES: u32 = 1000;
const REL: f64 = 1e-9;

fn close(a: f64, b: f64) -> bool {
    (a - b).abs() <= REL * (1.0 + a.abs().max(b.abs()))
}

/// Limits of one axis: range [-500, 500], velocity 1–200 units/s,
/// acceleration 0.5–100 × the velocity limit per second.
fn limits() -> impl Strategy<Value = AxisLimits> {
    (1.0..200.0f64, 0.5..100.0f64).prop_map(|(v, k)| AxisLimits { min: -500.0, max: 500.0, max_velocity: v, max_acceleration: v * k })
}

/// The profile's own breakpoints plus `n` evenly spread times.
fn times(p: &impl Profile, n: usize) -> Vec<f64> {
    let d = p.duration();
    let mut t = p.breakpoints();
    t.extend((0..=n).map(|k| d * k as f64 / n as f64));
    t
}

/// Every joint of `t` against its source profile.
fn matches_profiles<P: Profile>(t: &Trajectory, profiles: &[P]) -> Result<(), TestCaseError> {
    let longest = profiles.iter().map(|p| p.duration()).fold(0.0, f64::max);
    prop_assert!(close(t.duration(), longest), "duration {} vs {longest}", t.duration());
    for (j, p) in profiles.iter().enumerate() {
        for b in p.breakpoints() {
            prop_assert!(t.breakpoints().iter().any(|&x| (x - b).abs() <= 1e-12 * (1.0 + b)), "joint {j}: breakpoint {b} missing");
        }
        for x in times(p, 64) {
            let s = t.state(j, x);
            prop_assert!(close(s.position, p.position(x)), "joint {j}, t = {x}: position {} vs {}", s.position, p.position(x));
            if x > 0.0 && x < p.duration() {
                prop_assert!(close(s.velocity, p.velocity(x)), "joint {j}, t = {x}: velocity {} vs {}", s.velocity, p.velocity(x));
                // Acceleration jumps at breakpoints, where the source profile
                // itself is ambiguous by an ulp: compare it inside pieces.
                let at_break = p.breakpoints().iter().any(|&b| (x - b).abs() <= 1e-9 * (1.0 + b));
                if !at_break {
                    prop_assert!(close(s.acceleration, p.accel_at(x)), "joint {j}, t = {x}: acceleration {} vs {}", s.acceleration, p.accel_at(x));
                }
            }
            prop_assert_eq!(s.jerk, 0.0);
        }
        // After its own end, a joint holds its end position at rest.
        let end = t.state(j, t.duration());
        prop_assert!(close(end.position, p.position(p.duration())));
    }
    Ok(())
}

proptest! {
    #![proptest_config(ProptestConfig::with_cases(CASES))]

    #[test]
    fn single_axis_moves(l in limits(), start in -500.0..500.0f64, target in -500.0..500.0f64, speed in proptest::option::of(0.01..1.0f64)) {
        let p = plan_axis_move(start, target, speed.map(|f| f * l.max_velocity), &l).unwrap();
        let t = Trajectory::from_profiles(MotionKind::SingleAxis, &[p], &[l], 1.0).unwrap();
        matches_profiles(&t, &[p])?;
    }

    #[test]
    fn synchronised_moves(
        axes in prop::collection::vec((limits(), -500.0..500.0f64, -500.0..500.0f64), 1..=6),
        scale in 0.01..=1.0f64,
    ) {
        let limits: Vec<AxisLimits> = axes.iter().map(|a| a.0).collect();
        let starts: Vec<f64> = axes.iter().map(|a| a.1).collect();
        let targets: Vec<f64> = axes.iter().map(|a| a.2).collect();
        let profiles = plan_synchronized_scaled(&starts, &targets, &limits, scale).unwrap();
        let t = Trajectory::from_profiles(MotionKind::Joint, &profiles, &limits, scale).unwrap();
        prop_assert_eq!(t.joint_count(), limits.len());
        matches_profiles(&t, &profiles)?;
    }

    #[test]
    fn blended_paths(
        joints in 1..=6usize,
        lims in prop::collection::vec(limits(), 6),
        points in prop::collection::vec(prop::collection::vec(-400.0..400.0f64, 6), 2..=8),
        scale in 0.05..=1.0f64,
    ) {
        let limits = &lims[..joints];
        let waypoints: Vec<Vec<f64>> = points.iter().map(|w| w[..joints].to_vec()).collect();
        let profiles = plan_blended(&waypoints, limits, scale).map_err(|e| TestCaseError::fail(format!("refused: {e}")))?;
        let t = Trajectory::from_profiles(MotionKind::JointPath, &profiles, limits, scale).unwrap();
        matches_profiles(&t, &profiles)?;
    }

    #[test]
    fn time_scaled_paths(
        joints in 1..=6usize,
        lims in prop::collection::vec(limits(), 6),
        ends in prop::collection::vec((-400.0..400.0f64, -400.0..400.0f64, -50.0..50.0f64), 6),
        samples in 2..=400usize,
        s_speed in 0.05..10.0f64,
        s_accel in 0.1..100.0f64,
        scale in 0.05..=1.0f64,
    ) {
        let limits = &lims[..joints];
        // A curved path: a straight line plus a sine bump per joint.
        let path: Vec<Vec<f64>> = (0..=samples)
            .map(|i| {
                let s = i as f64 / samples as f64;
                ends[..joints].iter().map(|&(a, b, c)| a + (b - a) * s + c * (std::f64::consts::PI * s).sin()).collect()
            })
            .collect();
        let sampled = time_scale(&path, limits, scale, s_speed, s_accel).map_err(|e| TestCaseError::fail(format!("refused: {e}")))?;
        let t = Trajectory::from_sampled(MotionKind::CartesianLine, &sampled, limits, scale).unwrap();
        prop_assert!(close(t.duration(), sampled.duration()));
        prop_assert_eq!(t.breakpoints().len(), sampled.t_us.len());
        for (k, &us) in sampled.t_us.iter().enumerate() {
            let x = us as f64 * 1e-6;
            for j in 0..joints {
                let after = t.state(j, x);
                prop_assert!(close(after.position, sampled.q[k][j]), "knot {k}, joint {j}: position {} vs {}", after.position, sampled.q[k][j]);
                prop_assert!(close(after.velocity, sampled.v[k][j]), "knot {k}, joint {j}: velocity {} vs {}", after.velocity, sampled.v[k][j]);
                if k > 0 {
                    // The piece that ends here lands on the same knot.
                    let before = t.pieces(j)[k - 1].eval(x - t.breakpoints()[k - 1]);
                    prop_assert!(close(before.position, sampled.q[k][j]), "knot {k}, joint {j}: piece before ends at {}", before.position);
                    prop_assert!(close(before.velocity, sampled.v[k][j]), "knot {k}, joint {j}: piece before ends at velocity {}", before.velocity);
                }
            }
        }
    }
}
