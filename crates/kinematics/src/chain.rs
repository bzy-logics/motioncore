//! Serial kinematic chain built from the robot profile geometry: forward
//! kinematics of the tool point, a numeric inverse-kinematics solver and
//! straight-line Cartesian paths (MoveL).
//!
//! Units: joint values in deg (revolute) or mm (prismatic); positions in mm;
//! orientations as roll/pitch/yaw in degrees with the URDF convention
//! `R = Rz(yaw) · Ry(pitch) · Rx(roll)`.
//!
//! The solver proposes joint values; the motion engine still validates every
//! one against limits and the collision model before anything moves.

// Index loops mirror the matrix notation in this module.
#![allow(clippy::needless_range_loop)]

use crate::collision::JointSpec;
use crate::math::{apply, mul, rotate, rpy, translate, M4, V3};
use thiserror::Error;

/// Tool pose: position (mm) and rotation matrix.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ToolPose {
    pub position: V3,
    pub rotation: [[f64; 3]; 3],
}

impl ToolPose {
    /// Roll, pitch, yaw in degrees (URDF convention).
    pub fn rpy_deg(&self) -> V3 {
        rpy_of(&self.rotation)
    }
}

#[derive(Debug, Clone, PartialEq, Error)]
pub enum ChainError {
    #[error("the robot geometry is not a single serial chain ({0})")]
    NotSerial(String),
    #[error("the target pose is out of reach (closest pose found is {mm:.1} mm and {deg:.1} deg away)")]
    Unreachable { mm: f64, deg: f64 },
    #[error("the straight line leaves the arm's reach at {at_percent:.0} % of the way ({mm:.1} mm and {deg:.1} deg short there)")]
    LeavesReach { at_percent: f64, mm: f64, deg: f64 },
    #[error("the straight path would leave the soft limits of {axis} at {at_percent:.0} % of the way")]
    Limits { axis: String, at_percent: f64 },
    #[error("the straight path passes too close to a singularity near {at_percent:.0} % of the way ({axis} would jump {jump:.1})")]
    Singular { axis: String, at_percent: f64, jump: f64 },
    #[error("non-finite value in {0}")]
    NonFinite(&'static str),
}

/// A serial chain: every joint has exactly one child except the last, whose
/// tip is the tool point.
#[derive(Debug, Clone)]
pub struct Chain {
    joints: Vec<JointSpec>,
    /// Soft limits per joint, chain order.
    limits: Vec<(f64, f64)>,
    tool: V3,
}

/// Orientation error weight: 1 rad of orientation error counts like this
/// many mm of position error in the solver.
const ROT_WEIGHT_MM: f64 = 100.0;

impl Chain {
    /// `joints` in parent-first order (as for the collision model); `limits`
    /// in the same order.
    pub fn new(joints: Vec<JointSpec>, limits: Vec<(f64, f64)>) -> Result<Self, ChainError> {
        if joints.is_empty() || joints.len() != limits.len() {
            return Err(ChainError::NotSerial("no joints".into()));
        }
        for (i, j) in joints.iter().enumerate() {
            let want = if i == 0 { None } else { Some(i - 1) };
            if j.parent != want {
                return Err(ChainError::NotSerial(format!("{} is not mounted on the joint before it", j.id)));
            }
        }
        let tool = joints.last().unwrap().tip.unwrap_or([0.0; 3]);
        Ok(Self { joints, limits, tool })
    }

    pub fn len(&self) -> usize {
        self.joints.len()
    }

    pub fn is_empty(&self) -> bool {
        self.joints.is_empty()
    }

    pub fn ids(&self) -> impl Iterator<Item = &str> {
        self.joints.iter().map(|j| j.id.as_str())
    }

    /// Whether the chain can set the tool orientation (six or more joints).
    pub fn controls_orientation(&self) -> bool {
        self.joints.len() >= 6
    }

    /// Frames of every joint (after its motion) and of the tool.
    fn frames(&self, q: &[f64]) -> (Vec<M4>, M4) {
        let mut out: Vec<M4> = Vec::with_capacity(self.joints.len());
        let mut base = translate([0.0; 3]);
        for (j, &v) in self.joints.iter().zip(q) {
            let motion = if j.prismatic { translate([j.axis[0] * v, j.axis[1] * v, j.axis[2] * v]) } else { rotate(j.axis, v.to_radians()) };
            base = mul(&base, &mul(&mul(&translate(j.origin), &rpy(j.rotation_deg)), &motion));
            out.push(base);
        }
        let tool = mul(&base, &translate(self.tool));
        (out, tool)
    }

    /// Tool pose for joint values `q`.
    pub fn forward(&self, q: &[f64]) -> ToolPose {
        let (_, t) = self.frames(q);
        pose_of(&t)
    }

    /// Joint values that put the tool at `target`, starting from `seed`
    /// (the solver follows the seed's branch). Position only for chains of
    /// fewer than six joints. Joint values are kept within the soft limits.
    pub fn inverse(&self, target: &ToolPose, seed: &[f64]) -> Result<Vec<f64>, ChainError> {
        let orient = self.controls_orientation();
        let rows = if orient { 6 } else { 3 };
        let n = self.joints.len();
        let mut q: Vec<f64> = seed.iter().zip(&self.limits).map(|(v, &(lo, hi))| v.clamp(lo, hi)).collect();
        let (mut e, mut err) = self.residual(target, &q, rows);
        // Levenberg–Marquardt: damped least squares whose damping shrinks
        // after a good step and grows after a bad one (robust near
        // singularities, where the plain Jacobian step blows up).
        let mut lambda2 = 1e-3;
        for _ in 0..500 {
            if err < 1e-4 {
                return Ok(q);
            }
            let jac = self.jacobian(&q);
            let mut a = vec![vec![0.0f64; rows]; rows];
            for (r, row) in a.iter_mut().enumerate() {
                for (c, cell) in row.iter_mut().enumerate() {
                    *cell = (0..n).map(|i| jac[i][r] * jac[i][c]).sum::<f64>() + if r == c { lambda2 } else { 0.0 };
                }
            }
            let Some(y) = solve(a, e[..rows].to_vec()) else { break };
            let mut dq: Vec<f64> = (0..n).map(|i| (0..rows).map(|r| jac[i][r] * y[r]).sum()).collect();
            // Limit the step so the linearisation holds.
            let biggest = dq.iter().fold(0.0f64, |m, v| m.max(v.abs()));
            if biggest > 10.0 {
                dq.iter_mut().for_each(|v| *v *= 10.0 / biggest);
            }
            let trial: Vec<f64> = q.iter().zip(&dq).zip(&self.limits).map(|((v, d), &(lo, hi))| (v + d).clamp(lo, hi)).collect();
            let (te, terr) = self.residual(target, &trial, rows);
            if terr < err {
                (q, e, err) = (trial, te, terr);
                lambda2 = (lambda2 / 3.0).max(1e-9);
            } else {
                lambda2 *= 4.0;
                if lambda2 > 1e6 {
                    break;
                }
            }
        }
        if err < 1e-3 {
            Ok(q)
        } else {
            let cur = self.forward(&q);
            let mm = (0..3).map(|k| (cur.position[k] - target.position[k]).powi(2)).sum::<f64>().sqrt();
            let deg = if orient { axis_angle(&mul3(&target.rotation, &transpose(&cur.rotation))).1.to_degrees() } else { 0.0 };
            Err(ChainError::Unreachable { mm, deg })
        }
    }

    /// Weighted pose error (mm, and rad × ROT_WEIGHT_MM) and its norm.
    fn residual(&self, target: &ToolPose, q: &[f64], rows: usize) -> ([f64; 6], f64) {
        let cur = self.forward(q);
        let mut e = [0.0f64; 6];
        for k in 0..3 {
            e[k] = target.position[k] - cur.position[k];
        }
        if rows == 6 {
            let w = rotation_error(&target.rotation, &cur.rotation);
            for k in 0..3 {
                e[3 + k] = w[k] * ROT_WEIGHT_MM;
            }
        }
        (e, e[..rows].iter().map(|x| x * x).sum::<f64>().sqrt())
    }

    /// Column i = d(tool position, weighted rotation)/d(q_i), per deg or mm.
    fn jacobian(&self, q: &[f64]) -> Vec<[f64; 6]> {
        let (frames, tool) = self.frames(q);
        let p = apply(&tool, [0.0; 3]);
        let d = std::f64::consts::PI / 180.0;
        self.joints
            .iter()
            .enumerate()
            .map(|(i, j)| {
                let f = &frames[i];
                let z = [
                    f[0][0] * j.axis[0] + f[0][1] * j.axis[1] + f[0][2] * j.axis[2],
                    f[1][0] * j.axis[0] + f[1][1] * j.axis[1] + f[1][2] * j.axis[2],
                    f[2][0] * j.axis[0] + f[2][1] * j.axis[1] + f[2][2] * j.axis[2],
                ];
                let mut col = [0.0f64; 6];
                if j.prismatic {
                    col[..3].copy_from_slice(&z);
                } else {
                    let o = apply(f, [0.0; 3]);
                    let c = cross(z, [p[0] - o[0], p[1] - o[1], p[2] - o[2]]);
                    for k in 0..3 {
                        col[k] = c[k] * d;
                        col[3 + k] = z[k] * d * ROT_WEIGHT_MM;
                    }
                }
                col
            })
            .collect()
    }

    /// Joint values along the straight line from the tool pose at `q0` to
    /// `target` (position interpolated linearly, rotation about a single
    /// axis), at uniform path parameter steps: the tool moves at most
    /// `step_mm` and turns at most `step_deg` between samples. The first
    /// sample is `q0`, the last reaches the target.
    pub fn linear_path(&self, q0: &[f64], target: &ToolPose, step_mm: f64, step_deg: f64) -> Result<Vec<Vec<f64>>, ChainError> {
        if target.position.iter().chain(target.rotation.iter().flatten()).any(|v| !v.is_finite()) {
            return Err(ChainError::NonFinite("target"));
        }
        let start = self.forward(q0);
        let dist = (0..3).map(|k| (target.position[k] - start.position[k]).powi(2)).sum::<f64>().sqrt();
        let (axis, angle) = if self.controls_orientation() { axis_angle(&mul3(&target.rotation, &transpose(&start.rotation))) } else { ([0.0, 0.0, 1.0], 0.0) };
        let n = ((dist / step_mm).max(angle.to_degrees() / step_deg).ceil() as usize).clamp(1, 20_000);
        let mut out = vec![q0.to_vec()];
        let mut q = q0.to_vec();
        for k in 1..=n {
            let s = k as f64 / n as f64;
            let p = [0, 1, 2].map(|i| start.position[i] + (target.position[i] - start.position[i]) * s);
            let r = mul3(&axis_rotation(axis, angle * s), &start.rotation);
            let pct = s * 100.0;
            let next = match self.inverse(&ToolPose { position: p, rotation: r }, &q) {
                Ok(next) => next,
                Err(ChainError::Unreachable { mm, deg }) => return Err(ChainError::LeavesReach { at_percent: pct, mm, deg }),
                Err(e) => return Err(e),
            };
            for (i, (&a, &b)) in q.iter().zip(&next).enumerate() {
                let (lo, hi) = self.limits[i];
                if b <= lo + 1e-6 || b >= hi - 1e-6 {
                    // The solver clamps at the limits: the path needs more.
                    if (self.forward(&next).position.iter().zip(p).map(|(x, y)| (x - y).powi(2)).sum::<f64>()).sqrt() > 1e-2 {
                        return Err(ChainError::Limits { axis: self.joints[i].id.clone(), at_percent: pct });
                    }
                }
                // A big joint jump for a small tool step means a singularity
                // or a branch flip.
                let budget = if self.joints[i].prismatic { 5.0 * step_mm } else { 10.0 * step_deg.max(0.5) + 2.0 };
                if (b - a).abs() > budget {
                    return Err(ChainError::Singular { axis: self.joints[i].id.clone(), at_percent: pct, jump: (b - a).abs() });
                }
            }
            q = next;
            out.push(q.clone());
        }
        Ok(out)
    }
}

/// Tool pose from roll/pitch/yaw degrees.
pub fn pose_from(position: V3, rpy_deg: V3) -> ToolPose {
    let m = rpy(rpy_deg);
    ToolPose { position, rotation: [[m[0][0], m[0][1], m[0][2]], [m[1][0], m[1][1], m[1][2]], [m[2][0], m[2][1], m[2][2]]] }
}

fn pose_of(m: &M4) -> ToolPose {
    ToolPose { position: [m[0][3], m[1][3], m[2][3]], rotation: [[m[0][0], m[0][1], m[0][2]], [m[1][0], m[1][1], m[1][2]], [m[2][0], m[2][1], m[2][2]]] }
}

/// Roll, pitch, yaw (deg) of a rotation matrix, R = Rz(y)·Ry(p)·Rx(r).
fn rpy_of(r: &[[f64; 3]; 3]) -> V3 {
    let pitch = (-r[2][0]).clamp(-1.0, 1.0).asin();
    let (roll, yaw) = if r[2][0].abs() < 1.0 - 1e-9 {
        (r[2][1].atan2(r[2][2]), r[1][0].atan2(r[0][0]))
    } else {
        // Gimbal lock: put everything in yaw.
        (0.0, (-r[0][1]).atan2(r[1][1]))
    };
    [roll.to_degrees(), pitch.to_degrees(), yaw.to_degrees()]
}

fn cross(a: V3, b: V3) -> V3 {
    [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]
}

fn mul3(a: &[[f64; 3]; 3], b: &[[f64; 3]; 3]) -> [[f64; 3]; 3] {
    let mut o = [[0.0; 3]; 3];
    for (r, row) in o.iter_mut().enumerate() {
        for (c, cell) in row.iter_mut().enumerate() {
            *cell = (0..3).map(|k| a[r][k] * b[k][c]).sum();
        }
    }
    o
}

fn transpose(a: &[[f64; 3]; 3]) -> [[f64; 3]; 3] {
    [[a[0][0], a[1][0], a[2][0]], [a[0][1], a[1][1], a[2][1]], [a[0][2], a[1][2], a[2][2]]]
}

/// Axis and angle (rad) of a rotation matrix.
fn axis_angle(r: &[[f64; 3]; 3]) -> (V3, f64) {
    let c = ((r[0][0] + r[1][1] + r[2][2] - 1.0) / 2.0).clamp(-1.0, 1.0);
    let angle = c.acos();
    if angle < 1e-9 {
        return ([0.0, 0.0, 1.0], 0.0);
    }
    let v = [r[2][1] - r[1][2], r[0][2] - r[2][0], r[1][0] - r[0][1]];
    let n = (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt();
    if n > 1e-9 {
        return ([v[0] / n, v[1] / n, v[2] / n], angle);
    }
    // 180°: axis from the diagonal.
    let d = [((r[0][0] + 1.0) / 2.0).max(0.0).sqrt(), ((r[1][1] + 1.0) / 2.0).max(0.0).sqrt(), ((r[2][2] + 1.0) / 2.0).max(0.0).sqrt()];
    let i = (0..3).max_by(|&a, &b| d[a].total_cmp(&d[b])).unwrap();
    let mut a = [0.0; 3];
    a[i] = d[i];
    for k in 0..3 {
        if k != i {
            a[k] = r[i][k] / (2.0 * d[i]);
        }
    }
    (a, angle)
}

fn axis_rotation(axis: V3, angle: f64) -> [[f64; 3]; 3] {
    let m = rotate(axis, angle);
    [[m[0][0], m[0][1], m[0][2]], [m[1][0], m[1][1], m[1][2]], [m[2][0], m[2][1], m[2][2]]]
}

/// Rotation vector (axis · angle, rad) taking `cur` to `target`.
fn rotation_error(target: &[[f64; 3]; 3], cur: &[[f64; 3]; 3]) -> V3 {
    let (axis, angle) = axis_angle(&mul3(target, &transpose(cur)));
    [axis[0] * angle, axis[1] * angle, axis[2] * angle]
}

/// Solve `a · x = b` (Gaussian elimination with partial pivoting).
fn solve(mut a: Vec<Vec<f64>>, mut b: Vec<f64>) -> Option<Vec<f64>> {
    let n = b.len();
    for c in 0..n {
        let p = (c..n).max_by(|&i, &j| a[i][c].abs().total_cmp(&a[j][c].abs()))?;
        if a[p][c].abs() < 1e-15 {
            return None;
        }
        a.swap(c, p);
        b.swap(c, p);
        for r in c + 1..n {
            let f = a[r][c] / a[c][c];
            for k in c..n {
                a[r][k] -= f * a[c][k];
            }
            b[r] -= f * b[c];
        }
    }
    let mut x = vec![0.0; n];
    for r in (0..n).rev() {
        x[r] = (b[r] - (r + 1..n).map(|k| a[r][k] * x[k]).sum::<f64>()) / a[r][r];
    }
    Some(x)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::collision::tests::dummy;

    const LIMITS: [(f64, f64); 6] = [(-170.0, 170.0), (-73.0, 90.0), (35.0, 180.0), (-180.0, 180.0), (-120.0, 120.0), (-720.0, 720.0)];

    fn arm() -> Chain {
        Chain::new(dummy().joints().to_vec(), LIMITS.to_vec()).unwrap()
    }

    #[test]
    fn forward_matches_the_collision_model_and_the_firmware() {
        let c = arm();
        // Same values as the collision model's FK test (from the robot's
        // own firmware kinematics): arm straight up and the rest pose.
        let up = c.forward(&[0.0; 6]).position;
        assert!((up[0] + 17.0).abs() < 1e-6 && up[1].abs() < 1e-6 && (up[2] - 442.0).abs() < 1e-6, "{up:?}");
        let rest = c.forward(&[0.0, -73.0, 180.0, 0.0, 0.0, 0.0]).position;
        assert!((rest[0] - 89.41).abs() < 0.01 && (rest[2] - 146.74).abs() < 0.01, "{rest:?}");
    }

    #[test]
    fn rpy_round_trips() {
        for r in [[10.0, 20.0, 30.0], [-170.0, 45.0, 90.0], [0.0, -80.0, -120.0]] {
            let p = pose_from([0.0; 3], r);
            let back = p.rpy_deg();
            for k in 0..3 {
                assert!((back[k] - r[k]).abs() < 1e-9, "{r:?} -> {back:?}");
            }
        }
    }

    #[test]
    fn inverse_reaches_poses_from_a_nearby_seed() {
        let c = arm();
        let mut s: u64 = 11;
        let mut rnd = |lo: f64, hi: f64| {
            s = s.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
            lo + (hi - lo) * ((s >> 11) as f64 / (1u64 << 53) as f64)
        };
        for _ in 0..200 {
            let goal: Vec<f64> =
                [(-120.0, 120.0), (-40.0, 60.0), (60.0, 150.0), (-120.0, 120.0), (-90.0, 90.0), (-180.0, 180.0)].iter().map(|&(l, h)| rnd(l, h)).collect();
            let target = c.forward(&goal);
            // Seed a few degrees away, as when following a path.
            let seed: Vec<f64> = goal.iter().map(|g| g + rnd(-4.0, 4.0)).collect();
            let q = c.inverse(&target, &seed).unwrap_or_else(|e| panic!("{goal:?}: {e}"));
            let got = c.forward(&q);
            for k in 0..3 {
                assert!((got.position[k] - target.position[k]).abs() < 1e-3);
            }
            assert!(rotation_error(&target.rotation, &got.rotation).iter().all(|v| v.abs() < 1e-5));
        }
    }

    #[test]
    fn unreachable_targets_are_refused() {
        let c = arm();
        let far = pose_from([2000.0, 0.0, 300.0], [0.0; 3]);
        let e = c.inverse(&far, &[0.0, 0.0, 90.0, 0.0, 0.0, 0.0]).unwrap_err();
        assert!(matches!(e, ChainError::Unreachable { mm, .. } if mm > 1000.0), "{e}");
        // A line toward it says where it leaves the workspace.
        let q0 = [0.0, 20.0, 110.0, 0.0, -40.0, 0.0];
        let e = c.linear_path(&q0, &far, 1.0, 0.5).unwrap_err();
        assert!(matches!(e, ChainError::LeavesReach { at_percent, .. } if at_percent < 50.0), "{e}");
        assert!(e.to_string().contains("leaves the arm's reach at"), "{e}");
    }

    #[test]
    fn linear_path_keeps_the_tool_on_the_line() {
        let c = arm();
        let q0 = [0.0, 20.0, 110.0, 0.0, -40.0, 0.0];
        let start = c.forward(&q0);
        let mut goal = start;
        goal.position[1] += 120.0;
        goal.position[2] -= 40.0;
        let path = c.linear_path(&q0, &goal, 1.0, 0.5).unwrap();
        assert!(path.len() > 120);
        for (k, q) in path.iter().enumerate() {
            let s = k as f64 / (path.len() - 1) as f64;
            let p = c.forward(q);
            for i in 0..3 {
                let want = start.position[i] + (goal.position[i] - start.position[i]) * s;
                assert!((p.position[i] - want).abs() < 1e-3, "sample {k}: {:?}", p.position);
            }
            // Orientation held.
            assert!(rotation_error(&start.rotation, &p.rotation).iter().all(|v| v.abs() < 1e-5));
        }
    }

    #[test]
    fn linear_path_refuses_a_line_through_the_soft_limits() {
        let c = arm();
        let q0 = [0.0, 20.0, 110.0, 0.0, -40.0, 0.0];
        let mut goal = c.forward(&q0);
        goal.position[2] -= 600.0; // through the floor and beyond reach
        assert!(c.linear_path(&q0, &goal, 1.0, 0.5).is_err());
    }

    #[test]
    fn chains_must_be_serial() {
        let mut j = dummy().joints().to_vec();
        j[3].parent = Some(1);
        assert!(matches!(Chain::new(j, LIMITS.to_vec()), Err(ChainError::NotSerial(_))));
    }
}
