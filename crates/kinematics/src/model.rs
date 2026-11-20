//! The kinematic model interface ([`KinematicModel`]) and the one
//! forward-kinematics implementation ([`joint_frames`]).
//!
//! Every user of robot geometry — the IK solver, Cartesian paths, the
//! collision model, telemetry — takes its frames from a `KinematicModel`, so
//! they can never disagree about where a link is (ADR 0003, section 1).
//!
//! Joint frames follow the URDF convention of the robot profile geometry:
//!
//! `frame(child) = frame(parent) · T(origin) · RPY(rotation) · Joint(q about axis)`
//!
//! Units: joint values in deg (revolute) or mm (prismatic); lengths in mm.

use std::fmt::Debug;

use crate::chain::{ChainError, ToolPose};
use crate::math::{mul, rotate, rpy, translate, M4};

/// Joint description, in mm and degrees.
#[derive(Debug, Clone, PartialEq)]
pub struct JointSpec {
    pub id: String,
    /// Index of the parent joint (must come earlier in the list); None = base.
    pub parent: Option<usize>,
    pub origin: [f64; 3],
    pub rotation_deg: [f64; 3],
    /// Unit joint axis.
    pub axis: [f64; 3],
    pub tip: Option<[f64; 3]>,
    pub radius: f64,
    pub prismatic: bool,
}

/// World frame of every joint (after its own motion) for joint values `q`.
///
/// `joints` must be ordered parents first. A missing joint value counts as
/// zero. This is the only forward-kinematics implementation in MotionCore.
pub fn joint_frames(joints: &[JointSpec], q: &[f64]) -> Vec<M4> {
    let mut out: Vec<M4> = Vec::with_capacity(joints.len());
    for (i, j) in joints.iter().enumerate() {
        let base = j.parent.map(|p| out[p]).unwrap_or(translate([0.0; 3]));
        let v = q.get(i).copied().unwrap_or(0.0);
        let motion = if j.prismatic { translate([j.axis[0] * v, j.axis[1] * v, j.axis[2] * v]) } else { rotate(j.axis, v.to_radians()) };
        out.push(mul(&base, &mul(&mul(&translate(j.origin), &rpy(j.rotation_deg)), &motion)));
    }
    out
}

/// A robot's kinematics: geometry, limits, frames and, where the geometry
/// allows it, the tool pose, Jacobian and inverse kinematics.
///
/// Joint vectors are in the model's joint order ([`Self::joints`], parents
/// first). Solvers *propose* joint values; the motion engine still validates
/// every one against limits and the collision model before anything moves.
pub trait KinematicModel: Send + Sync + Debug {
    /// Joint geometry, parents first.
    fn joints(&self) -> &[JointSpec];

    /// Soft limits per joint, in joint order.
    fn limits(&self) -> &[(f64, f64)];

    /// World frame of every joint. Implementations use [`joint_frames`];
    /// an analytic model may override this only with an equivalent closed
    /// form that the shared FK tests also cover.
    fn frames(&self, q: &[f64]) -> Vec<M4> {
        joint_frames(self.joints(), q)
    }

    /// Tool pose, when the geometry has a single tool point.
    fn tool_pose(&self, _q: &[f64]) -> Option<ToolPose> {
        None
    }

    /// Jacobian of the tool pose: one column per joint, d(position mm,
    /// weighted rotation) per deg or mm. None without a tool point.
    fn jacobian(&self, _q: &[f64]) -> Option<Vec<[f64; 6]>> {
        None
    }

    /// Joint values that put the tool at `target`, near `seed` (the solver
    /// stays on the seed's branch).
    fn ik_near(&self, _target: &ToolPose, _seed: &[f64]) -> Result<Vec<f64>, ChainError> {
        Err(ChainError::NotSerial("this geometry has no inverse kinematics".into()))
    }

    /// Every solution for `target`, for models with a closed-form inverse
    /// (#81). None when the model only has a numerical solver.
    fn ik_all(&self, _target: &ToolPose) -> Option<Vec<Vec<f64>>> {
        None
    }
}

/// Any joint tree: frames only (no single tool point, no inverse).
#[derive(Debug, Clone)]
pub struct JointTree {
    joints: Vec<JointSpec>,
    limits: Vec<(f64, f64)>,
}

impl JointTree {
    /// `joints` parents first; `limits` in the same order.
    pub fn new(joints: Vec<JointSpec>, limits: Vec<(f64, f64)>) -> Self {
        Self { joints, limits }
    }

    /// A tree without soft limits (geometry only).
    pub fn unbounded(joints: Vec<JointSpec>) -> Self {
        let limits = vec![(f64::NEG_INFINITY, f64::INFINITY); joints.len()];
        Self { joints, limits }
    }
}

impl KinematicModel for JointTree {
    fn joints(&self) -> &[JointSpec] {
        &self.joints
    }

    fn limits(&self) -> &[(f64, f64)] {
        &self.limits
    }
}

#[cfg(test)]
pub(crate) mod tests {
    use super::*;
    use crate::chain::Chain;
    use crate::collision::CollisionModel;
    use crate::math::{apply, V3};
    use std::sync::Arc;

    /// The six-axis desktop arm of robot-profiles/dummy-arm-virtual.toml.
    pub(crate) fn dummy_joints() -> Vec<JointSpec> {
        let j = |id: &str, parent: Option<usize>, origin: V3, rot: V3, tip: Option<V3>, radius: f64| JointSpec {
            id: id.into(),
            parent,
            origin,
            rotation_deg: rot,
            axis: [0.0, 0.0, 1.0],
            tip,
            radius,
            prismatic: false,
        };
        vec![
            j("j1", None, [0.0; 3], [0.0; 3], None, 30.0),
            j("j2", Some(0), [35.0, 0.0, 109.0], [0.0, -90.0, -90.0], None, 22.0),
            j("j3", Some(1), [146.0, 0.0, 0.0], [0.0, 0.0, 90.0], None, 20.0),
            j("j4", Some(2), [-52.0, -115.0, 0.0], [90.0, 0.0, 0.0], None, 18.0),
            j("j5", Some(3), [0.0; 3], [-90.0, 0.0, 0.0], None, 18.0),
            j("j6", Some(4), [0.0; 3], [90.0, 0.0, 0.0], Some([0.0, 0.0, 72.0]), 15.0),
        ]
    }

    pub(crate) const DUMMY_LIMITS: [(f64, f64); 6] = [(-170.0, 170.0), (-73.0, 90.0), (35.0, 180.0), (-180.0, 180.0), (-120.0, 120.0), (-720.0, 720.0)];

    #[test]
    fn forward_kinematics_matches_the_dummy_firmware() {
        // Reference poses from the robot's own firmware kinematics: arm
        // straight up, and the rest pose.
        let joints = dummy_joints();
        let tip = |q: &[f64]| apply(&joint_frames(&joints, q)[5], [0.0, 0.0, 72.0]);
        let up = tip(&[0.0; 6]);
        assert!((up[0] + 17.0).abs() < 1e-9 && up[1].abs() < 1e-9 && (up[2] - 442.0).abs() < 1e-9, "{up:?}");
        let rest = tip(&[0.0, -73.0, 180.0, 0.0, 0.0, 0.0]);
        assert!((rest[0] - 89.41).abs() < 0.01 && (rest[2] - 146.74).abs() < 0.01, "{rest:?}");
    }

    #[test]
    fn every_user_of_the_geometry_gets_identical_frames() {
        let chain = Chain::new(dummy_joints(), DUMMY_LIMITS.to_vec()).unwrap();
        let tree = JointTree::new(dummy_joints(), DUMMY_LIMITS.to_vec());
        let from_tree = CollisionModel::new(dummy_joints(), 2.0);
        let from_chain = CollisionModel::from_model(Arc::new(chain.clone()), 2.0);
        for q in [[0.0; 6], [10.0, -30.0, 120.0, 45.0, -60.0, 200.0], [-150.0, 80.0, 40.0, -170.0, 110.0, -700.0]] {
            let want = joint_frames(&dummy_joints(), &q);
            // Bit-identical, not just close: one implementation.
            assert_eq!(chain.frames(&q), want);
            assert_eq!(tree.frames(&q), want);
            assert_eq!(from_tree.frames(&q), want);
            assert_eq!(from_chain.frames(&q), want);
            assert_eq!(from_tree.clearance(&q), from_chain.clearance(&q));
            // The tool point is the last joint's tip.
            assert_eq!(chain.tool_pose(&q).unwrap().position, apply(&want[5], [0.0, 0.0, 72.0]));
        }
    }

    #[test]
    fn a_tree_has_frames_but_no_tool_or_inverse() {
        let tree = JointTree::unbounded(dummy_joints());
        assert!(tree.tool_pose(&[0.0; 6]).is_none() && tree.jacobian(&[0.0; 6]).is_none());
        let target = Chain::new(dummy_joints(), DUMMY_LIMITS.to_vec()).unwrap().forward(&[0.0; 6]);
        assert!(tree.ik_near(&target, &[0.0; 6]).is_err());
        assert!(tree.ik_all(&target).is_none());
    }

    #[test]
    fn the_chain_solves_through_the_trait() {
        let chain: Arc<dyn KinematicModel> = Arc::new(Chain::new(dummy_joints(), DUMMY_LIMITS.to_vec()).unwrap());
        let goal = [20.0, 10.0, 100.0, 30.0, -40.0, 15.0];
        let target = chain.tool_pose(&goal).unwrap();
        let seed: Vec<f64> = goal.iter().map(|g| g + 2.0).collect();
        let q = chain.ik_near(&target, &seed).unwrap();
        let got = chain.tool_pose(&q).unwrap();
        for k in 0..3 {
            assert!((got.position[k] - target.position[k]).abs() < 1e-3);
        }
        assert_eq!(chain.jacobian(&q).unwrap().len(), 6);
        assert_eq!(chain.limits(), &DUMMY_LIMITS[..]);
    }
}
