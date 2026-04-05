//! Forward/inverse kinematics abstractions.
//!
//! A [`Kinematics`] implementation maps between joint space (one value per
//! joint, in each joint's configured unit) and a task-space pose. v0.2 ships
//! [`JointIdentity`] for joint-space robots; articulated-arm solvers (e.g. for
//! the AR4-MK5) implement the same trait, typically as plugins.
//!
//! Kinematics output is a *proposal*: the motion engine still validates every
//! joint target against limits before anything moves.

use thiserror::Error;

/// Task-space pose. For joint-space robots this is simply the joint vector;
/// Cartesian solvers use `[x, y, z, rx, ry, rz]` (mm, deg).
#[derive(Debug, Clone, PartialEq)]
pub struct Pose(pub Vec<f64>);

#[derive(Debug, Clone, PartialEq, Error)]
pub enum KinematicsError {
    #[error("expected {expected} joint values, got {got}")]
    JointCount { expected: usize, got: usize },
    #[error("expected {expected} pose values, got {got}")]
    PoseSize { expected: usize, got: usize },
    #[error("non-finite value at index {0}")]
    NonFinite(usize),
    #[error("target pose is unreachable")]
    Unreachable,
}

pub trait Kinematics: Send + Sync {
    fn joint_count(&self) -> usize;
    /// Joint vector → pose.
    fn forward(&self, joints: &[f64]) -> Result<Pose, KinematicsError>;
    /// Pose → joint vector. `seed` is the current joint vector, used by
    /// solvers with multiple solutions to stay on the same branch.
    fn inverse(&self, pose: &Pose, seed: &[f64]) -> Result<Vec<f64>, KinematicsError>;
}

fn check(values: &[f64], expected: usize, pose: bool) -> Result<(), KinematicsError> {
    if values.len() != expected {
        return Err(if pose {
            KinematicsError::PoseSize { expected, got: values.len() }
        } else {
            KinematicsError::JointCount { expected, got: values.len() }
        });
    }
    match values.iter().position(|v| !v.is_finite()) {
        Some(i) => Err(KinematicsError::NonFinite(i)),
        None => Ok(()),
    }
}

/// Identity kinematics: the pose *is* the joint vector.
#[derive(Debug, Clone)]
pub struct JointIdentity {
    joints: usize,
}

impl JointIdentity {
    pub fn new(joints: usize) -> Self {
        Self { joints }
    }
}

impl Kinematics for JointIdentity {
    fn joint_count(&self) -> usize {
        self.joints
    }

    fn forward(&self, joints: &[f64]) -> Result<Pose, KinematicsError> {
        check(joints, self.joints, false)?;
        Ok(Pose(joints.to_vec()))
    }

    fn inverse(&self, pose: &Pose, seed: &[f64]) -> Result<Vec<f64>, KinematicsError> {
        check(seed, self.joints, false)?;
        check(&pose.0, self.joints, true)?;
        Ok(pose.0.clone())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn identity_round_trip() {
        let k = JointIdentity::new(3);
        let q = [10.0, -20.0, 30.0];
        let p = k.forward(&q).unwrap();
        assert_eq!(k.inverse(&p, &[0.0; 3]).unwrap(), q);
    }

    #[test]
    fn rejects_bad_input() {
        let k = JointIdentity::new(2);
        assert_eq!(k.forward(&[1.0]), Err(KinematicsError::JointCount { expected: 2, got: 1 }));
        assert_eq!(k.forward(&[1.0, f64::NAN]), Err(KinematicsError::NonFinite(1)));
        assert_eq!(k.inverse(&Pose(vec![1.0]), &[0.0, 0.0]), Err(KinematicsError::PoseSize { expected: 2, got: 1 }));
    }
}
