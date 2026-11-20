//! Robot kinematics: one model interface, one forward-kinematics
//! implementation, and what is built on it.
//!
//! - [`model`]: [`KinematicModel`], [`JointTree`] and [`joint_frames`], the
//!   only forward kinematics.
//! - [`chain`]: serial chains with tool pose, Jacobian, numerical IK and
//!   straight-line Cartesian paths ([`chain::Chain`] implements
//!   `KinematicModel`).
//! - [`collision`]: the certified capsule collision check, which takes its
//!   frames from a `KinematicModel`.
//!
//! Kinematics output is a *proposal*: the motion engine still validates every
//! joint target against limits and the collision model before anything moves.

pub mod chain;
pub mod collision;
pub mod math;
pub mod model;

pub use model::{joint_frames, JointSpec, JointTree, KinematicModel};
