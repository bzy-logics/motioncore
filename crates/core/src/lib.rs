//! MotionCore runtime: the authoritative robot state, motion pipeline, MCU
//! management, firmware jobs and the gRPC control plane.

pub mod engine;
pub mod error;
pub mod firmware;
pub mod grpc;
