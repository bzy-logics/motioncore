//! MotionCore engine API — versioned gRPC contract.
//!
//! The source of truth is `proto/motioncore/engine/v1/engine.proto`. Releases
//! are tagged `engine-api/vX.Y.Z`; external clients (e.g. `ratavyr`)
//! generate bindings from a tagged release, never from an edited copy.

/// Generated messages, clients and server traits for `motioncore.engine.v1`.
pub mod v1 {
    #![allow(clippy::all)]
    tonic::include_proto!("motioncore.engine.v1");
}

/// Semantic version of the contract in this crate. Bump on every change:
/// minor for additive changes, major (with a new proto package) for breaking ones.
pub const API_VERSION: &str = "1.1.0";

/// Trailing-metadata key carrying the precise MotionCore error code.
pub const ERROR_CODE_METADATA: &str = "motioncore-error-code";

/// The contract source, embedded for tooling (`motioncore engine-api proto`).
pub const PROTO_V1: &str = include_str!("../proto/motioncore/engine/v1/engine.proto");

#[cfg(test)]
mod tests {
    #[test]
    fn contract_version_matches_package() {
        assert!(super::PROTO_V1.contains("package motioncore.engine.v1;"));
        assert!(super::API_VERSION.starts_with("1."));
    }
}
