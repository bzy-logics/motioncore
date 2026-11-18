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

/// Version of the contract in this crate (`0.MINOR.PATCH` while MotionCore
/// is pre-1.0). Bump on every change: PATCH for additive, compatible changes;
/// MINOR for breaking changes and major features. Clients accept an engine
/// with the same `0.MINOR` (see [`is_compatible`]).
pub const API_VERSION: &str = "0.3.0";

/// Whether an engine reporting `api_version` speaks a contract compatible
/// with this crate's: same major and, while the major is 0, same minor.
pub fn is_compatible(api_version: &str) -> bool {
    fn major_minor(v: &str) -> Option<(u64, u64)> {
        let mut it = v.split('.');
        Some((it.next()?.parse().ok()?, it.next()?.parse().ok()?))
    }
    match (major_minor(API_VERSION), major_minor(api_version)) {
        (Some((0, a)), Some((0, b))) => a == b,
        (Some((a, _)), Some((b, _))) => a == b,
        _ => false,
    }
}

/// Trailing-metadata key carrying the precise MotionCore error code.
pub const ERROR_CODE_METADATA: &str = "motioncore-error-code";

/// The contract source, embedded for tooling (`motioncore engine-api proto`).
pub const PROTO_V1: &str = include_str!("../proto/motioncore/engine/v1/engine.proto");

#[cfg(test)]
mod tests {
    #[test]
    fn contract_version_matches_package() {
        assert!(super::PROTO_V1.contains("package motioncore.engine.v1;"));
        assert!(super::API_VERSION.starts_with("0."));
    }

    #[test]
    fn compatibility_is_same_zero_minor() {
        use super::is_compatible;
        assert!(is_compatible(super::API_VERSION));
        assert!(is_compatible("0.3.0"));
        assert!(is_compatible("0.3.99"));
        assert!(!is_compatible("0.2.0"));
        assert!(!is_compatible("0.4.0"));
        assert!(!is_compatible("1.2.0"));
        assert!(!is_compatible("garbage"));
        assert!(!is_compatible(""));
    }
}
