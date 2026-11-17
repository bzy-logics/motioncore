//! Engine command errors and their mapping onto the gRPC contract.

use motioncore_engine_api::ERROR_CODE_METADATA;
use tonic::{metadata::MetadataValue, Code, Status};

/// A command failure with a stable MotionCore error code.
#[derive(Debug, Clone, PartialEq)]
pub struct ApiError {
    pub code: &'static str,
    pub message: String,
}

impl ApiError {
    pub fn new(code: &'static str, message: impl Into<String>) -> Self {
        Self { code, message: message.into() }
    }
    pub fn bad_request(m: impl Into<String>) -> Self {
        Self::new("bad_request", m)
    }
    pub fn invalid_state(m: impl Into<String>) -> Self {
        Self::new("invalid_state", m)
    }
    pub fn not_found(m: impl Into<String>) -> Self {
        Self::new("not_found", m)
    }
    pub fn mcu(m: impl std::fmt::Display) -> Self {
        Self::new("mcu_error", m.to_string())
    }
    pub fn internal(m: impl std::fmt::Display) -> Self {
        Self::new("internal", m.to_string())
    }

    /// gRPC status code for this error (see the contract header).
    pub fn grpc_code(&self) -> Code {
        match self.code {
            "bad_request" | "validation_failed" => Code::InvalidArgument,
            "unknown_axis" | "not_found" => Code::NotFound,
            "not_homed" | "invalid_state" | "collision" => Code::FailedPrecondition,
            "mcu_error" => Code::Unavailable,
            _ => Code::Internal,
        }
    }
}

impl std::fmt::Display for ApiError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}: {}", self.code, self.message)
    }
}

impl From<ApiError> for Status {
    fn from(e: ApiError) -> Self {
        let mut status = Status::new(e.grpc_code(), e.message.clone());
        status.metadata_mut().insert(ERROR_CODE_METADATA, MetadataValue::from_static(e.code));
        status
    }
}

/// Recover the MotionCore error code from a status (client side).
pub fn error_code(status: &Status) -> Option<&str> {
    status.metadata().get(ERROR_CODE_METADATA).and_then(|v| v.to_str().ok())
}
