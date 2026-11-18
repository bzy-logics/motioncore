//! Versioned host-plugin contract.
//!
//! Everything a plugin author depends on lives here — the manifest format,
//! the permission vocabulary, the capabilities the engine core provides and
//! the [`Plugin`] trait implemented by builtin plugins. The runtime
//! (`motioncore-plugin-runtime`) depends on this crate, never the other way.
//!
//! Compatibility: [`API_VERSION`] changes only on breaking changes to this
//! crate. Manifests declaring a different `api_version` are rejected. It is
//! `"0"` while the plugin interface is unstable (pre-1.0); `"1"` will mark
//! the first stable interface.

use serde::{Deserialize, Serialize};

/// Host plugin API version: `"0"` while the interface is unstable.
pub const API_VERSION: &str = "0";

/// Permissions a plugin may request. Anything else is rejected.
pub const KNOWN_PERMISSIONS: &[&str] = &["robot.read_geometry", "robot.read_state", "motion.propose", "telemetry.read", "config.read"];

/// Capabilities provided by the engine core itself.
pub const CORE_CAPABILITIES: &[&str] = &["robot.joint_model", "mcu.stepper", "telemetry.stream"];

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum PluginKind {
    /// Trusted, compiled into the engine.
    Builtin,
    /// Isolated WASM component (execution is a later release).
    Wasm,
}

impl PluginKind {
    pub fn as_str(self) -> &'static str {
        match self {
            Self::Builtin => "builtin",
            Self::Wasm => "wasm",
        }
    }
}

/// `plugin.toml`.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
#[serde(deny_unknown_fields)]
pub struct Manifest {
    /// Reverse-DNS, lowercase, at least three segments.
    pub id: String,
    /// Semantic version.
    pub version: String,
    pub api_version: String,
    pub kind: PluginKind,
    #[serde(default)]
    pub description: String,
    /// Required for `wasm` plugins, relative to the manifest directory.
    pub entrypoint: Option<String>,
    /// Hex SHA-256 of the entrypoint, verified before activation.
    pub sha256: Option<String>,
    #[serde(default)]
    pub provides: Vec<String>,
    #[serde(default)]
    pub requires: Vec<String>,
    #[serde(default)]
    pub permissions: Vec<String>,
}

impl Manifest {
    /// Source-independent manifest checks (id, version, API, permissions).
    pub fn validate(&self) -> Result<(), String> {
        if !valid_id(&self.id) {
            return Err(format!("invalid id '{}' (expected reverse-DNS, e.g. org.example.name)", self.id));
        }
        semver::Version::parse(&self.version).map_err(|e| format!("invalid version '{}': {e}", self.version))?;
        if self.api_version != API_VERSION {
            return Err(format!("unsupported api_version '{}' (engine implements '{API_VERSION}')", self.api_version));
        }
        if let Some(p) = self.permissions.iter().find(|p| !KNOWN_PERMISSIONS.contains(&p.as_str())) {
            return Err(format!("unknown permission '{p}'"));
        }
        if self.provides.is_empty() {
            return Err("plugin must provide at least one capability".into());
        }
        Ok(())
    }
}

fn valid_id(id: &str) -> bool {
    let parts: Vec<&str> = id.split('.').collect();
    parts.len() >= 3 && parts.iter().all(|p| !p.is_empty() && p.chars().all(|c| c.is_ascii_lowercase() || c.is_ascii_digit() || c == '_'))
}

/// Read-only context handed to a plugin on activation.
#[derive(Debug, Clone, Default)]
pub struct PluginContext {
    pub robot_name: String,
    pub axis_ids: Vec<String>,
}

/// Implemented by builtin plugins.
pub trait Plugin: Send {
    fn manifest(&self) -> Manifest;
    fn activate(&mut self, ctx: &PluginContext) -> Result<(), String>;
    fn deactivate(&mut self) {}
}

#[cfg(test)]
mod tests {
    use super::*;

    fn m() -> Manifest {
        Manifest {
            id: "org.example.thing".into(),
            version: "1.0.0".into(),
            api_version: API_VERSION.into(),
            kind: PluginKind::Builtin,
            description: String::new(),
            entrypoint: None,
            sha256: None,
            provides: vec!["x".into()],
            requires: vec![],
            permissions: vec!["robot.read_state".into()],
        }
    }

    #[test]
    fn validates() {
        assert!(m().validate().is_ok());
        assert!(Manifest { id: "Bad".into(), ..m() }.validate().is_err());
        assert!(Manifest { version: "x".into(), ..m() }.validate().is_err());
        assert!(Manifest { api_version: "1".into(), ..m() }.validate().is_err());
        assert!(Manifest { permissions: vec!["hw.raw".into()], ..m() }.validate().is_err());
        assert!(Manifest { provides: vec![], ..m() }.validate().is_err());
    }

    #[test]
    fn parses_toml() {
        let t = "id = \"org.a.b\"\nversion = \"0.1.0\"\napi_version = \"0\"\nkind = \"wasm\"\nentrypoint = \"p.wasm\"\nprovides = [\"k\"]\n";
        let m: Manifest = toml::from_str(t).unwrap();
        assert_eq!(m.kind, PluginKind::Wasm);
        assert!(toml::from_str::<Manifest>(&format!("{t}bogus = 1\n")).is_err());
    }
}
