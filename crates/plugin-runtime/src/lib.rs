//! Host plugin runtime: manifests, validation, dependency resolution,
//! lifecycle and the capability registry.
//!
//! Lifecycle: `discover → validate → resolve → initialize → activate → (monitor) → deactivate`.
//!
//! Plugins never touch hardware. They *propose* (geometry, kinematics,
//! trajectories); the core validates and arbitrates. A failing plugin is moved
//! to `failed` together with everything that depends on it — it can never
//! silently change robot state.
//!
//! Trusted **builtin** plugins (see `plugins/builtin`) are compiled into the engine. Manifests
//! with `kind = "wasm"` are fully parsed, validated and resolved, but
//! activation reports that the Wasmtime runtime is not part of this build
//! (an open design decision in the specification, section 16).

use serde::{Deserialize, Serialize};
use std::collections::{BTreeMap, HashMap, HashSet};
use std::path::{Path, PathBuf};

pub use motioncore_plugin_sdk::{Manifest, Plugin, PluginContext, PluginKind, API_VERSION, CORE_CAPABILITIES, KNOWN_PERMISSIONS};

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum PluginState {
    Discovered,
    Validated,
    Resolved,
    Initialized,
    Active,
    Failed,
    Deactivated,
}

/// Runtime view of one plugin.
#[derive(Debug, Clone, Serialize)]
pub struct PluginInfo {
    pub id: String,
    pub version: String,
    pub api_version: String,
    pub kind: PluginKind,
    pub description: String,
    pub provides: Vec<String>,
    pub requires: Vec<String>,
    pub permissions: Vec<String>,
    pub state: PluginState,
    pub error: Option<String>,
}

enum Source {
    Builtin(Box<dyn Plugin>),
    Wasm { dir: PathBuf },
}

struct Entry {
    manifest: Manifest,
    source: Source,
    state: PluginState,
    error: Option<String>,
}

/// Registry of plugins and the capabilities they provide.
pub struct Registry {
    entries: BTreeMap<String, Entry>,
    core_capabilities: HashSet<String>,
    /// capability -> providing plugin id ("core" for engine-provided).
    providers: HashMap<String, String>,
    order: Vec<String>,
}

impl Registry {
    /// `core_capabilities` are provided by the engine itself.
    pub fn new<I: IntoIterator<Item = S>, S: Into<String>>(core_capabilities: I) -> Self {
        Self {
            entries: BTreeMap::new(),
            core_capabilities: core_capabilities.into_iter().map(Into::into).collect(),
            providers: HashMap::new(),
            order: Vec::new(),
        }
    }

    pub fn add_builtin(&mut self, plugin: Box<dyn Plugin>) {
        let manifest = plugin.manifest();
        self.insert(manifest, Source::Builtin(plugin));
    }

    /// Discover `*/plugin.toml` manifests under `dir`.
    pub fn discover_dir(&mut self, dir: &Path) -> Vec<String> {
        let mut warnings = Vec::new();
        let Ok(rd) = std::fs::read_dir(dir) else {
            return warnings;
        };
        let mut paths: Vec<_> = rd.filter_map(|e| e.ok()).map(|e| e.path().join("plugin.toml")).filter(|p| p.is_file()).collect();
        paths.sort();
        for path in paths {
            match std::fs::read_to_string(&path).map_err(|e| e.to_string()).and_then(|t| toml::from_str::<Manifest>(&t).map_err(|e| e.to_string())) {
                Ok(m) => {
                    let dir = path.parent().unwrap().to_path_buf();
                    self.insert(m, Source::Wasm { dir });
                }
                Err(e) => warnings.push(format!("{}: {e}", path.display())),
            }
        }
        warnings
    }

    fn insert(&mut self, manifest: Manifest, source: Source) {
        let id = manifest.id.clone();
        if self.entries.contains_key(&id) {
            if let Some(e) = self.entries.get_mut(&id) {
                e.state = PluginState::Failed;
                e.error = Some("duplicate plugin id".into());
            }
            return;
        }
        self.entries.insert(id, Entry { manifest, source, state: PluginState::Discovered, error: None });
    }

    /// Validate manifests, resolve dependencies, then initialise and activate
    /// in dependency order. Returns ids of plugins that failed.
    pub fn start(&mut self, ctx: &PluginContext) -> Vec<String> {
        // validate
        for e in self.entries.values_mut() {
            if e.state != PluginState::Discovered {
                continue;
            }
            match validate_manifest(&e.manifest, &e.source) {
                Ok(()) => e.state = PluginState::Validated,
                Err(msg) => fail(e, msg),
            }
        }
        // capability providers
        self.providers.clear();
        for c in &self.core_capabilities {
            self.providers.insert(c.clone(), "core".into());
        }
        let ids: Vec<String> = self.entries.keys().cloned().collect();
        for id in &ids {
            let e = &self.entries[id];
            if e.state != PluginState::Validated {
                continue;
            }
            let provides = e.manifest.provides.clone();
            let mut conflict = None;
            for cap in &provides {
                if let Some(owner) = self.providers.get(cap) {
                    conflict = Some(format!("capability '{cap}' already provided by {owner}"));
                }
            }
            match conflict {
                Some(msg) => fail(self.entries.get_mut(id).unwrap(), msg),
                None => {
                    for cap in provides {
                        self.providers.insert(cap, id.clone());
                    }
                }
            }
        }
        // resolve (topological order over requires -> provider)
        self.order.clear();
        let mut visiting = HashSet::new();
        for id in &ids {
            self.visit(id, &mut visiting);
        }
        // initialise + activate in order
        for id in self.order.clone() {
            let deps_ok = self.dependencies(&id).iter().all(|d| d == "core" || self.entries.get(d).is_some_and(|e| e.state == PluginState::Active));
            let e = self.entries.get_mut(&id).unwrap();
            if e.state != PluginState::Resolved {
                continue;
            }
            if !deps_ok {
                fail(e, "a dependency failed to activate".into());
                continue;
            }
            e.state = PluginState::Initialized;
            let result = match &mut e.source {
                Source::Builtin(p) => p.activate(ctx),
                Source::Wasm { .. } => Err("WASM runtime is not included in this build (v0.1 runs builtin plugins only)".into()),
            };
            match result {
                Ok(()) => e.state = PluginState::Active,
                Err(msg) => fail(e, msg),
            }
        }
        // Failed providers must not count as providing.
        self.providers.retain(|_, owner| owner == "core" || self.entries.get(owner).is_some_and(|e| e.state == PluginState::Active));
        self.entries.iter().filter(|(_, e)| e.state == PluginState::Failed).map(|(id, _)| id.clone()).collect()
    }

    fn dependencies(&self, id: &str) -> Vec<String> {
        self.entries[id].manifest.requires.iter().filter_map(|c| self.providers.get(c).cloned()).collect()
    }

    fn visit(&mut self, id: &str, visiting: &mut HashSet<String>) -> bool {
        let Some(e) = self.entries.get(id) else { return false };
        match e.state {
            PluginState::Resolved => return true,
            PluginState::Validated => {}
            _ => return false,
        }
        if !visiting.insert(id.to_string()) {
            fail(self.entries.get_mut(id).unwrap(), "dependency cycle".into());
            return false;
        }
        let requires = e.manifest.requires.clone();
        let mut error = None;
        for cap in &requires {
            match self.providers.get(cap).cloned() {
                None => {
                    error = Some(format!("missing capability '{cap}'"));
                    break;
                }
                Some(p) if p == "core" => {}
                Some(p) if p == id => {
                    error = Some(format!("plugin requires its own capability '{cap}'"));
                    break;
                }
                Some(p) => {
                    if !self.visit(&p, visiting) {
                        error = Some(format!("dependency '{p}' (for '{cap}') unavailable"));
                        break;
                    }
                }
            }
        }
        visiting.remove(id);
        let e = self.entries.get_mut(id).unwrap();
        if e.state != PluginState::Validated {
            return e.state == PluginState::Resolved;
        }
        match error {
            Some(msg) => {
                fail(e, msg);
                false
            }
            None => {
                e.state = PluginState::Resolved;
                self.order.push(id.to_string());
                true
            }
        }
    }

    /// Mark a running plugin as failed (e.g. it panicked or misbehaved) and
    /// cascade to dependents. Returns all plugins that changed to `failed`.
    pub fn mark_failed(&mut self, id: &str, reason: &str) -> Vec<String> {
        let mut changed = Vec::new();
        let mut queue = vec![(id.to_string(), reason.to_string())];
        while let Some((pid, why)) = queue.pop() {
            let Some(e) = self.entries.get_mut(&pid) else { continue };
            if e.state == PluginState::Failed {
                continue;
            }
            if let Source::Builtin(p) = &mut e.source {
                p.deactivate();
            }
            fail(e, why);
            changed.push(pid.clone());
            let lost: Vec<String> = e.manifest.provides.clone();
            self.providers.retain(|c, _| !lost.contains(c));
            for (other, oe) in &self.entries {
                if oe.state == PluginState::Active && oe.manifest.requires.iter().any(|c| lost.contains(c)) {
                    queue.push((other.clone(), format!("dependency '{pid}' failed")));
                }
            }
        }
        changed
    }

    pub fn deactivate_all(&mut self) {
        for id in self.order.iter().rev() {
            if let Some(e) = self.entries.get_mut(id) {
                if e.state == PluginState::Active {
                    if let Source::Builtin(p) = &mut e.source {
                        p.deactivate();
                    }
                    e.state = PluginState::Deactivated;
                }
            }
        }
    }

    pub fn is_active(&self, id: &str) -> bool {
        self.entries.get(id).is_some_and(|e| e.state == PluginState::Active)
    }

    pub fn has_capability(&self, cap: &str) -> bool {
        self.providers.contains_key(cap)
    }

    pub fn info(&self) -> Vec<PluginInfo> {
        self.entries
            .values()
            .map(|e| PluginInfo {
                id: e.manifest.id.clone(),
                version: e.manifest.version.clone(),
                api_version: e.manifest.api_version.clone(),
                kind: e.manifest.kind,
                description: e.manifest.description.clone(),
                provides: e.manifest.provides.clone(),
                requires: e.manifest.requires.clone(),
                permissions: e.manifest.permissions.clone(),
                state: e.state,
                error: e.error.clone(),
            })
            .collect()
    }
}

fn fail(e: &mut Entry, msg: String) {
    e.state = PluginState::Failed;
    e.error = Some(msg);
}

fn validate_manifest(m: &Manifest, source: &Source) -> Result<(), String> {
    m.validate()?;
    match (m.kind, source) {
        (PluginKind::Builtin, Source::Builtin(_)) => Ok(()),
        (PluginKind::Builtin, Source::Wasm { .. }) => Err("kind 'builtin' cannot be loaded from disk".into()),
        (PluginKind::Wasm, Source::Builtin(_)) => Err("builtin source with kind 'wasm'".into()),
        (PluginKind::Wasm, Source::Wasm { dir }) => {
            let entry = m.entrypoint.as_deref().ok_or("wasm plugin requires 'entrypoint'")?;
            if entry.contains("..") || Path::new(entry).is_absolute() {
                return Err("entrypoint must be a relative path inside the plugin directory".into());
            }
            let path = dir.join(entry);
            let bytes = std::fs::read(&path).map_err(|e| format!("cannot read entrypoint {}: {e}", path.display()))?;
            let expected = m.sha256.as_deref().ok_or("wasm plugin requires 'sha256'")?;
            let actual = hex(&sha256(&bytes));
            if !actual.eq_ignore_ascii_case(expected) {
                return Err(format!("checksum mismatch for {entry}"));
            }
            Ok(())
        }
    }
}

fn hex(b: &[u8]) -> String {
    b.iter().map(|x| format!("{x:02x}")).collect()
}

fn sha256(data: &[u8]) -> [u8; 32] {
    use sha2::{Digest, Sha256};
    Sha256::digest(data).into()
}

#[cfg(test)]
mod tests;
