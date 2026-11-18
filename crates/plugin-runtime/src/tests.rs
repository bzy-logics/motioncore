use super::*;

struct Fake {
    m: Manifest,
    fail: bool,
}

impl Plugin for Fake {
    fn manifest(&self) -> Manifest {
        self.m.clone()
    }
    fn activate(&mut self, _: &PluginContext) -> Result<(), String> {
        if self.fail { Err("boom".into()) } else { Ok(()) }
    }
}

fn fake(id: &str, provides: &[&str], requires: &[&str]) -> Box<Fake> {
    Box::new(Fake {
        m: Manifest {
            id: id.into(),
            version: "1.2.3".into(),
            api_version: API_VERSION.into(),
            kind: PluginKind::Builtin,
            description: String::new(),
            entrypoint: None,
            sha256: None,
            provides: provides.iter().map(|s| s.to_string()).collect(),
            requires: requires.iter().map(|s| s.to_string()).collect(),
            permissions: vec![],
        },
        fail: false,
    })
}

fn state(r: &Registry, id: &str) -> PluginState {
    r.info().into_iter().find(|i| i.id == id).unwrap().state
}

fn ctx() -> PluginContext {
    PluginContext { robot_name: "t".into(), axis_ids: vec!["j1".into()] }
}

#[test]
fn resolves_in_dependency_order_regardless_of_insertion() {
    let mut r = Registry::new(["core.cap"]);
    r.add_builtin(fake("org.t.c", &["c"], &["b"]));
    r.add_builtin(fake("org.t.b", &["b"], &["a"]));
    r.add_builtin(fake("org.t.a", &["a"], &["core.cap"]));
    assert!(r.start(&ctx()).is_empty());
    assert_eq!(r.order, ["org.t.a", "org.t.b", "org.t.c"]);
}

#[test]
fn missing_capability_fails_plugin_and_dependents() {
    let mut r = Registry::new(Vec::<String>::new());
    r.add_builtin(fake("org.t.a", &["a"], &["nope"]));
    r.add_builtin(fake("org.t.b", &["b"], &["a"]));
    r.add_builtin(fake("org.t.ok", &["ok"], &[]));
    let mut failed = r.start(&ctx());
    failed.sort();
    assert_eq!(failed, ["org.t.a", "org.t.b"]);
    assert!(r.is_active("org.t.ok"));
    assert!(!r.has_capability("a"));
}

#[test]
fn activation_failure_cascades() {
    let mut r = Registry::new(Vec::<String>::new());
    let mut a = fake("org.t.a", &["a"], &[]);
    a.fail = true;
    r.add_builtin(a);
    r.add_builtin(fake("org.t.b", &["b"], &["a"]));
    r.start(&ctx());
    assert_eq!(state(&r, "org.t.a"), PluginState::Failed);
    assert_eq!(state(&r, "org.t.b"), PluginState::Failed);
}

#[test]
fn cycles_are_detected() {
    let mut r = Registry::new(Vec::<String>::new());
    r.add_builtin(fake("org.t.a", &["a"], &["b"]));
    r.add_builtin(fake("org.t.b", &["b"], &["a"]));
    assert_eq!(r.start(&ctx()).len(), 2);
}

#[test]
fn manifest_validation() {
    let mut r = Registry::new(Vec::<String>::new());
    let mut bad_ver = fake("org.t.v", &["v"], &[]);
    bad_ver.m.version = "one".into();
    let mut bad_api = fake("org.t.api", &["api"], &[]);
    bad_api.m.api_version = "99".into();
    let mut bad_perm = fake("org.t.perm", &["perm"], &[]);
    bad_perm.m.permissions = vec!["hardware.raw_steps".into()];
    r.add_builtin(bad_ver);
    r.add_builtin(bad_api);
    r.add_builtin(bad_perm);
    r.add_builtin(fake("BadId", &["x"], &[]));
    r.add_builtin(fake("org.t.dup1", &["same"], &[]));
    r.add_builtin(fake("org.t.dup2", &["same"], &[]));
    let failed = r.start(&ctx());
    assert_eq!(failed.len(), 5, "{:?}", r.info());
    let perm = r.info().into_iter().find(|i| i.id == "org.t.perm").unwrap();
    assert!(perm.error.unwrap().contains("unknown permission"));
}

#[test]
fn runtime_failure_cascades_to_dependents() {
    let mut r = Registry::new(Vec::<String>::new());
    r.add_builtin(fake("org.t.a", &["a"], &[]));
    r.add_builtin(fake("org.t.b", &["b"], &["a"]));
    r.add_builtin(fake("org.t.c", &["c"], &[]));
    r.start(&ctx());
    let mut changed = r.mark_failed("org.t.a", "panicked");
    changed.sort();
    assert_eq!(changed, ["org.t.a", "org.t.b"]);
    assert!(r.is_active("org.t.c"));
    assert!(!r.has_capability("b"));
}

#[test]
fn wasm_manifests_are_validated_from_disk() {
    let dir = std::env::temp_dir().join(format!("mc-plugin-test-{}", std::process::id()));
    let p = dir.join("kin");
    std::fs::create_dir_all(&p).unwrap();
    std::fs::write(p.join("plugin.wasm"), b"\0asm\x01\0\0\0").unwrap();
    let sum = hex(&sha256(b"\0asm\x01\0\0\0"));
    std::fs::write(
        p.join("plugin.toml"),
        format!("id = \"org.t.kin\"\nversion = \"0.1.0\"\napi_version = \"0\"\nkind = \"wasm\"\nentrypoint = \"plugin.wasm\"\nsha256 = \"{sum}\"\nprovides = [\"kinematics.fk\"]\n"),
    )
    .unwrap();
    let bad = dir.join("bad");
    std::fs::create_dir_all(&bad).unwrap();
    std::fs::write(bad.join("plugin.wasm"), b"x").unwrap();
    std::fs::write(
        bad.join("plugin.toml"),
        "id = \"org.t.bad\"\nversion = \"0.1.0\"\napi_version = \"0\"\nkind = \"wasm\"\nentrypoint = \"plugin.wasm\"\nsha256 = \"00\"\nprovides = [\"x\"]\n",
    )
    .unwrap();

    let mut r = Registry::new(Vec::<String>::new());
    assert!(r.discover_dir(&dir).is_empty());
    r.start(&ctx());
    let info = r.info();
    let kin = info.iter().find(|i| i.id == "org.t.kin").unwrap();
    assert_eq!(kin.state, PluginState::Failed);
    assert!(kin.error.as_ref().unwrap().contains("WASM runtime"), "validated, then refused at activation");
    let bad = info.iter().find(|i| i.id == "org.t.bad").unwrap();
    assert!(bad.error.as_ref().unwrap().contains("checksum"));
    std::fs::remove_dir_all(&dir).ok();
}
