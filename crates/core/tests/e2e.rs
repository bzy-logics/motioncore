//! End-to-end tests over the real gRPC contract: engine + EngineService +
//! virtual MCU (shared firmware runtime). Covers the v0.1 acceptance flow in
//! simulation. Responses are projected into JSON so assertions read like the
//! REST API clients see.

use std::path::Path;
use std::time::{Duration, Instant};

use motioncore_core::error::error_code;
use motioncore_core::{engine, firmware::FirmwareJobs, grpc};
use motioncore_engine_api::v1 as pb;
use motioncore_plugin_runtime::{PluginContext, Registry, CORE_CAPABILITIES};
use pb::engine_service_client::EngineServiceClient;
use pb::firmware_service_client::FirmwareServiceClient;
use serde_json::{json, Value};
use tonic::transport::Channel;

struct Client {
    engine: EngineServiceClient<Channel>,
    firmware: FirmwareServiceClient<Channel>,
}

fn lower(name: &str, prefix: &str) -> Value {
    json!(name.trim_start_matches(prefix).to_lowercase())
}

fn telemetry_json(t: pb::Telemetry) -> Value {
    let m = t.mcu.unwrap_or_default();
    json!({
        "state": lower(pb::RobotState::try_from(t.state).unwrap().as_str_name(), "ROBOT_STATE_"),
        "fault": t.fault.map(|f| json!({
            "code": lower(pb::FaultCode::try_from(f.code).unwrap().as_str_name(), "FAULT_CODE_"),
            "message": f.message,
        })),
        "mcu": {"connected": m.connected, "target": m.target, "capabilities": m.capabilities, "build_hash": m.build_hash},
        "axes": t.axes.iter().map(|a| json!({
            "id": a.id, "homed": a.homed, "enabled": a.enabled,
            "commanded_position": a.commanded_position, "measured_position": a.measured_position,
            "velocity": a.velocity, "endstop": a.endstop,
        })).collect::<Vec<_>>(),
    })
}

fn err(s: tonic::Status) -> Value {
    json!({"code": error_code(&s).unwrap_or("?"), "message": s.message()})
}

fn ok<T>(r: Result<tonic::Response<T>, tonic::Status>) -> Result<Value, Value> {
    r.map(|_| json!({})).map_err(err)
}

fn f(p: &Value, k: &str) -> f64 {
    p[k].as_f64().unwrap_or(f64::NAN)
}

fn s(p: &Value, k: &str) -> String {
    p[k].as_str().unwrap_or_default().to_string()
}

impl Client {
    /// Call by method name with JSON params (mirrors the REST command API).
    async fn call(&mut self, method: &str, p: Value) -> Result<Value, Value> {
        let e = &mut self.engine;
        let vel = p["velocity"].as_f64();
        match method {
            "status" => e.get_status(pb::GetStatusRequest {}).await.map(|r| telemetry_json(r.into_inner())).map_err(err),
            "config" => e.get_robot_config(pb::GetRobotConfigRequest {}).await.map(|r| {
                let c = r.into_inner();
                json!({"axes": c.axes.iter().map(|a| json!({"id": a.id, "unit": a.unit, "actuator": a.actuator})).collect::<Vec<_>>()})
            }).map_err(err),
            "plugins" => e.list_plugins(pb::ListPluginsRequest {}).await.map(|r| {
                json!(r.into_inner().plugins.iter().map(|p| json!({
                    "id": p.id,
                    "state": lower(pb::PluginState::try_from(p.state).unwrap().as_str_name(), "PLUGIN_STATE_"),
                })).collect::<Vec<_>>())
            }).map_err(err),
            "enable" => ok(e.enable(pb::EnableRequest {}).await),
            "disable" => ok(e.disable(pb::DisableRequest {}).await),
            "stop" => ok(e.stop(pb::StopRequest {}).await),
            "estop" => ok(e.emergency_stop(pb::EmergencyStopRequest {}).await),
            "reset_fault" => ok(e.reset_fault(pb::ResetFaultRequest {}).await),
            "jog" => {
                if p.get("delta").is_none() {
                    return Err(json!({"code": "bad_request"}));
                }
                ok(e.jog(pb::JogRequest { axis: s(&p, "axis"), delta: f(&p, "delta"), velocity: vel }).await)
            }
            "move_to" => ok(e.move_to(pb::MoveToRequest { axis: s(&p, "axis"), position: f(&p, "position"), velocity: vel }).await),
            "home" => ok(e.home(pb::HomeRequest { axis: s(&p, "axis") }).await),
            "debug_disconnect_mcu" => ok(e.debug_disconnect_mcu(pb::DebugDisconnectMcuRequest {}).await),
            "debug_fail_plugin" => ok(e.debug_fail_plugin(pb::DebugFailPluginRequest { id: s(&p, "id") }).await),
            other => panic!("unknown method {other}"),
        }
    }

    async fn status(&mut self) -> Value {
        self.call("status", json!({})).await.unwrap()
    }

    async fn wait_for<F: Fn(&Value) -> bool>(&mut self, what: &str, timeout: Duration, f: F) -> Value {
        let deadline = Instant::now() + timeout;
        loop {
            let s = self.status().await;
            if f(&s) {
                return s;
            }
            if Instant::now() > deadline {
                panic!("timed out waiting for {what}; last status: {s:#}");
            }
            tokio::time::sleep(Duration::from_millis(25)).await;
        }
    }

    async fn wait_state(&mut self, state: &str, timeout: Duration) -> Value {
        self.wait_for(state, timeout, |s| s["state"] == state).await
    }
}

fn profile(name: &str) -> std::path::PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("../../robot-profiles").join(name)
}

async fn start(fault_injection: bool) -> Client {
    start_with("single-axis-virtual.toml", fault_injection).await
}

async fn start_with(name: &str, fault_injection: bool) -> Client {
    let path = profile(name);
    let robot = motioncore_device_graph::load(&path).unwrap();
    let axis_ids: Vec<String> = robot.file.axes.iter().map(|a| a.id.clone()).collect();
    let mut reg = Registry::new(CORE_CAPABILITIES.iter().copied());
    for p in motioncore_builtin_plugins::all() {
        reg.add_builtin(p);
    }
    assert!(reg.start(&PluginContext { robot_name: "t".into(), axis_ids }).is_empty());
    let eng = engine::Engine::new(robot, reg, engine::Options { fault_injection }).unwrap();
    eng.spawn();
    let jobs = FirmwareJobs::new(None, path);
    // Reserve a free port, then serve on it.
    let port = std::net::TcpListener::bind("127.0.0.1:0").unwrap().local_addr().unwrap().port();
    let addr = format!("127.0.0.1:{port}");
    tokio::spawn(grpc::serve(eng, jobs, grpc::ListenAddr::parse(&addr).unwrap(), std::future::pending()));
    let url = format!("http://{addr}");
    let channel = loop {
        match Channel::from_shared(url.clone()).unwrap().connect().await {
            Ok(c) => break c,
            Err(_) => tokio::time::sleep(Duration::from_millis(20)).await,
        }
    };
    let mut c = Client { engine: EngineServiceClient::new(channel.clone()), firmware: FirmwareServiceClient::new(channel) };
    c.wait_state("disabled", Duration::from_secs(5)).await;
    c
}

fn axis(s: &Value) -> &Value {
    &s["axes"][0]
}

const T: Duration = Duration::from_secs(8);

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn full_single_axis_flow() {
    let mut c = start(false).await;

    // Connected virtual MCU reports its capabilities.
    let s = c.status().await;
    assert_eq!(s["mcu"]["connected"], true);
    assert_eq!(s["mcu"]["target"], "virtual");
    assert!(s["mcu"]["capabilities"].as_array().unwrap().contains(&json!("stepper.step_dir")));

    // Config and plugins are exposed per the contract.
    let cfg = c.call("config", json!({})).await.unwrap();
    assert_eq!(cfg["axes"][0]["id"], "j1");
    assert_eq!(cfg["axes"][0]["unit"], "deg");
    let plugins = c.call("plugins", json!({})).await.unwrap();
    assert!(plugins.as_array().unwrap().iter().all(|p| p["state"] == "active"));

    // Motion is refused while disabled.
    let e = c.call("jog", json!({"axis": "j1", "delta": 5.0})).await.unwrap_err();
    assert_eq!(e["code"], "invalid_state");

    c.call("enable", json!({})).await.unwrap();
    c.wait_state("ready", T).await;

    // Unhomed: absolute moves refused, relative jog allowed.
    let e = c.call("move_to", json!({"axis": "j1", "position": 10.0})).await.unwrap_err();
    assert_eq!(e["code"], "not_homed");
    c.call("jog", json!({"axis": "j1", "delta": 5.0})).await.unwrap();
    assert_eq!(c.status().await["state"], "running");
    let e = c.call("jog", json!({"axis": "j1", "delta": 1.0})).await.unwrap_err();
    assert_eq!(e["code"], "invalid_state", "one motion at a time");
    let s = c.wait_state("ready", T).await;
    assert!((axis(&s)["measured_position"].as_f64().unwrap() - 5.0).abs() < 0.05, "{s:#}");

    // Validation errors.
    let e = c.call("jog", json!({"axis": "j9", "delta": 1.0})).await.unwrap_err();
    assert_eq!(e["code"], "unknown_axis");
    let e = c.call("jog", json!({"axis": "j1", "delta": 1e9})).await.unwrap_err();
    assert_eq!(e["code"], "validation_failed");
    let e = c.call("jog", json!({"axis": "j1"})).await.unwrap_err();
    assert_eq!(e["code"], "bad_request");

    // Homing: switch at -170 deg; search at 30 deg/s from +5 deg.
    c.call("home", json!({"axis": "j1"})).await.unwrap();
    assert_eq!(c.status().await["state"], "homing");
    let s = c.wait_for("homed", Duration::from_secs(10), |s| axis(s)["homed"] == true && s["state"] == "ready").await;
    assert!((axis(&s)["measured_position"].as_f64().unwrap() + 170.0).abs() < 0.05);

    // Absolute move after homing.
    c.call("move_to", json!({"axis": "j1", "position": -90.0})).await.unwrap();
    let s = c.wait_state("ready", T).await;
    assert!((axis(&s)["measured_position"].as_f64().unwrap() + 90.0).abs() < 0.05, "{s:#}");
    assert_eq!(axis(&s)["commanded_position"].as_f64().unwrap().round(), -90.0);

    // Soft limits enforced by the host after homing.
    let e = c.call("move_to", json!({"axis": "j1", "position": 200.0})).await.unwrap_err();
    assert_eq!(e["code"], "validation_failed");

    // Controlled stop mid-move: decelerates, stays enabled, position resynced.
    c.call("move_to", json!({"axis": "j1", "position": 90.0})).await.unwrap();
    c.wait_for("cruise", T, |s| axis(s)["velocity"].as_f64().unwrap() > 60.0).await;
    c.call("stop", json!({})).await.unwrap();
    let s = c.wait_state("ready", T).await;
    let p = axis(&s)["measured_position"].as_f64().unwrap();
    assert!(p > -90.0 && p < 85.0, "stopped early at {p}");
    assert!((axis(&s)["commanded_position"].as_f64().unwrap() - p).abs() < 0.05, "commanded resynced to MCU");
    assert_eq!(axis(&s)["enabled"], true);
    assert_eq!(s["fault"], Value::Null);

    // Emergency stop: latches, drives off, homing lost, reset does not resume.
    c.call("move_to", json!({"axis": "j1", "position": -150.0})).await.unwrap();
    tokio::time::sleep(Duration::from_millis(300)).await;
    c.call("estop", json!({})).await.unwrap();
    let s = c.wait_for("estopped & idle", T, |s| s["state"] == "emergency_stopped" && axis(s)["velocity"].as_f64().unwrap() == 0.0).await;
    assert_eq!(s["fault"]["code"], "emergency_stop");
    assert_eq!(axis(&s)["enabled"], false);
    assert_eq!(axis(&s)["homed"], false);
    let frozen = axis(&s)["measured_position"].as_f64().unwrap();
    tokio::time::sleep(Duration::from_millis(300)).await;
    assert_eq!(axis(&c.status().await)["measured_position"].as_f64().unwrap(), frozen, "no motion after e-stop");
    assert_eq!(c.call("enable", json!({})).await.unwrap_err()["code"], "invalid_state");
    c.call("reset_fault", json!({})).await.unwrap();
    let s = c.wait_state("disabled", T).await;
    assert_eq!(s["fault"], Value::Null);
    assert_eq!(axis(&s)["enabled"], false);
    assert_eq!(axis(&s)["measured_position"].as_f64().unwrap(), frozen);

    // After reset the robot can be re-enabled and jogged again.
    c.call("enable", json!({})).await.unwrap();
    c.call("jog", json!({"axis": "j1", "delta": -2.0})).await.unwrap();
    let s = c.wait_state("ready", T).await;
    assert!((axis(&s)["measured_position"].as_f64().unwrap() - (frozen - 2.0)).abs() < 0.05);
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn mcu_disconnect_faults_without_continued_motion() {
    let mut c = start(true).await;
    c.call("enable", json!({})).await.unwrap();
    c.call("jog", json!({"axis": "j1", "delta": 20.0})).await.unwrap();
    c.wait_for("moving", T, |s| axis(s)["velocity"].as_f64().unwrap() > 1.0).await;
    c.call("debug_disconnect_mcu", json!({})).await.unwrap();
    let s = c.wait_state("faulted", T).await;
    assert_eq!(s["fault"]["code"], "communication_lost");
    assert_eq!(s["mcu"]["connected"], false);
    assert_eq!(c.call("jog", json!({"axis": "j1", "delta": 1.0})).await.unwrap_err()["code"], "invalid_state");

    // Reset while disconnected -> disconnected; the engine then reconnects
    // (a fresh virtual MCU) and lands in disabled, never moving by itself.
    c.call("reset_fault", json!({})).await.unwrap();
    let s = c.wait_state("disabled", Duration::from_secs(10)).await;
    assert_eq!(s["mcu"]["connected"], true);
    assert_eq!(axis(&s)["homed"], false);
    assert_eq!(axis(&s)["enabled"], false);
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn plugin_crash_faults_robot() {
    let mut c = start(true).await;
    c.call("enable", json!({})).await.unwrap();
    c.call("jog", json!({"axis": "j1", "delta": 30.0})).await.unwrap();
    c.wait_for("moving", T, |s| axis(s)["velocity"].as_f64().unwrap() > 1.0).await;
    c.call("debug_fail_plugin", json!({"id": "org.motioncore.actuator.step_dir"})).await.unwrap();
    let s = c.wait_for("faulted & idle", T, |s| s["state"] == "faulted" && axis(s)["velocity"].as_f64().unwrap() == 0.0).await;
    assert_eq!(s["fault"]["code"], "plugin_failed");
    assert_eq!(axis(&s)["enabled"], false);
    let plugins = c.call("plugins", json!({})).await.unwrap();
    let p = plugins.as_array().unwrap().iter().find(|p| p["id"] == "org.motioncore.actuator.step_dir").unwrap();
    assert_eq!(p["state"], "failed");
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn fault_injection_is_off_by_default() {
    let mut c = start(false).await;
    let e = c.call("debug_disconnect_mcu", json!({})).await.unwrap_err();
    assert_eq!(e["code"], "bad_request");
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn info_and_event_stream() {
    let mut c = start(false).await;
    let info = c.engine.get_info(pb::GetInfoRequest {}).await.unwrap().into_inner();
    assert_eq!(info.api_version, motioncore_engine_api::API_VERSION);
    assert!(!info.fault_injection_enabled);

    let mut stream = c.engine.stream_events(pb::StreamEventsRequest {}).await.unwrap().into_inner();
    let first = stream.message().await.unwrap().unwrap();
    assert!(matches!(first.event, Some(pb::event::Event::Telemetry(_))), "first event is a snapshot");
    c.call("enable", json!({})).await.unwrap();
    let deadline = Instant::now() + T;
    loop {
        let ev = tokio::time::timeout(T, stream.message()).await.unwrap().unwrap().unwrap();
        if let Some(pb::event::Event::Log(l)) = ev.event {
            if l.message.contains("drives enabled") {
                break;
            }
        }
        assert!(Instant::now() < deadline, "no log event for enable");
    }
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn firmware_service_is_restricted() {
    let mut c = start(false).await;
    let targets = c.firmware.list_targets(pb::ListTargetsRequest {}).await.unwrap().into_inner().targets;
    assert!(targets.iter().any(|t| t.board == "stm32f411-blackpill" && t.firmware_target == "stm32f4"));
    // No source tree configured for this engine -> builds refused.
    let e = c.firmware.start_build(pb::StartBuildRequest { mcu: String::new() }).await.unwrap_err();
    assert_eq!(error_code(&e), Some("invalid_state"));
    // Flash requires explicit confirmation, then an existing build.
    let e = c.firmware.start_flash(pb::StartFlashRequest { build_job_id: "build-1".into(), confirm: false }).await.unwrap_err();
    assert_eq!(error_code(&e), Some("bad_request"));
    let e = c.firmware.start_flash(pb::StartFlashRequest { build_job_id: "build-1".into(), confirm: true }).await.unwrap_err();
    assert_eq!(error_code(&e), Some("not_found"));
    // Flashing is refused while drives are energised.
    c.call("enable", json!({})).await.unwrap();
    let e = c.firmware.start_flash(pb::StartFlashRequest { build_job_id: "build-1".into(), confirm: true }).await.unwrap_err();
    assert_eq!(error_code(&e), Some("invalid_state"));
    assert!(c.firmware.list_jobs(pb::ListJobsRequest {}).await.unwrap().into_inner().jobs.is_empty());
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn geometry_is_served_and_api_version_is_current() {
    let mut c = start_with("rail-arm-virtual.toml", false).await;
    let info = c.engine.get_info(pb::GetInfoRequest {}).await.unwrap().into_inner();
    assert_eq!(info.api_version, motioncore_engine_api::API_VERSION);
    let cfg = c.engine.get_robot_config(pb::GetRobotConfigRequest {}).await.unwrap().into_inner();
    let g = |id: &str| cfg.axes.iter().find(|a| a.id == id).unwrap().geometry.clone().unwrap();
    let rail = g("x");
    assert_eq!(rail.parent, "");
    let ax = rail.axis.unwrap();
    assert_eq!((ax.x, ax.y, ax.z), (1.0, 0.0, 0.0));
    let shoulder = g("j1");
    assert_eq!(shoulder.parent, "x");
    assert_eq!(shoulder.origin_mm.unwrap().z, 120.0);
    let elbow = g("j2");
    assert_eq!(elbow.parent, "j1");
    assert_eq!(elbow.origin_mm.unwrap().x, 250.0);
    assert_eq!(elbow.rotation_deg.unwrap(), pb::Vec3 { x: 0.0, y: 0.0, z: 0.0 });
    assert_eq!(elbow.tip_mm.unwrap().x, 200.0);
    assert!(shoulder.tip_mm.is_none());

    // Profiles without geometry leave the field unset.
    let mut plain = start(false).await;
    let cfg = plain.engine.get_robot_config(pb::GetRobotConfigRequest {}).await.unwrap().into_inner();
    assert!(cfg.axes.iter().all(|a| a.geometry.is_none()));
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn home_all_runs_groups_together_and_collisions_are_refused() {
    let mut c = start_with("dummy-arm-virtual.toml", false).await;
    let cfg = c.engine.get_robot_config(pb::GetRobotConfigRequest {}).await.unwrap().into_inner();
    assert!(cfg.collision_checking);
    let plan = cfg.home_all.unwrap();
    assert!(plan.from_profile);
    let groups: Vec<Vec<String>> = plan.groups.iter().map(|g| g.axes.clone()).collect();
    assert_eq!(groups, [vec!["j4", "j5", "j6"], vec!["j1", "j2", "j3"]]);
    assert!(plan.start_pose.contains("straight up"));

    c.engine.enable(pb::EnableRequest {}).await.unwrap();
    c.wait_state("ready", T).await;
    c.engine.home_all(pb::HomeAllRequest {}).await.unwrap();

    let wrist = ["j4", "j5", "j6"];
    let mut max_together = 0;
    let mut arm_while_wrist = false;
    let deadline = std::time::Instant::now() + Duration::from_secs(60);
    let done = loop {
        let t = c.engine.get_status(pb::GetStatusRequest {}).await.unwrap().into_inner();
        let homing: Vec<&str> = t.axes.iter().filter(|a| a.homing).map(|a| a.id.as_str()).collect();
        max_together = max_together.max(homing.len());
        if homing.iter().any(|h| wrist.contains(h)) && homing.iter().any(|h| !wrist.contains(h)) {
            arm_while_wrist = true;
        }
        if t.state == pb::RobotState::Ready as i32 && t.axes.iter().all(|a| a.homed) {
            break t;
        }
        assert!(t.fault.is_none(), "fault while homing: {:?}", t.fault);
        assert!(std::time::Instant::now() < deadline, "home all did not finish");
        tokio::time::sleep(Duration::from_millis(20)).await;
    };
    assert!(max_together >= 2, "groups must home together (max {max_together})");
    assert!(!arm_while_wrist, "the arm group started before the wrist group finished");
    let pos: Vec<(String, f64)> = done.axes.iter().map(|a| (a.id.clone(), (a.commanded_position * 10.0).round() / 10.0)).collect();
    assert_eq!(
        pos,
        [("j1", -170.0), ("j2", -73.0), ("j3", 180.0), ("j4", -180.0), ("j5", 120.0), ("j6", -720.0)].map(|(a, b)| (a.to_string(), b))
    );

    // Pitching the wrist from +120 to -120 here folds it into the upper arm.
    let err = c.engine.move_to(pb::MoveToRequest { axis: "j5".into(), position: -120.0, velocity: None }).await.unwrap_err();
    assert_eq!(err.metadata().get("motioncore-error-code").unwrap(), "collision");
    assert!(err.message().contains("would collide"), "{}", err.message());
    // Rolling the flange cannot collide.
    c.engine.move_to(pb::MoveToRequest { axis: "j6".into(), position: 0.0, velocity: None }).await.unwrap();
}

/// Joint values (measured, commanded) by axis id.
fn joints(t: &pb::Telemetry) -> Vec<(f64, f64)> {
    t.axes.iter().map(|a| (a.measured_position, a.commanded_position)).collect()
}

fn targets(q: &[f64]) -> Vec<pb::JointTarget> {
    ["j1", "j2", "j3", "j4", "j5", "j6"].iter().zip(q).map(|(a, &p)| pb::JointTarget { axis: a.to_string(), position: p }).collect()
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn move_j_moves_six_axes_together_on_a_straight_joint_path() {
    let mut c = start_with("dummy-arm-virtual.toml", false).await;
    let info = c.engine.get_info(pb::GetInfoRequest {}).await.unwrap().into_inner();
    assert_eq!(info.api_version, motioncore_engine_api::API_VERSION);
    c.engine.enable(pb::EnableRequest {}).await.unwrap();
    c.wait_state("ready", T).await;

    // Positions are unknown before homing.
    let goal = [-30.0, 20.0, 120.0, -60.0, 40.0, -540.0];
    let e = c.engine.move_j(pb::MoveJRequest { targets: targets(&goal), speed_scale: None }).await.unwrap_err();
    assert_eq!(e.metadata().get("motioncore-error-code").unwrap(), "not_homed");

    c.engine.home_all(pb::HomeAllRequest {}).await.unwrap();
    c.wait_for("homed", Duration::from_secs(60), |s| s["state"] == "ready" && s["axes"].as_array().unwrap().iter().all(|a| a["homed"] == true))
        .await;
    let t0 = c.engine.get_status(pb::GetStatusRequest {}).await.unwrap().into_inner();
    let start: Vec<f64> = joints(&t0).iter().map(|j| j.1).collect();

    // Rejected requests leave the robot where it is.
    let bad = |e: tonic::Status| e.metadata().get("motioncore-error-code").unwrap().to_str().unwrap().to_string();
    let mut dup = targets(&goal);
    dup.push(pb::JointTarget { axis: "j1".into(), position: 0.0 });
    assert_eq!(bad(c.engine.move_j(pb::MoveJRequest { targets: dup, speed_scale: None }).await.unwrap_err()), "bad_request");
    assert_eq!(bad(c.engine.move_j(pb::MoveJRequest { targets: vec![], speed_scale: None }).await.unwrap_err()), "bad_request");
    let mut over = goal;
    over[1] = 95.0;
    let e = c.engine.move_j(pb::MoveJRequest { targets: targets(&over), speed_scale: None }).await.unwrap_err();
    assert_eq!(bad(e.clone()), "validation_failed");
    assert!(e.message().contains("j2"), "{}", e.message());
    for s in [0.0, 1.5] {
        assert_eq!(bad(c.engine.move_j(pb::MoveJRequest { targets: targets(&goal), speed_scale: Some(s) }).await.unwrap_err()), "validation_failed");
    }

    // The coordinated move: sample measured positions from MCU telemetry.
    c.engine.move_j(pb::MoveJRequest { targets: targets(&goal), speed_scale: None }).await.unwrap();
    let mut samples: Vec<(Instant, Vec<f64>)> = Vec::new();
    let deadline = Instant::now() + Duration::from_secs(20);
    loop {
        let t = c.engine.get_status(pb::GetStatusRequest {}).await.unwrap().into_inner();
        assert!(t.fault.is_none(), "fault during move_j: {:?}", t.fault);
        samples.push((Instant::now(), joints(&t).iter().map(|j| j.0).collect()));
        if t.state == pb::RobotState::Ready as i32 && samples.len() > 3 {
            break;
        }
        assert!(Instant::now() < deadline, "move_j did not finish");
        tokio::time::sleep(Duration::from_millis(10)).await;
    }
    let frac = |q: &[f64], i: usize| (q[i] - start[i]) / (goal[i] - start[i]);
    let spu = [444.444444, 266.666667, 266.666667, 213.333333, 266.666667, 444.444444];
    let mut mid = 0;
    for (_, q) in &samples {
        let f: Vec<f64> = (0..6).map(|i| frac(q, i)).collect();
        // Every axis is at the same fraction of its own distance: they start,
        // move and stop together (one step of slack per axis).
        for i in 0..6 {
            let tol = 1.5 / spu[i] / (goal[i] - start[i]).abs() + 0.004;
            assert!((f[i] - f[0]).abs() <= tol, "axis j{} at {:.4} while j1 at {:.4}: {q:?}", i + 1, f[i], f[0]);
        }
        if f[0] > 0.1 && f[0] < 0.9 {
            mid += 1;
        }
    }
    assert!(mid >= 10, "too few samples in mid-move ({mid})");
    let end = samples.last().unwrap();
    for (i, (q, g)) in end.1.iter().zip(goal).enumerate() {
        assert!((q - g).abs() < 0.01, "j{} ended at {q}", i + 1);
    }
    let first_move = samples.iter().find(|(_, q)| frac(q, 0) > 0.0).unwrap().0;
    let arrived = samples.iter().find(|(_, q)| (0..6).all(|i| (q[i] - goal[i]).abs() < 0.01)).unwrap().0;
    let took = (arrived - first_move).as_secs_f64();
    // Lead axis j1: 140 deg at 60 deg/s and 150 deg/s^2 is 2.73 s.
    assert!((2.4..3.3).contains(&took), "move took {took:.2} s");

    // A path whose ends are clear but which swings the flange through the
    // base column is refused before anything moves.
    let a = [-120.0, 20.0, 180.0, 130.0, 80.0, -10.0];
    let b = [150.0, -35.0, 150.0, -145.0, 60.0, 465.0];
    c.engine.move_j(pb::MoveJRequest { targets: targets(&a), speed_scale: None }).await.unwrap();
    c.wait_state("running", T).await;
    c.wait_state("ready", Duration::from_secs(20)).await;
    let e = c.engine.move_j(pb::MoveJRequest { targets: targets(&b), speed_scale: Some(0.5) }).await.unwrap_err();
    assert_eq!(bad(e.clone()), "collision");
    assert!(e.message().contains("j1 and j6") && e.message().contains(" at j1 "), "{}", e.message());
    let still = c.engine.get_status(pb::GetStatusRequest {}).await.unwrap().into_inner();
    assert!(joints(&still).iter().zip(a).all(|(j, q)| (j.0 - q).abs() < 0.01));

    // Listing only some axes moves those and holds the rest.
    c.engine
        .move_j(pb::MoveJRequest {
            targets: vec![pb::JointTarget { axis: "j1".into(), position: -100.0 }, pb::JointTarget { axis: "j3".into(), position: 160.0 }],
            speed_scale: Some(0.5),
        })
        .await
        .unwrap();
    c.wait_state("running", T).await;
    let done = c.wait_state("ready", Duration::from_secs(20)).await;
    let q: Vec<f64> = done["axes"].as_array().unwrap().iter().map(|x| x["measured_position"].as_f64().unwrap()).collect();
    let want = [-100.0, 20.0, 160.0, 130.0, 80.0, -10.0];
    assert!(q.iter().zip(want).all(|(x, y)| (x - y).abs() < 0.01), "{q:?}");
}
