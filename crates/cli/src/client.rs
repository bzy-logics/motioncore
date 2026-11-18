//! gRPC client helpers for the CLI.

use anyhow::{Context, Result};
use motioncore_core::error::error_code;
use motioncore_engine_api::v1 as pb;
use pb::engine_service_client::EngineServiceClient;
use tonic::transport::{Channel, Endpoint};

pub struct Client {
    pub engine: EngineServiceClient<Channel>,
}

/// Connect to `unix:///path`, `tcp://host:port` or `host:port`.
pub async fn connect(addr: &str) -> Result<Client> {
    let channel = if let Some(path) = addr.strip_prefix("unix://") {
        unix_channel(path).await?
    } else {
        let hp = addr.strip_prefix("tcp://").unwrap_or(addr);
        Endpoint::from_shared(format!("http://{hp}"))?
            .connect_timeout(std::time::Duration::from_secs(2))
            .connect()
            .await
            .with_context(|| format!("connecting to engine at {addr} (is `motioncore start` running?)"))?
    };
    let mut engine = EngineServiceClient::new(channel);
    let info = engine.get_info(pb::GetInfoRequest {}).await?.into_inner();
    if !motioncore_engine_api::is_compatible(&info.api_version) {
        anyhow::bail!("engine speaks API {}, this CLI needs {} (same 0.MINOR)", info.api_version, motioncore_engine_api::API_VERSION);
    }
    Ok(Client { engine })
}

#[cfg(unix)]
async fn unix_channel(path: &str) -> Result<Channel> {
    let path = path.to_string();
    Ok(Endpoint::try_from("http://[::1]:0")?
        .connect_with_connector(tower::service_fn(move |_| {
            let path = path.clone();
            async move { Ok::<_, std::io::Error>(hyper_util::rt::TokioIo::new(tokio::net::UnixStream::connect(path).await?)) }
        }))
        .await
        .context("connecting to engine socket (is `motioncore start` running?)")?)
}

#[cfg(not(unix))]
async fn unix_channel(path: &str) -> Result<Channel> {
    anyhow::bail!("unix sockets are not supported on this platform ({path}); use tcp://127.0.0.1:7600")
}

/// Run a command RPC and report the MotionCore error code on failure.
pub async fn command<F, Fut>(addr: &str, f: F) -> Result<()>
where
    F: FnOnce(Client) -> Fut,
    Fut: std::future::Future<Output = Result<tonic::Response<pb::CommandResponse>, tonic::Status>>,
{
    match f(connect(addr).await?).await {
        Ok(_) => {
            println!("ok");
            Ok(())
        }
        Err(s) => anyhow::bail!("{}: {}", error_code(&s).unwrap_or("error"), s.message()),
    }
}

fn enum_name(full: &str, prefix: &str) -> String {
    full.trim_start_matches(prefix).to_lowercase()
}

pub fn print_telemetry(t: &pb::Telemetry) {
    let state = pb::RobotState::try_from(t.state).map(|s| enum_name(s.as_str_name(), "ROBOT_STATE_")).unwrap_or_default();
    println!("state: {state}");
    if let Some(f) = &t.fault {
        let code = pb::FaultCode::try_from(f.code).map(|c| enum_name(c.as_str_name(), "FAULT_CODE_")).unwrap_or_default();
        println!("fault: {code} — {}{}", f.message, if f.axis.is_empty() { String::new() } else { format!(" (axis {})", f.axis) });
    }
    if let Some(m) = &t.mcu {
        println!(
            "mcu:   {} via {}  {} {} fw {} build {}  queue {}/{}  rtt {} µs  crc errors {}",
            if m.connected { "connected" } else { "disconnected" },
            m.transport,
            m.target,
            m.board,
            m.firmware_version,
            m.build_hash,
            m.queue_depth,
            m.queue_capacity,
            m.rtt_us,
            m.crc_errors
        );
    }
    for a in &t.axes {
        println!(
            "axis {:<6} pos {:>10.3}  cmd {:>10.3}  vel {:>8.3}  {}{}{}",
            a.id,
            a.measured_position,
            a.commanded_position,
            a.velocity,
            if a.homed { "homed " } else { "unhomed " },
            if a.enabled { "enabled " } else { "disabled " },
            if a.endstop { "ENDSTOP" } else { "" }
        );
    }
    if let Some(tool) = &t.tool {
        let v = |v: &Option<pb::Vec3>| v.as_ref().map_or([0.0; 3], |v| [v.x, v.y, v.z]);
        let (p, r) = (v(&tool.position_mm), v(&tool.rotation_deg));
        println!("tool   x {:.2} y {:.2} z {:.2} mm   roll {:.2} pitch {:.2} yaw {:.2} deg", p[0], p[1], p[2], r[0], r[1], r[2]);
    }
}

pub fn print_config(c: &pb::RobotConfig) {
    println!("robot {} (profile {})", c.name, c.profile);
    for m in &c.mcus {
        println!("mcu {:<8} board {:<22} transport {:<24} watchdog {} ms", m.name, m.board, m.transport, m.watchdog_ms);
    }
    for a in &c.axes {
        println!(
            "axis {:<6} {:<12} {} [{} .. {}] vmax {} amax {} spu {} -> {}#{} ({}, step {} dir {} en {}{})",
            a.id,
            a.name,
            a.unit,
            a.min,
            a.max,
            a.max_velocity,
            a.max_acceleration,
            a.steps_per_unit,
            a.mcu,
            a.mcu_channel,
            a.actuator,
            a.step_pin,
            a.dir_pin,
            a.enable_pin,
            if a.endstop_pin.is_empty() { String::new() } else { format!(" endstop {}", a.endstop_pin) }
        );
    }
}

pub async fn watch(addr: &str) -> Result<()> {
    let mut c = connect(addr).await?;
    let mut stream = c.engine.stream_events(pb::StreamEventsRequest {}).await?.into_inner();
    let mut last_state = -1;
    while let Some(ev) = stream.message().await? {
        match ev.event {
            Some(pb::event::Event::Telemetry(t)) => {
                if t.state != last_state {
                    last_state = t.state;
                    print_telemetry(&t);
                }
            }
            Some(pb::event::Event::Log(l)) => {
                let level = pb::LogLevel::try_from(l.level).map(|x| enum_name(x.as_str_name(), "LOG_LEVEL_")).unwrap_or_default();
                println!("[{:>9.3}] {:<5} {}", l.time_ms as f64 / 1000.0, level, l.message);
            }
            None => {}
        }
    }
    Ok(())
}
