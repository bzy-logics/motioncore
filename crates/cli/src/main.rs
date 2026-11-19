//! `motioncore` — headless runtime and tooling. Works without the Go server
//! or the web UI.
//!
//! ```text
//! motioncore start --config robot-profiles/single-axis-virtual.toml
//! motioncore status | config | plugins | info | watch
//! motioncore enable | disable | stop | estop | reset-fault
//! motioncore jog j1 5      motioncore move j1 -90      motioncore home j1
//! motioncore mcu list
//! motioncore simulator --listen 127.0.0.1:7700 --config ROBOT.toml
//! motioncore firmware targets | configure | build | flash | verify
//! motioncore engine-api proto
//! ```

mod client;

use std::path::{Path, PathBuf};
use std::sync::Arc;

use anyhow::{bail, Context, Result};
use clap::{Parser, Subcommand};
use motioncore_core::{engine, firmware::FirmwareJobs, grpc};
use motioncore_device_graph::FlashMethod;
use motioncore_engine_api::v1 as pb;
use motioncore_firmware_builder as builder;
use motioncore_plugin_runtime::{PluginContext, Registry, CORE_CAPABILITIES};

#[derive(Parser)]
#[command(name = "motioncore", version, about = "MotionCore robotics motion-control runtime and tools")]
struct Cli {
    /// Engine gRPC address for client commands (unix:///path or tcp://host:port).
    #[arg(long, global = true, env = "MOTIONCORE_ENGINE_ADDR")]
    engine: Option<String>,
    #[command(subcommand)]
    cmd: Cmd,
}

#[derive(Subcommand)]
enum Cmd {
    /// Run the engine (state authority, motion pipeline, MCU link, gRPC API).
    Start {
        #[arg(long, default_value = "robot-profiles/single-axis-virtual.toml")]
        config: PathBuf,
        /// Listen address; defaults to the --engine address or the platform default.
        #[arg(long)]
        listen: Option<String>,
        /// Directory with external plugin manifests.
        #[arg(long)]
        plugins: Option<PathBuf>,
        /// motioncore source tree used for firmware builds (default: auto-detect).
        #[arg(long)]
        repo: Option<PathBuf>,
        /// Enable debug fault-injection RPCs. Never with real hardware.
        #[arg(long)]
        enable_fault_injection: bool,
    },
    /// Engine and API versions.
    Info,
    /// Current telemetry snapshot.
    Status,
    /// Loaded robot configuration.
    Config,
    /// Plugin states.
    Plugins,
    /// Stream telemetry and log events.
    Watch,
    Enable,
    Disable,
    Stop,
    /// Software emergency stop (complements, never replaces, a hardware E-stop).
    Estop,
    ResetFault,
    /// Relative move in axis units.
    Jog {
        axis: String,
        #[arg(allow_negative_numbers = true)]
        delta: f64,
        #[arg(long)]
        velocity: Option<f64>,
    },
    /// Absolute move in axis units (axis must be homed).
    Move {
        axis: String,
        #[arg(allow_negative_numbers = true)]
        position: f64,
        #[arg(long)]
        velocity: Option<f64>,
    },
    /// Coordinated joint move: listed axes start and finish together
    /// (all must be homed), e.g. `movej j1=30 j2=-10 --speed 0.5`.
    Movej {
        /// AXIS=POSITION pairs, axis units.
        #[arg(required = true, value_parser = parse_joint_target, allow_negative_numbers = true)]
        targets: Vec<(String, f64)>,
        /// Fraction of every axis' maximum velocity, 0 < SPEED <= 1.
        #[arg(long)]
        speed: Option<f64>,
    },
    /// Blended joint path through waypoints without stopping at the
    /// intermediate ones, e.g. `movej-path "j1=30 j2=10" "j1=-30" "j1=0 j2=0"`.
    MovejPath {
        /// One argument per waypoint: space-separated AXIS=POSITION pairs.
        #[arg(required = true)]
        waypoints: Vec<String>,
        /// Fraction of every axis' maximum velocity, 0 < SPEED <= 1.
        #[arg(long)]
        speed: Option<f64>,
    },
    /// Cartesian straight-line move of the tool point to X Y Z (mm).
    Movel {
        #[arg(allow_negative_numbers = true)]
        x: f64,
        #[arg(allow_negative_numbers = true)]
        y: f64,
        #[arg(allow_negative_numbers = true)]
        z: f64,
        /// Target orientation as ROLL,PITCH,YAW in degrees (default: keep).
        #[arg(long, value_delimiter = ',', num_args = 3, allow_negative_numbers = true)]
        rpy: Option<Vec<f64>>,
        /// Tool speed along the line, mm/s (default 50).
        #[arg(long)]
        speed: Option<f64>,
        /// Fraction of every axis' maximum velocity, 0 < SCALE <= 1.
        #[arg(long)]
        speed_scale: Option<f64>,
    },
    Home {
        axis: String,
    },
    /// Home every axis, following the profile's home-all plan.
    HomeAll,
    /// MCU discovery.
    Mcu {
        #[command(subcommand)]
        cmd: McuCmd,
    },
    /// Run a virtual MCU on TCP (each connection is a fresh boot).
    Simulator {
        #[arg(long, default_value = "127.0.0.1:7700")]
        listen: String,
        #[arg(long)]
        config: Option<PathBuf>,
        #[arg(long, default_value = "main")]
        mcu: String,
    },
    /// Firmware configure/build/flash/verify (local, restricted builder).
    Firmware {
        #[command(subcommand)]
        cmd: FirmwareCmd,
    },
    /// Engine API contract.
    EngineApi {
        #[command(subcommand)]
        cmd: EngineApiCmd,
    },
}

#[derive(Subcommand)]
enum McuCmd {
    /// List serial ports.
    List,
}

#[derive(Subcommand)]
enum EngineApiCmd {
    /// Print the contract (.proto) this binary was built with.
    Proto,
}

#[derive(Subcommand)]
enum FirmwareCmd {
    /// Supported boards.
    Targets,
    /// Resolve the configuration and write the lockfile.
    Configure {
        #[arg(long)]
        config: PathBuf,
        #[arg(long)]
        mcu: Option<String>,
    },
    /// Configure and compile, from a robot configuration or a board's
    /// reference profile (`--board`, artifacts in build/firmware/<board>).
    Build {
        #[arg(long, required_unless_present = "board", conflicts_with = "board")]
        config: Option<PathBuf>,
        #[arg(long)]
        board: Option<String>,
        #[arg(long, requires = "config")]
        mcu: Option<String>,
        /// Bench test build with this feature (`bench-panic` or
        /// `bench-isr-probe`); artifacts in a separate `<name>-<feature>`
        /// directory. Never for production.
        #[arg(long)]
        bench: Option<String>,
    },
    /// Flash a built artifact with the board's method: probe-rs (STM32F411)
    /// or UF2 to the BOOTSEL drive (RP2040). Drives must be off.
    Flash {
        #[arg(long, required_unless_present = "board", conflicts_with = "board")]
        artifact: Option<PathBuf>,
        /// Use the artifact of `firmware build --board <id>`.
        #[arg(long)]
        board: Option<String>,
        /// With `--board`: the artifact of `firmware build --board <id> --bench <feature>`.
        #[arg(long, requires = "board")]
        bench: Option<String>,
        /// Mount point of the UF2 bootloader drive (default: auto-detect).
        #[arg(long)]
        uf2_volume: Option<PathBuf>,
        /// Confirm that drives are powered off.
        #[arg(long)]
        yes: bool,
    },
    /// Check the running firmware against an artifact.
    Verify {
        #[arg(long, required_unless_present = "board", conflicts_with = "board")]
        artifact: Option<PathBuf>,
        #[arg(long)]
        board: Option<String>,
        /// With `--board`: the artifact of `firmware build --board <id> --bench <feature>`.
        #[arg(long, requires = "board")]
        bench: Option<String>,
        #[arg(long)]
        device: String,
    },
    /// Bench test: raise the step rate stage by stage until the firmware
    /// faults, talking to the MCU directly (the engine caps moves at the
    /// board's advertised rate). Drivers must have NO motor power. Stop the
    /// engine first.
    RateTest {
        #[arg(long)]
        device: String,
        /// Channels to drive together, e.g. `0,1,2,3,4,5` (default: all).
        #[arg(long, value_delimiter = ',')]
        axes: Option<Vec<u8>>,
        /// First stage rate (steps/s).
        #[arg(long, default_value_t = 10_000.0)]
        from: f32,
        /// Last stage rate (steps/s).
        #[arg(long, default_value_t = 250_000.0)]
        to: f32,
        /// Increment between stages (steps/s).
        #[arg(long, default_value_t = 10_000.0)]
        step: f32,
        /// Time at each stage rate (ms).
        #[arg(long, default_value_t = 1000)]
        hold_ms: u64,
        /// Confirm that the drivers have no motor power (or no motors).
        #[arg(long)]
        no_motor_power: bool,
    },
}

fn main() -> Result<()> {
    let cli = Cli::parse();
    let rt = tokio::runtime::Runtime::new()?;
    rt.block_on(run(cli))
}

fn engine_addr(cli_addr: &Option<String>) -> String {
    cli_addr.clone().unwrap_or_else(|| grpc::ListenAddr::default_for_platform().to_string())
}

async fn run(cli: Cli) -> Result<()> {
    let addr = engine_addr(&cli.engine);
    match cli.cmd {
        Cmd::Start { config, listen, plugins, repo, enable_fault_injection } => {
            start(&config, &listen.unwrap_or(addr), plugins.as_deref(), repo, enable_fault_injection).await
        }
        Cmd::Info => {
            let i = client::connect(&addr).await?.engine.get_info(pb::GetInfoRequest {}).await?.into_inner();
            println!(
                "engine {}  api {}  mcu protocol {}  plugin api {}  fault injection {}",
                i.engine_version, i.api_version, i.mcu_protocol_version, i.plugin_api_version, i.fault_injection_enabled
            );
            Ok(())
        }
        Cmd::Status => {
            let t = client::connect(&addr).await?.engine.get_status(pb::GetStatusRequest {}).await?.into_inner();
            client::print_telemetry(&t);
            Ok(())
        }
        Cmd::Config => {
            let c = client::connect(&addr).await?.engine.get_robot_config(pb::GetRobotConfigRequest {}).await?.into_inner();
            client::print_config(&c);
            Ok(())
        }
        Cmd::Plugins => {
            let p = client::connect(&addr).await?.engine.list_plugins(pb::ListPluginsRequest {}).await?.into_inner();
            for p in p.plugins {
                let state = pb::PluginState::try_from(p.state).map(|s| s.as_str_name()).unwrap_or("?");
                println!(
                    "{:<45} {:<8} {:<8} {}{}",
                    p.id,
                    p.version,
                    p.kind,
                    state.trim_start_matches("PLUGIN_STATE_").to_lowercase(),
                    if p.error.is_empty() { String::new() } else { format!("  ({})", p.error) }
                );
            }
            Ok(())
        }
        Cmd::Watch => client::watch(&addr).await,
        Cmd::Enable => client::command(&addr, |mut c| async move { c.engine.enable(pb::EnableRequest {}).await }).await,
        Cmd::Disable => client::command(&addr, |mut c| async move { c.engine.disable(pb::DisableRequest {}).await }).await,
        Cmd::Stop => client::command(&addr, |mut c| async move { c.engine.stop(pb::StopRequest {}).await }).await,
        Cmd::Estop => client::command(&addr, |mut c| async move { c.engine.emergency_stop(pb::EmergencyStopRequest {}).await }).await,
        Cmd::ResetFault => client::command(&addr, |mut c| async move { c.engine.reset_fault(pb::ResetFaultRequest {}).await }).await,
        Cmd::Jog { axis, delta, velocity } => client::command(&addr, |mut c| async move { c.engine.jog(pb::JogRequest { axis, delta, velocity }).await }).await,
        Cmd::Move { axis, position, velocity } => {
            client::command(&addr, |mut c| async move { c.engine.move_to(pb::MoveToRequest { axis, position, velocity }).await }).await
        }
        Cmd::Home { axis } => client::command(&addr, |mut c| async move { c.engine.home(pb::HomeRequest { axis }).await }).await,
        Cmd::Movej { targets, speed } => {
            let targets = targets.into_iter().map(|(axis, position)| pb::JointTarget { axis, position }).collect();
            client::command(&addr, |mut c| async move { c.engine.move_j(pb::MoveJRequest { targets, speed_scale: speed }).await }).await
        }
        Cmd::MovejPath { waypoints, speed } => {
            let mut wps = Vec::with_capacity(waypoints.len());
            for w in &waypoints {
                let targets: Vec<pb::JointTarget> = w
                    .split_whitespace()
                    .map(|t| parse_joint_target(t).map(|(axis, position)| pb::JointTarget { axis, position }))
                    .collect::<Result<_, _>>()
                    .map_err(|e| anyhow::anyhow!(e))?;
                wps.push(pb::JointWaypoint { targets });
            }
            client::command(&addr, |mut c| async move { c.engine.move_j_path(pb::MoveJPathRequest { waypoints: wps, speed_scale: speed }).await }).await
        }
        Cmd::Movel { x, y, z, rpy, speed, speed_scale } => {
            let rotation_deg = rpy.map(|r| pb::Vec3 { x: r[0], y: r[1], z: r[2] });
            let req = pb::MoveLRequest { position_mm: Some(pb::Vec3 { x, y, z }), rotation_deg, speed_mm_s: speed, speed_scale };
            client::command(&addr, |mut c| async move { c.engine.move_l(req).await }).await
        }
        Cmd::HomeAll => client::command(&addr, |mut c| async move { c.engine.home_all(pb::HomeAllRequest {}).await }).await,
        Cmd::Mcu { cmd: McuCmd::List } => {
            let ports = builder::list_ports()?;
            if ports.is_empty() {
                println!("no serial ports found");
            }
            for p in ports {
                println!("{:<20} {}", p.name, p.description);
            }
            Ok(())
        }
        Cmd::Simulator { listen, config, mcu } => {
            let opts = match config {
                Some(path) => {
                    // Only geometry matters here; force a virtual MCU.
                    let mut file: motioncore_device_graph::RobotFile = toml::from_str(&std::fs::read_to_string(&path)?)?;
                    for m in file.mcu.values_mut() {
                        m.transport = "virtual".into();
                        m.board = "virtual".into();
                    }
                    let robot = motioncore_device_graph::resolve(file).map_err(|e| anyhow::anyhow!("invalid config: {e:?}"))?;
                    motioncore_simulator::VirtualOptions::from_robot(&robot, &mcu)
                }
                None => Default::default(),
            };
            motioncore_simulator::serve_tcp(&listen, opts).await?;
            Ok(())
        }
        Cmd::Firmware { cmd } => firmware(cmd, &addr).await,
        Cmd::EngineApi { cmd: EngineApiCmd::Proto } => {
            print!("{}", motioncore_engine_api::PROTO_V1);
            Ok(())
        }
    }
}

async fn start(config: &Path, listen: &str, plugin_dir: Option<&Path>, repo: Option<PathBuf>, fault_injection: bool) -> Result<()> {
    tracing_subscriber::fmt().with_env_filter(tracing_subscriber::EnvFilter::try_from_default_env().unwrap_or_else(|_| "info".into())).init();
    let robot = motioncore_device_graph::load(config).with_context(|| format!("loading {}", config.display()))?;
    tracing::info!("robot '{}' loaded from {}", robot.file.robot.name, config.display());

    let mut registry = Registry::new(CORE_CAPABILITIES.iter().copied());
    for p in motioncore_builtin_plugins::all() {
        registry.add_builtin(p);
    }
    if let Some(dir) = plugin_dir {
        for w in registry.discover_dir(dir) {
            tracing::warn!("plugin manifest ignored: {w}");
        }
    }
    let ctx = PluginContext { robot_name: robot.file.robot.name.clone(), axis_ids: robot.file.axes.iter().map(|a| a.id.clone()).collect() };
    for id in registry.start(&ctx) {
        let err = registry.info().into_iter().find(|i| i.id == id).and_then(|i| i.error).unwrap_or_default();
        tracing::warn!("plugin {id} failed: {err}");
    }

    let repo_root = match repo {
        Some(r) => Some(builder::find_repo_root(&r)?),
        None => std::env::current_dir().ok().and_then(|d| builder::find_repo_root(&d).ok()),
    };
    let config_abs = std::fs::canonicalize(config)?;
    let engine = engine::Engine::new(robot, registry, engine::Options { fault_injection, ..Default::default() })?;
    engine.spawn();
    let jobs = FirmwareJobs::new(repo_root.clone(), config_abs);

    let addr = grpc::ListenAddr::parse(listen)?;
    engine.log("info", format!("engine API v{} listening on {listen}", motioncore_engine_api::API_VERSION));
    match &repo_root {
        Some(r) => engine.log("info", format!("firmware builds enabled (source tree {})", r.display())),
        None => engine.log("warn", "firmware builds disabled: motioncore source tree not found (use --repo)"),
    }
    if fault_injection {
        engine.log("warn", "fault injection enabled — not for use with real hardware");
    }
    let e2 = Arc::clone(&engine);
    let shutdown = async move {
        let _ = tokio::signal::ctrl_c().await;
        e2.log("info", "shutdown requested");
        // Leave hardware safe before exiting.
        let _ = e2.estop().await;
        tokio::time::sleep(std::time::Duration::from_millis(100)).await;
    };
    grpc::serve(engine, jobs, addr, shutdown).await
}

/// `--artifact <dir>` or the directory of a `--board` build (with `--bench`,
/// of that bench build).
fn artifact_dir(artifact: Option<PathBuf>, board: Option<String>, bench: Option<String>) -> Result<PathBuf> {
    match (artifact, board) {
        (Some(a), _) => Ok(a),
        (None, Some(b)) => {
            builder::board(&b)?;
            Ok(builder::board_artifact_dir(&builder::find_repo_root(&std::env::current_dir()?)?, &b, bench.as_deref()))
        }
        (None, None) => bail!("--artifact or --board is required"),
    }
}

async fn firmware(cmd: FirmwareCmd, engine_addr: &str) -> Result<()> {
    let root = || builder::find_repo_root(&std::env::current_dir()?);
    match cmd {
        FirmwareCmd::Targets => {
            println!("{:<22} {:<8} {:<24} {:>8} {:>11} {:<9} description", "board", "target", "rust target", "channels", "max steps/s", "flash");
            for b in builder::targets() {
                let flash = match b.flash {
                    FlashMethod::None => "-",
                    FlashMethod::ProbeRs { .. } => "probe-rs",
                    FlashMethod::Uf2 { .. } => "uf2",
                };
                println!(
                    "{:<22} {:<8} {:<24} {:>8} {:>11} {:<9} {}{}",
                    b.id,
                    b.firmware_target.unwrap_or("-"),
                    b.rust_target.unwrap_or("-"),
                    b.step_channels,
                    b.max_step_rate,
                    flash,
                    b.description,
                    if b.provisional { " [provisional]" } else { "" }
                );
            }
            Ok(())
        }
        FirmwareCmd::Configure { config, mcu } => {
            let c = builder::configure(&root()?, &config, mcu.as_deref())?;
            println!("lockfile {} (build hash {})", c.dir.join("motioncore.lock.toml").display(), c.build_hash);
            println!("{}", toml::to_string_pretty(&c.lock)?);
            Ok(())
        }
        FirmwareCmd::Build { config, board, mcu, bench } => {
            let root = root()?;
            match (config, board) {
                (Some(config), _) => builder::build_bench(&root, &config, mcu.as_deref(), bench.as_deref(), &mut |l| println!("{l}"))?,
                (None, Some(board)) => builder::build_board(&root, &board, bench.as_deref(), &mut |l| println!("{l}"))?,
                (None, None) => unreachable!("clap requires --config or --board"),
            };
            Ok(())
        }
        FirmwareCmd::Flash { artifact, board, bench, uf2_volume, yes } => {
            let artifact = artifact_dir(artifact, board, bench)?;
            let m = builder::load_artifact(&artifact)?;
            // Refuse while a reachable engine reports energised drives.
            match client::connect(engine_addr).await {
                Ok(mut c) => {
                    let t = c.engine.get_status(pb::GetStatusRequest {}).await?.into_inner();
                    let state = pb::RobotState::try_from(t.state).unwrap_or(pb::RobotState::Unspecified);
                    if matches!(state, pb::RobotState::Ready | pb::RobotState::Homing | pb::RobotState::Running) {
                        bail!("engine reports {} — disable drives before flashing", state.as_str_name());
                    }
                }
                Err(_) => println!("engine not reachable at {engine_addr}; make sure drives are powered off"),
            }
            println!("about to flash {} (build {}, fw {}) for {}", m.image, m.build_hash, m.firmware_version, m.board);
            if !m.bench_features.is_empty() {
                println!("BENCH TEST BUILD ({}): for bench measurements only, never for production", m.bench_features.join(", "));
            }
            if !yes {
                bail!("refusing to flash without operator confirmation: re-run with --yes once drives are powered off");
            }
            let opts = builder::FlashOptions { uf2_volume, uf2_timeout: None };
            builder::flash_with(&artifact, &opts, &mut |l| println!("{l}"))?;
            Ok(())
        }
        FirmwareCmd::Verify { artifact, board, bench, device } => {
            let artifact = artifact_dir(artifact, board, bench)?;
            let r = builder::verify(&artifact, &device).await?;
            println!(
                "MCU reports: target {} board {} fw {} build {} caps [{}]",
                r.target,
                r.board,
                r.firmware_version,
                r.build_hash,
                r.capabilities.join(", ")
            );
            if r.bench_features.is_empty() {
                println!("build type: production");
            } else {
                println!("build type: BENCH TEST BUILD ({}), never for production", r.bench_features.join(", "));
            }
            if r.problems.is_empty() {
                println!("verified: running firmware matches {}", artifact.display());
                Ok(())
            } else {
                for p in &r.problems {
                    println!("MISMATCH: {p}");
                }
                bail!("verification failed")
            }
        }
        FirmwareCmd::RateTest { device, axes, from, to, step, hold_ms, no_motor_power } => {
            if !no_motor_power {
                bail!(
                    "the rate test enables the drivers and steps far above any usable speed: \
                     remove motor power (or disconnect the motors), then re-run with --no-motor-power"
                );
            }
            if client::connect(engine_addr).await.is_ok() {
                bail!("an engine is running at {engine_addr}: stop it first, the rate test talks to the MCU directly");
            }
            let o = builder::bench::RateTestOptions { axes, from, to, step, hold: std::time::Duration::from_millis(hold_ms), ..Default::default() };
            let r = builder::bench::rate_test_serial(&device, &o, &mut |l| println!("{l}")).await?;
            let max = r.max_sustained();
            if r.limit_not_reached() {
                println!("no fault up to {max:.0} steps/s on channels {:?}: the limit is above that; raise --to", r.axes);
            } else {
                println!("max sustained step rate: {max:.0} steps/s on channels {:?}", r.axes);
            }
            if let Ok(b) = builder::board(&r.board) {
                let advertised = b.max_step_rate as f32;
                let verdict = if advertised <= 0.5 * max { "pass" } else { "FAIL (or not measured high enough)" };
                println!("advertised limit {advertised:.0} steps/s must be <= 50 % of the measured rate: {verdict}");
            }
            Ok(())
        }
    }
}

/// Parse `AXIS=POSITION`.
fn parse_joint_target(s: &str) -> Result<(String, f64), String> {
    let (axis, pos) = s.split_once('=').ok_or_else(|| format!("expected AXIS=POSITION, got {s:?}"))?;
    let pos: f64 = pos.trim().parse().map_err(|_| format!("invalid position in {s:?}"))?;
    if axis.trim().is_empty() || !pos.is_finite() {
        return Err(format!("invalid joint target {s:?}"));
    }
    Ok((axis.trim().to_string(), pos))
}
