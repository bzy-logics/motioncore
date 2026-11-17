//! gRPC implementation of the engine contract (`motioncore.engine.v1`).
#![allow(clippy::result_large_err)] // tonic::Status is large by design

use std::pin::Pin;
use std::sync::Arc;

use motioncore_engine_api::v1 as pb;
use motioncore_engine_api::v1::engine_service_server::{EngineService, EngineServiceServer};
use motioncore_engine_api::v1::firmware_service_server::{FirmwareService, FirmwareServiceServer};
use tokio::sync::broadcast::error::RecvError;
use tokio_stream::Stream;
use tonic::{Request, Response, Status};

use crate::engine::Engine;
use crate::error::ApiError;
use crate::firmware::FirmwareJobs;

type Res<T> = Result<Response<T>, Status>;

fn done(r: Result<(), ApiError>) -> Res<pb::CommandResponse> {
    r.map(|_| Response::new(pb::CommandResponse {})).map_err(Status::from)
}

#[derive(Clone)]
pub struct EngineSvc {
    engine: Arc<Engine>,
}

#[tonic::async_trait]
impl EngineService for EngineSvc {
    async fn get_info(&self, _: Request<pb::GetInfoRequest>) -> Res<pb::EngineInfo> {
        Ok(Response::new(pb::EngineInfo {
            api_version: motioncore_engine_api::API_VERSION.into(),
            engine_version: env!("CARGO_PKG_VERSION").into(),
            mcu_protocol_version: motioncore_protocol::PROTOCOL_VERSION as u32,
            plugin_api_version: motioncore_plugin_runtime::API_VERSION.into(),
            fault_injection_enabled: self.engine.fault_injection_enabled(),
        }))
    }

    async fn get_status(&self, _: Request<pb::GetStatusRequest>) -> Res<pb::Telemetry> {
        Ok(Response::new(self.engine.telemetry_snapshot()))
    }

    async fn get_robot_config(&self, _: Request<pb::GetRobotConfigRequest>) -> Res<pb::RobotConfig> {
        Ok(Response::new(self.engine.robot_config()))
    }

    async fn list_plugins(&self, _: Request<pb::ListPluginsRequest>) -> Res<pb::ListPluginsResponse> {
        Ok(Response::new(pb::ListPluginsResponse { plugins: self.engine.plugins() }))
    }

    type StreamEventsStream = Pin<Box<dyn Stream<Item = Result<pb::Event, Status>> + Send>>;

    async fn stream_events(&self, _: Request<pb::StreamEventsRequest>) -> Res<Self::StreamEventsStream> {
        let mut telemetry = self.engine.subscribe_telemetry();
        let mut logs = self.engine.subscribe_logs();
        let first = self.engine.telemetry_snapshot();
        let stream = async_stream(move |tx| async move {
            use pb::event::Event as E;
            if tx.send(Ok(pb::Event { event: Some(E::Telemetry(first)) })).await.is_err() {
                return;
            }
            loop {
                let ev = tokio::select! {
                    t = telemetry.recv() => match t {
                        Ok(t) => E::Telemetry((*t).clone()),
                        Err(RecvError::Lagged(_)) => continue,
                        Err(RecvError::Closed) => return,
                    },
                    l = logs.recv() => match l {
                        Ok(l) => E::Log(l),
                        Err(RecvError::Lagged(n)) => E::Log(pb::LogEntry {
                            time_ms: 0,
                            level: pb::LogLevel::Warn as i32,
                            message: format!("{n} log entries dropped (slow client)"),
                        }),
                        Err(RecvError::Closed) => return,
                    },
                };
                if tx.send(Ok(pb::Event { event: Some(ev) })).await.is_err() {
                    return; // client went away
                }
            }
        });
        Ok(Response::new(Box::pin(stream)))
    }

    async fn enable(&self, _: Request<pb::EnableRequest>) -> Res<pb::CommandResponse> {
        done(self.engine.enable().await)
    }
    async fn disable(&self, _: Request<pb::DisableRequest>) -> Res<pb::CommandResponse> {
        done(self.engine.disable().await)
    }
    async fn jog(&self, r: Request<pb::JogRequest>) -> Res<pb::CommandResponse> {
        let r = r.into_inner();
        done(self.engine.jog(&r.axis, r.delta, r.velocity).await)
    }
    async fn move_to(&self, r: Request<pb::MoveToRequest>) -> Res<pb::CommandResponse> {
        let r = r.into_inner();
        done(self.engine.move_to(&r.axis, r.position, r.velocity).await)
    }
    async fn home(&self, r: Request<pb::HomeRequest>) -> Res<pb::CommandResponse> {
        done(self.engine.home(&r.into_inner().axis).await)
    }
    async fn home_all(&self, _: Request<pb::HomeAllRequest>) -> Res<pb::CommandResponse> {
        done(self.engine.home_all().await)
    }
    async fn stop(&self, _: Request<pb::StopRequest>) -> Res<pb::CommandResponse> {
        done(self.engine.stop().await)
    }
    async fn emergency_stop(&self, _: Request<pb::EmergencyStopRequest>) -> Res<pb::CommandResponse> {
        done(self.engine.estop().await)
    }
    async fn reset_fault(&self, _: Request<pb::ResetFaultRequest>) -> Res<pb::CommandResponse> {
        done(self.engine.reset_fault().await)
    }
    async fn debug_disconnect_mcu(&self, _: Request<pb::DebugDisconnectMcuRequest>) -> Res<pb::CommandResponse> {
        done(self.engine.debug_disconnect_mcu())
    }
    async fn debug_fail_plugin(&self, r: Request<pb::DebugFailPluginRequest>) -> Res<pb::CommandResponse> {
        done(self.engine.debug_fail_plugin(&r.into_inner().id))
    }
}

#[derive(Clone)]
pub struct FirmwareSvc {
    engine: Arc<Engine>,
    jobs: Arc<FirmwareJobs>,
}

#[tonic::async_trait]
impl FirmwareService for FirmwareSvc {
    async fn list_targets(&self, _: Request<pb::ListTargetsRequest>) -> Res<pb::ListTargetsResponse> {
        Ok(Response::new(pb::ListTargetsResponse { targets: self.jobs.targets() }))
    }
    async fn start_build(&self, r: Request<pb::StartBuildRequest>) -> Res<pb::FirmwareJob> {
        Ok(Response::new(self.jobs.start_build(&self.engine, &r.into_inner().mcu)?))
    }
    async fn start_flash(&self, r: Request<pb::StartFlashRequest>) -> Res<pb::FirmwareJob> {
        let r = r.into_inner();
        Ok(Response::new(self.jobs.start_flash(&self.engine, &r.build_job_id, r.confirm)?))
    }
    async fn get_job(&self, r: Request<pb::GetJobRequest>) -> Res<pb::FirmwareJob> {
        Ok(Response::new(self.jobs.get(&r.into_inner().id)?))
    }
    async fn list_jobs(&self, _: Request<pb::ListJobsRequest>) -> Res<pb::ListJobsResponse> {
        Ok(Response::new(pb::ListJobsResponse { jobs: self.jobs.list() }))
    }
}

/// Build a stream from a producer task writing into a bounded channel. A slow
/// consumer back-pressures only its own task, never the engine.
fn async_stream<F, Fut>(f: F) -> impl Stream<Item = Result<pb::Event, Status>>
where
    F: FnOnce(tokio::sync::mpsc::Sender<Result<pb::Event, Status>>) -> Fut,
    Fut: std::future::Future<Output = ()> + Send + 'static,
{
    let (tx, rx) = tokio::sync::mpsc::channel(64);
    tokio::spawn(f(tx));
    tokio_stream::wrappers::ReceiverStream::new(rx)
}

/// Where the engine listens.
#[derive(Debug, Clone, PartialEq)]
pub enum ListenAddr {
    Tcp(std::net::SocketAddr),
    #[cfg(unix)]
    Unix(std::path::PathBuf),
}

impl ListenAddr {
    /// `unix:///run/motioncore/engine.sock`, `tcp://127.0.0.1:7600` or `127.0.0.1:7600`.
    pub fn parse(s: &str) -> anyhow::Result<Self> {
        if let Some(p) = s.strip_prefix("unix://") {
            #[cfg(unix)]
            return Ok(Self::Unix(p.into()));
            #[cfg(not(unix))]
            anyhow::bail!("unix sockets are not supported on this platform ({p}); use tcp://127.0.0.1:7600");
        }
        let addr = s.strip_prefix("tcp://").unwrap_or(s);
        Ok(Self::Tcp(addr.parse().map_err(|e| anyhow::anyhow!("invalid listen address '{s}': {e}"))?))
    }

    pub fn default_for_platform() -> &'static str {
        if cfg!(unix) {
            "unix:///run/motioncore/engine.sock"
        } else {
            "tcp://127.0.0.1:7600"
        }
    }
}

/// Serve both services until `shutdown` resolves.
pub async fn serve(
    engine: Arc<Engine>,
    jobs: Arc<FirmwareJobs>,
    addr: ListenAddr,
    shutdown: impl std::future::Future<Output = ()> + Send + 'static,
) -> anyhow::Result<()> {
    let router = tonic::transport::Server::builder()
        .add_service(EngineServiceServer::new(EngineSvc { engine: engine.clone() }))
        .add_service(FirmwareServiceServer::new(FirmwareSvc { engine, jobs }));
    match addr {
        ListenAddr::Tcp(a) => {
            let listener = tokio::net::TcpListener::bind(a).await?;
            let incoming = tokio_stream::wrappers::TcpListenerStream::new(listener);
            router.serve_with_incoming_shutdown(incoming, shutdown).await?;
        }
        #[cfg(unix)]
        ListenAddr::Unix(path) => {
            if let Some(dir) = path.parent() {
                std::fs::create_dir_all(dir)?;
            }
            let _ = std::fs::remove_file(&path); // stale socket from a previous run
            let listener = tokio::net::UnixListener::bind(&path)?;
            let incoming = tokio_stream::wrappers::UnixListenerStream::new(listener);
            router.serve_with_incoming_shutdown(incoming, shutdown).await?;
        }
    }
    Ok(())
}
