//! Firmware build/flash jobs for `FirmwareService`.
//!
//! Jobs always use the robot configuration the engine was started with —
//! clients choose only *which MCU*. The builder runs in a blocking worker
//! thread; flashing is refused while drives may be energised and requires an
//! explicit operator confirmation flag.

use std::collections::VecDeque;
use std::path::PathBuf;
use std::sync::{Arc, Mutex};

use motioncore_engine_api::v1 as pb;
use motioncore_firmware_builder as builder;

use crate::engine::Engine;
use crate::error::ApiError;

const MAX_JOBS: usize = 50;
const LOG_TAIL: usize = 16 * 1024;

pub struct FirmwareJobs {
    repo_root: Option<PathBuf>,
    config_path: PathBuf,
    jobs: Mutex<VecDeque<Job>>,
    next_id: Mutex<u64>,
}

#[derive(Clone)]
struct Job {
    pb: pb::FirmwareJob,
    artifact_dir: Option<PathBuf>,
}

fn now_unix() -> u64 {
    std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH).map(|d| d.as_secs()).unwrap_or(0)
}

impl FirmwareJobs {
    /// `repo_root` is the motioncore source tree used for builds; `None`
    /// disables building (e.g. a binary-only installation).
    pub fn new(repo_root: Option<PathBuf>, config_path: PathBuf) -> Arc<Self> {
        Arc::new(Self { repo_root, config_path, jobs: Mutex::new(VecDeque::new()), next_id: Mutex::new(1) })
    }

    pub fn targets(&self) -> Vec<pb::FirmwareTarget> {
        builder::targets()
            .iter()
            .map(|b| pb::FirmwareTarget {
                board: b.id.into(),
                description: b.description.into(),
                firmware_target: b.firmware_target.unwrap_or_default().into(),
                rust_target: b.rust_target.unwrap_or_default().into(),
                step_channels: b.step_channels as u32,
                max_step_rate: b.max_step_rate,
                provisional: b.provisional,
                is_virtual: b.is_virtual,
            })
            .collect()
    }

    pub fn list(&self) -> Vec<pb::FirmwareJob> {
        self.jobs.lock().unwrap().iter().rev().map(|j| j.pb.clone()).collect()
    }

    pub fn get(&self, id: &str) -> Result<pb::FirmwareJob, ApiError> {
        self.jobs
            .lock()
            .unwrap()
            .iter()
            .find(|j| j.pb.id == id)
            .map(|j| j.pb.clone())
            .ok_or_else(|| ApiError::not_found(format!("no firmware job '{id}'")))
    }

    fn create(&self, kind: pb::JobKind, mcu: &str) -> pb::FirmwareJob {
        let mut n = self.next_id.lock().unwrap();
        let prefix = if kind == pb::JobKind::Build { "build" } else { "flash" };
        let job = pb::FirmwareJob {
            id: format!("{prefix}-{n}"),
            kind: kind as i32,
            state: pb::JobState::Queued as i32,
            mcu: mcu.into(),
            created_unix: now_unix(),
            ..Default::default()
        };
        *n += 1;
        let mut jobs = self.jobs.lock().unwrap();
        if jobs.len() >= MAX_JOBS {
            jobs.pop_front();
        }
        jobs.push_back(Job { pb: job.clone(), artifact_dir: None });
        job
    }

    fn update(&self, id: &str, f: impl FnOnce(&mut Job)) {
        if let Some(j) = self.jobs.lock().unwrap().iter_mut().find(|j| j.pb.id == id) {
            f(j);
        }
    }

    fn append_log(&self, id: &str, line: &str) {
        self.update(id, |j| {
            j.pb.log.push_str(line);
            j.pb.log.push('\n');
            if j.pb.log.len() > LOG_TAIL {
                let cut = j.pb.log.len() - LOG_TAIL;
                let cut = (cut..j.pb.log.len()).find(|&i| j.pb.log.is_char_boundary(i)).unwrap_or(cut);
                j.pb.log.drain(..cut);
            }
        });
    }

    fn finish(&self, id: &str, ok: bool, message: String) {
        self.update(id, |j| {
            j.pb.state = if ok { pb::JobState::Succeeded } else { pb::JobState::Failed } as i32;
            j.pb.finished_unix = now_unix();
            j.pb.message = message;
        });
    }

    fn busy(&self) -> bool {
        self.jobs.lock().unwrap().iter().any(|j| {
            j.pb.state == pb::JobState::Queued as i32 || j.pb.state == pb::JobState::Running as i32
        })
    }

    pub fn start_build(self: &Arc<Self>, engine: &Engine, mcu: &str) -> Result<pb::FirmwareJob, ApiError> {
        let root = self.repo_root.clone().ok_or_else(|| {
            ApiError::invalid_state("firmware builds need the motioncore source tree (start the engine with --repo)")
        })?;
        let m = builder::select_mcu(engine.robot(), Some(mcu)).map_err(|e| ApiError::bad_request(format!("{e:#}")))?;
        if m.board.firmware_target.is_none() {
            return Err(ApiError::bad_request(format!("board '{}' needs no firmware", m.board.id)));
        }
        if self.busy() {
            return Err(ApiError::invalid_state("another firmware job is running"));
        }
        let job = self.create(pb::JobKind::Build, &m.name);
        let (this, id, mcu, config) = (self.clone(), job.id.clone(), m.name.clone(), self.config_path.clone());
        tokio::task::spawn_blocking(move || {
            this.update(&id, |j| j.pb.state = pb::JobState::Running as i32);
            let res = builder::build(&root, &config, Some(&mcu), &mut |l| this.append_log(&id, l));
            match res {
                Ok((c, m)) => {
                    let msg = format!("built {} (build {})", m.image, m.build_hash);
                    this.update(&id, |j| {
                        j.artifact_dir = Some(c.dir.clone());
                        j.pb.artifact = Some(pb::FirmwareArtifact {
                            board: m.board,
                            firmware_version: m.firmware_version,
                            build_hash: m.build_hash,
                            image_sha256: m.image_sha256,
                            toolchain: m.toolchain,
                            required_capabilities: m.required_capabilities,
                        });
                    });
                    this.finish(&id, true, msg);
                }
                Err(e) => {
                    this.append_log(&id, &format!("error: {e:#}"));
                    this.finish(&id, false, format!("{e:#}"));
                }
            }
        });
        Ok(job)
    }

    pub fn start_flash(self: &Arc<Self>, engine: &Engine, build_job_id: &str, confirm: bool) -> Result<pb::FirmwareJob, ApiError> {
        if !confirm {
            return Err(ApiError::bad_request("flashing requires confirm=true (operator confirms drives are powered off)"));
        }
        if engine.drives_energised() {
            return Err(ApiError::invalid_state(format!("refusing to flash while drives may be energised (state: {})", engine.state())));
        }
        let (dir, mcu) = {
            let jobs = self.jobs.lock().unwrap();
            let b = jobs
                .iter()
                .find(|j| j.pb.id == build_job_id)
                .ok_or_else(|| ApiError::not_found(format!("no firmware job '{build_job_id}'")))?;
            if b.pb.kind != pb::JobKind::Build as i32 || b.pb.state != pb::JobState::Succeeded as i32 {
                return Err(ApiError::bad_request(format!("job '{build_job_id}' is not a succeeded build")));
            }
            (b.artifact_dir.clone().ok_or_else(|| ApiError::internal("build has no artifact"))?, b.pb.mcu.clone())
        };
        if self.busy() {
            return Err(ApiError::invalid_state("another firmware job is running"));
        }
        let job = self.create(pb::JobKind::Flash, &mcu);
        let (this, id) = (self.clone(), job.id.clone());
        tokio::task::spawn_blocking(move || {
            this.update(&id, |j| j.pb.state = pb::JobState::Running as i32);
            match builder::flash(&dir, &mut |l| this.append_log(&id, l)) {
                Ok(m) => this.finish(&id, true, format!("flashed build {}", m.build_hash)),
                Err(e) => {
                    this.append_log(&id, &format!("error: {e:#}"));
                    this.finish(&id, false, format!("{e:#}"));
                }
            }
        });
        Ok(job)
    }
}
