//! Restricted firmware builder: configure, compile, flash and verify.
//!
//! * Builds are reproducible from a lockfile (board, Rust target, toolchain,
//!   workspace version, protocol version, required capabilities, robot-profile
//!   hash). The first 8 hex digits of the lockfile hash are embedded in the
//!   image (`MOTIONCORE_BUILD_HASH`) and reported by the MCU at handshake, so
//!   [`verify`] can prove which build is running.
//! * Only in-repo board crates (`boards/*`) are compiled, with `cargo --locked`.
//!   Nothing supplied by plugins or API clients is ever executed.
//! * Flashing safety (drives off, operator confirmation) is enforced by the
//!   callers that know the machine state: the engine and the CLI.
//! * Each board declares how it is flashed ([`FlashMethod`]): an SWD probe
//!   via probe-rs (STM32F411) or a UF2 copied to the USB bootloader drive
//!   (RP2040 BOOTSEL). UF2 images are generated here from the ELF and
//!   validated block by block.

use std::io::{BufRead, BufReader, Read, Write};
use std::path::{Path, PathBuf};
use std::process::{Child, Command, Stdio};
use std::sync::mpsc;
use std::time::{Duration, Instant};

use anyhow::{bail, Context, Result};
use motioncore_device_graph::{Board, FlashMethod, ResolvedMcu, ResolvedRobot, BOARDS};
use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct Lockfile {
    pub lock_version: u32,
    pub board: String,
    pub firmware_target: String,
    pub rust_target: String,
    pub firmware_version: String,
    pub toolchain: String,
    pub profile_sha256: String,
    pub required_capabilities: Vec<String>,
    pub protocol_version: u16,
}

/// `manifest.json` written next to a built image.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Manifest {
    pub board: String,
    pub firmware_target: String,
    pub rust_target: String,
    pub firmware_version: String,
    pub build_hash: String,
    pub image: String,
    pub image_sha256: String,
    pub toolchain: String,
    pub protocol_version: u16,
    pub required_capabilities: Vec<String>,
    pub built_unix: u64,
    /// UF2 image for bootloader flashing (boards with [`FlashMethod::Uf2`]).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub uf2: Option<String>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub uf2_sha256: Option<String>,
}

pub struct Configured {
    pub mcu: String,
    pub dir: PathBuf,
    pub lock: Lockfile,
    pub build_hash: String,
}

/// All boards known to the device graph.
pub fn targets() -> &'static [Board] {
    BOARDS
}

/// Look up a board by id.
pub fn board(id: &str) -> Result<&'static Board> {
    motioncore_device_graph::boards::find(id).with_context(|| {
        let known: Vec<&str> = BOARDS.iter().map(|b| b.id).collect();
        format!("unknown board '{id}' (known: {})", known.join(", "))
    })
}

/// The reference robot profile for a board (`--board` builds).
pub fn reference_profile(root: &Path, id: &str) -> Result<PathBuf> {
    let b = board(id)?;
    if b.is_virtual {
        bail!("board '{id}' is virtual and needs no firmware");
    }
    let name = b.reference_profile.with_context(|| format!("board '{id}' has no reference profile; use --config"))?;
    Ok(root.join("robot-profiles").join(name))
}

/// Artifact directory of a `--board` build.
pub fn board_artifact_dir(root: &Path, id: &str) -> PathBuf {
    root.join("build/firmware").join(id)
}

fn uf2_region(board: &Board) -> Result<uf2::FlashRegion> {
    match board.id {
        "rp2040-pico" => Ok(uf2::RP2040_FLASH),
        other => bail!("no UF2 flash layout known for board '{other}'"),
    }
}

/// Find the `motioncore` repository root (workspace with `boards/`) at or above `start`.
pub fn find_repo_root(start: &Path) -> Result<PathBuf> {
    let mut dir = start.canonicalize().unwrap_or_else(|_| start.to_path_buf());
    loop {
        if dir.join("Cargo.toml").is_file() && dir.join("boards").is_dir() && dir.join("crates").is_dir() {
            return Ok(dir);
        }
        if !dir.pop() {
            bail!("not inside the motioncore repository (no Cargo.toml + boards/ + crates/ found above {})", start.display());
        }
    }
}

pub fn sha256_hex(data: &[u8]) -> String {
    Sha256::digest(data).iter().map(|b| format!("{b:02x}")).collect()
}

fn toolchain(root: &Path) -> Result<String> {
    let out = Command::new("rustc").arg("-V").current_dir(root).output().context("running rustc -V")?;
    Ok(String::from_utf8_lossy(&out.stdout).trim().to_string())
}

pub fn select_mcu<'a>(robot: &'a ResolvedRobot, mcu: Option<&str>) -> Result<&'a ResolvedMcu> {
    match mcu.filter(|m| !m.is_empty()) {
        Some(name) => robot.mcus.get(name).with_context(|| format!("no mcu '{name}' in robot configuration")),
        None if robot.mcus.len() == 1 => Ok(robot.mcus.values().next().unwrap()),
        None => bail!("robot has several MCUs; specify which one"),
    }
}

fn board_target(board: &Board) -> Result<(&'static str, &'static str)> {
    match (board.firmware_target, board.rust_target) {
        (Some(t), Some(r)) => Ok((t, r)),
        _ => bail!("board '{}' has no firmware target (virtual boards need no firmware)", board.id),
    }
}

/// Resolve the configuration and write `build/firmware/<mcu>/motioncore.lock.toml`.
pub fn configure(root: &Path, config: &Path, mcu: Option<&str>) -> Result<Configured> {
    configure_in(root, config, mcu, None)
}

/// [`configure`] with an explicit artifact directory name (default: the MCU
/// name). `--board` builds use the board id so boards never share a directory.
pub fn configure_in(root: &Path, config: &Path, mcu: Option<&str>, dir_name: Option<&str>) -> Result<Configured> {
    let text = std::fs::read(config).with_context(|| format!("reading {}", config.display()))?;
    let robot = motioncore_device_graph::load(config)?;
    let m = select_mcu(&robot, mcu)?;
    let (target, rust_target) = board_target(m.board)?;
    if !root.join("boards").join(target).join("Cargo.toml").is_file() {
        bail!("board crate boards/{target} not found");
    }
    let mut caps: Vec<String> = m.required_firmware_capabilities.iter().map(|s| s.to_string()).collect();
    caps.sort();
    let lock = Lockfile {
        lock_version: 1,
        board: m.board.id.into(),
        firmware_target: target.into(),
        rust_target: rust_target.into(),
        firmware_version: env!("CARGO_PKG_VERSION").into(),
        toolchain: toolchain(root)?,
        profile_sha256: sha256_hex(&text),
        required_capabilities: caps,
        protocol_version: motioncore_protocol::PROTOCOL_VERSION,
    };
    let lock_text = toml::to_string_pretty(&lock)?;
    let build_hash = sha256_hex(lock_text.as_bytes())[..8].to_string();
    let dir = root.join("build/firmware").join(dir_name.unwrap_or(&m.name));
    std::fs::create_dir_all(&dir)?;
    std::fs::write(dir.join("motioncore.lock.toml"), &lock_text)?;
    Ok(Configured { mcu: m.name.clone(), dir, lock, build_hash })
}

/// Longest output line passed on; longer lines are split into chunks.
pub const MAX_LINE_BYTES: usize = 4096;
/// `build.log` stops growing at this size (a truncation note is appended).
pub const MAX_BUILD_LOG_BYTES: u64 = 16 * 1024 * 1024;
/// Wall-clock limit for a firmware compile.
pub const BUILD_TIMEOUT: Duration = Duration::from_secs(30 * 60);
/// Wall-clock limit for one probe-rs invocation.
pub const FLASH_TIMEOUT: Duration = Duration::from_secs(5 * 60);

/// Kills and reaps the child unless it already exited, so an error, timeout
/// or panic in the caller never leaves the process running.
struct ChildGuard(Child);

impl Drop for ChildGuard {
    fn drop(&mut self) {
        if let Ok(None) = self.0.try_wait() {
            let _ = self.0.kill();
            let _ = self.0.wait();
        }
    }
}

/// Forward `r` line by line (each at most [`MAX_LINE_BYTES`]) into `tx`.
fn pump(r: impl Read, tx: mpsc::SyncSender<String>) {
    let mut r = BufReader::new(r);
    let mut buf = Vec::new();
    loop {
        buf.clear();
        match r.by_ref().take(MAX_LINE_BYTES as u64).read_until(b'\n', &mut buf) {
            Ok(0) | Err(_) => return,
            Ok(_) => {
                let line = String::from_utf8_lossy(&buf);
                if tx.send(line.trim_end_matches(['\r', '\n']).to_string()).is_err() {
                    return;
                }
            }
        }
    }
}

/// Run a command, streaming each output line to `log` as it arrives. Memory
/// use is bounded (a small line channel), and the process is killed if it
/// runs longer than `timeout` or this function returns early.
fn run_logged(mut cmd: Command, timeout: Duration, log: &mut dyn FnMut(&str)) -> Result<bool> {
    let program = cmd.get_program().to_string_lossy().into_owned();
    cmd.stdin(Stdio::null()).stdout(Stdio::piped()).stderr(Stdio::piped());
    let mut child = ChildGuard(cmd.spawn().with_context(|| format!("starting {program}"))?);
    let (tx, rx) = mpsc::sync_channel(256);
    let stdout = child.0.stdout.take().unwrap();
    let stderr = child.0.stderr.take().unwrap();
    let tx2 = tx.clone();
    std::thread::spawn(move || pump(stdout, tx2));
    std::thread::spawn(move || pump(stderr, tx));
    let deadline = Instant::now() + timeout;
    loop {
        match rx.recv_timeout(deadline.saturating_duration_since(Instant::now())) {
            Ok(line) => log(&line),
            Err(mpsc::RecvTimeoutError::Disconnected) => break,
            Err(mpsc::RecvTimeoutError::Timeout) => bail!("{program} timed out after {timeout:?} and was stopped"),
        }
    }
    // Output closed; the process is exiting.
    loop {
        if let Some(status) = child.0.try_wait()? {
            return Ok(status.success());
        }
        if Instant::now() >= deadline {
            bail!("{program} timed out after {timeout:?} and was stopped");
        }
        std::thread::sleep(Duration::from_millis(20));
    }
}

/// Size-capped log file writer.
struct CappedLog {
    file: std::fs::File,
    written: u64,
    truncated: bool,
}

impl CappedLog {
    fn line(&mut self, l: &str) {
        if self.truncated {
            return;
        }
        if self.written + l.len() as u64 + 1 > MAX_BUILD_LOG_BYTES {
            self.truncated = true;
            let _ = writeln!(self.file, "... build.log truncated at {MAX_BUILD_LOG_BYTES} bytes");
            return;
        }
        self.written += l.len() as u64 + 1;
        let _ = writeln!(self.file, "{l}");
    }
}

/// Configure and compile. Writes `firmware.elf`, `manifest.json` and
/// `build.log` (plus `firmware.uf2` for UF2 boards).
pub fn build(root: &Path, config: &Path, mcu: Option<&str>, log: &mut dyn FnMut(&str)) -> Result<(Configured, Manifest)> {
    build_in(root, config, mcu, None, log)
}

/// Build the board's reference profile into `build/firmware/<board>/`.
pub fn build_board(root: &Path, id: &str, log: &mut dyn FnMut(&str)) -> Result<(Configured, Manifest)> {
    let profile = reference_profile(root, id)?;
    log(&format!("using reference profile {}", profile.display()));
    build_in(root, &profile, None, Some(id), log)
}

fn build_in(root: &Path, config: &Path, mcu: Option<&str>, dir_name: Option<&str>, log: &mut dyn FnMut(&str)) -> Result<(Configured, Manifest)> {
    let c = configure_in(root, config, mcu, dir_name)?;
    let package = format!("motioncore-board-{}", c.lock.firmware_target);
    log(&format!("building {package} for {} ({}) with build hash {}", c.lock.board, c.lock.rust_target, c.build_hash));
    let mut cmd = Command::new("cargo");
    cmd.args(["build", "--locked", "-p", &package, "--profile", "firmware", "--target", &c.lock.rust_target])
        .current_dir(root)
        .env("MOTIONCORE_BUILD_HASH", &c.build_hash);
    let mut file_log = CappedLog { file: std::fs::File::create(c.dir.join("build.log"))?, written: 0, truncated: false };
    let ok = run_logged(cmd, BUILD_TIMEOUT, &mut |l| {
        file_log.line(l);
        log(l);
    });
    let _ = file_log.file.flush();
    let ok = ok?;
    if !ok {
        bail!("firmware build failed; see {}", c.dir.join("build.log").display());
    }
    let bin = root.join("target").join(&c.lock.rust_target).join("firmware").join(&package);
    let image = std::fs::read(&bin).with_context(|| format!("reading {}", bin.display()))?;
    std::fs::write(c.dir.join("firmware.elf"), &image)?;
    let board = board(&c.lock.board)?;
    let (uf2, uf2_sha256) = match board.flash {
        FlashMethod::Uf2 { family_id, .. } => {
            let bytes = uf2::elf_to_uf2(&image, family_id, uf2_region(board)?)?;
            uf2::validate(&bytes, family_id, uf2_region(board)?)?;
            std::fs::write(c.dir.join("firmware.uf2"), &bytes)?;
            log(&format!("uf2: {} ({} blocks)", c.dir.join("firmware.uf2").display(), bytes.len() / uf2::BLOCK));
            (Some("firmware.uf2".to_string()), Some(sha256_hex(&bytes)))
        }
        _ => (None, None),
    };
    let manifest = Manifest {
        board: c.lock.board.clone(),
        firmware_target: c.lock.firmware_target.clone(),
        rust_target: c.lock.rust_target.clone(),
        firmware_version: c.lock.firmware_version.clone(),
        build_hash: c.build_hash.clone(),
        image: "firmware.elf".into(),
        image_sha256: sha256_hex(&image),
        toolchain: c.lock.toolchain.clone(),
        protocol_version: c.lock.protocol_version,
        required_capabilities: c.lock.required_capabilities.clone(),
        built_unix: std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH)?.as_secs(),
        uf2,
        uf2_sha256,
    };
    std::fs::write(c.dir.join("manifest.json"), serde_json::to_string_pretty(&manifest)?)?;
    log(&format!("ok: {} ({} bytes, sha256 {})", c.dir.join("firmware.elf").display(), image.len(), &manifest.image_sha256[..16]));
    Ok((c, manifest))
}

/// Load an artifact directory and check the image against its manifest.
pub fn load_artifact(dir: &Path) -> Result<Manifest> {
    let m: Manifest = serde_json::from_str(&std::fs::read_to_string(dir.join("manifest.json")).context("reading manifest.json (build first)")?)?;
    let image = std::fs::read(dir.join(&m.image))?;
    if sha256_hex(&image) != m.image_sha256 {
        bail!("image checksum mismatch — artifact was modified after build");
    }
    let board = board(&m.board)?;
    if m.firmware_target != board.firmware_target.unwrap_or_default() || m.rust_target != board.rust_target.unwrap_or_default() {
        bail!(
            "artifact targets {} / {}, but board '{}' needs {} / {}",
            m.firmware_target,
            m.rust_target,
            board.id,
            board.firmware_target.unwrap_or("-"),
            board.rust_target.unwrap_or("-")
        );
    }
    if let FlashMethod::Uf2 { family_id, .. } = board.flash {
        let (Some(name), Some(sum)) = (&m.uf2, &m.uf2_sha256) else { bail!("artifact has no UF2 image (rebuild)") };
        let bytes = std::fs::read(dir.join(name))?;
        if &sha256_hex(&bytes) != sum {
            bail!("UF2 checksum mismatch — artifact was modified after build");
        }
        uf2::validate(&bytes, family_id, uf2_region(board)?)?;
    }
    Ok(m)
}

/// probe-rs chip name for a board.
pub fn chip_for_board(id: &str) -> Result<&'static str> {
    match board(id)?.flash {
        FlashMethod::ProbeRs { chip } => Ok(chip),
        _ => bail!("board '{id}' is not flashed with probe-rs"),
    }
}

/// Where the UF2 bootloader drive may be mounted.
fn uf2_volume_candidates(label: &str) -> Vec<PathBuf> {
    let mut v = Vec::new();
    if cfg!(windows) {
        for d in b'D'..=b'Z' {
            v.push(PathBuf::from(format!("{}:\\", d as char)));
        }
    } else {
        v.push(PathBuf::from("/Volumes").join(label));
        if let Ok(user) = std::env::var("USER") {
            v.push(PathBuf::from("/media").join(&user).join(label));
            v.push(PathBuf::from("/run/media").join(&user).join(label));
        }
        v.push(PathBuf::from("/media").join(label));
        v.push(PathBuf::from("/mnt").join(label));
    }
    v
}

/// `true` if `dir` is a mounted UF2 bootloader drive for `label`
/// (its `INFO_UF2.TXT` names the board).
pub fn is_uf2_volume(dir: &Path, label: &str) -> bool {
    std::fs::read_to_string(dir.join("INFO_UF2.TXT")).is_ok_and(|t| t.contains(&format!("Board-ID: {label}")))
}

/// Find the bootloader drive, or explain how to get one.
pub fn find_uf2_volume(label: &str) -> Result<PathBuf> {
    let found: Vec<PathBuf> = uf2_volume_candidates(label).into_iter().filter(|d| is_uf2_volume(d, label)).collect();
    match found.len() {
        1 => Ok(found.into_iter().next().unwrap()),
        0 => bail!(
            "no {label} bootloader drive found: hold BOOTSEL while plugging in USB (or reset with BOOTSEL held), \
             then retry, or pass the mount point explicitly"
        ),
        _ => bail!("several {label} drives found ({found:?}); pass the mount point explicitly"),
    }
}

/// Options for [`flash_with`].
#[derive(Debug, Default, Clone)]
pub struct FlashOptions {
    /// Mounted UF2 bootloader drive (default: auto-detect).
    pub uf2_volume: Option<PathBuf>,
    /// How long to wait for the bootloader to take the image.
    pub uf2_timeout: Option<Duration>,
}

/// Flash an artifact with the board's method. Callers must have checked that
/// drives are de-energised and obtained operator confirmation.
pub fn flash(dir: &Path, log: &mut dyn FnMut(&str)) -> Result<Manifest> {
    flash_with(dir, &FlashOptions::default(), log)
}

pub fn flash_with(dir: &Path, opts: &FlashOptions, log: &mut dyn FnMut(&str)) -> Result<Manifest> {
    let m = load_artifact(dir)?;
    match board(&m.board)?.flash {
        FlashMethod::ProbeRs { .. } => flash_probe_rs(dir, m, log),
        FlashMethod::Uf2 { volume_label, .. } => flash_uf2(dir, m, volume_label, opts, log),
        FlashMethod::None => bail!("board '{}' has no firmware to flash", m.board),
    }
}

/// Copy the UF2 to the bootloader drive and wait until the bootloader takes
/// it (the drive disappears when the MCU reboots into the new image). That is
/// only evidence the copy was accepted; `verify` proves what is running.
fn flash_uf2(dir: &Path, m: Manifest, label: &str, opts: &FlashOptions, log: &mut dyn FnMut(&str)) -> Result<Manifest> {
    let uf2 = m.uf2.as_deref().context("artifact has no UF2 image")?;
    let volume = match &opts.uf2_volume {
        Some(v) if is_uf2_volume(v, label) => v.clone(),
        Some(v) => bail!("{} is not a {label} bootloader drive (no matching INFO_UF2.TXT)", v.display()),
        None => find_uf2_volume(label)?,
    };
    log(&format!("copying {uf2} (build {}, fw {}) to {}", m.build_hash, m.firmware_version, volume.display()));
    std::fs::copy(dir.join(uf2), volume.join("firmware.uf2")).with_context(|| format!("copying to {}", volume.display()))?;
    let deadline = Instant::now() + opts.uf2_timeout.unwrap_or(Duration::from_secs(15));
    while is_uf2_volume(&volume, label) {
        if Instant::now() >= deadline {
            bail!("the {label} drive is still mounted after copying: the bootloader did not accept the image; nothing is confirmed flashed");
        }
        std::thread::sleep(Duration::from_millis(200));
    }
    log("bootloader accepted the image and the board rebooted (not yet verified)");
    log("confirm the running build with `motioncore firmware verify`");
    Ok(m)
}

fn flash_probe_rs(dir: &Path, m: Manifest, log: &mut dyn FnMut(&str)) -> Result<Manifest> {
    let chip = chip_for_board(&m.board)?;
    log(&format!("flashing {} (build {}, fw {}) to {chip} via probe-rs", m.image, m.build_hash, m.firmware_version));
    let mut cmd = Command::new("probe-rs");
    cmd.args(["download", "--chip", chip]).arg(dir.join(&m.image));
    match run_logged(cmd, FLASH_TIMEOUT, log) {
        Ok(true) => {}
        Ok(false) => bail!("flashing failed; the previous image may still be on the MCU — re-run or recover via SWD"),
        Err(e) => bail!("{e:#} (install probe-rs: `cargo install probe-rs-tools`)"),
    }
    let mut reset = Command::new("probe-rs");
    reset.args(["reset", "--chip", chip]);
    run_logged(reset, FLASH_TIMEOUT, log)?;
    log("probe-rs reported success (not yet verified)");
    log("confirm the running build with `motioncore firmware verify`");
    Ok(m)
}

#[derive(Debug)]
pub struct VerifyReport {
    pub target: String,
    pub board: String,
    pub firmware_version: String,
    pub build_hash: String,
    pub capabilities: Vec<String>,
    pub problems: Vec<String>,
}

/// Connect to the MCU on `device`, identify it and compare with the artifact.
pub async fn verify(dir: &Path, device: &str) -> Result<VerifyReport> {
    use tokio_serial::SerialPortBuilderExt;
    let m = load_artifact(dir)?;
    let port = tokio_serial::new(device, 250_000).open_native_async().with_context(|| format!("opening {device}"))?;
    let (link, _ev) = motioncore_mcu_manager::McuLink::start(port);
    let id = link.identify().await?;
    let build_hash = format!("{:08x}", id.build_hash);
    let capabilities: Vec<String> = motioncore_protocol::capability_names(id.capabilities).map(String::from).collect();
    let mut problems = Vec::new();
    if build_hash != m.build_hash {
        problems.push(format!("build hash {build_hash} != artifact {}", m.build_hash));
    }
    if id.board.as_str() != m.board {
        problems.push(format!("board {} != artifact {}", id.board, m.board));
    }
    for c in &m.required_capabilities {
        if !capabilities.contains(c) {
            problems.push(format!("missing capability {c}"));
        }
    }
    Ok(VerifyReport {
        target: id.target.to_string(),
        board: id.board.to_string(),
        firmware_version: format!("{}.{}.{}", id.firmware_version.0, id.firmware_version.1, id.firmware_version.2),
        build_hash,
        capabilities,
        problems,
    })
}

#[derive(Debug)]
pub struct PortInfo {
    pub name: String,
    pub description: String,
}

pub fn list_ports() -> Result<Vec<PortInfo>> {
    Ok(tokio_serial::available_ports()?
        .into_iter()
        .map(|p| PortInfo {
            name: p.port_name,
            description: match p.port_type {
                tokio_serial::SerialPortType::UsbPort(u) => {
                    format!("usb {:04x}:{:04x} {} {}", u.vid, u.pid, u.manufacturer.unwrap_or_default(), u.product.unwrap_or_default())
                }
                other => format!("{other:?}"),
            },
        })
        .collect())
}

pub mod uf2;

#[cfg(test)]
mod tests {
    use super::*;

    fn root() -> PathBuf {
        find_repo_root(Path::new(env!("CARGO_MANIFEST_DIR"))).unwrap()
    }

    #[test]
    fn configure_is_deterministic() {
        let r = root();
        let cfg = r.join("robot-profiles/single-axis-stm32f411.toml");
        let a = configure(&r, &cfg, None).unwrap();
        let b = configure(&r, &cfg, Some("main")).unwrap();
        assert_eq!(a.lock, b.lock);
        assert_eq!(a.build_hash, b.build_hash);
        assert_eq!(a.lock.firmware_target, "stm32f4");
        assert!(a.lock.required_capabilities.contains(&"endstop".to_string()));
    }

    #[test]
    fn virtual_boards_have_no_firmware() {
        let r = root();
        let err = configure(&r, &r.join("robot-profiles/single-axis-virtual.toml"), None).err().unwrap();
        assert!(err.to_string().contains("no firmware target"));
    }

    #[test]
    fn tampered_artifact_is_rejected() {
        let dir = std::env::temp_dir().join(format!("mc-artifact-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        std::fs::write(dir.join("firmware.elf"), b"image").unwrap();
        let m = Manifest {
            board: "stm32f411-blackpill".into(),
            firmware_target: "stm32f4".into(),
            rust_target: "thumbv7em-none-eabihf".into(),
            firmware_version: "0.2.0".into(),
            build_hash: "00000000".into(),
            image: "firmware.elf".into(),
            image_sha256: sha256_hex(b"image"),
            toolchain: "test".into(),
            protocol_version: 1,
            required_capabilities: vec![],
            built_unix: 0,
            uf2: None,
            uf2_sha256: None,
        };
        std::fs::write(dir.join("manifest.json"), serde_json::to_string(&m).unwrap()).unwrap();
        assert!(load_artifact(&dir).is_ok());
        std::fs::write(dir.join("firmware.elf"), b"evil").unwrap();
        assert!(load_artifact(&dir).unwrap_err().to_string().contains("checksum"));
        std::fs::remove_dir_all(&dir).ok();
    }
}

#[cfg(test)]
mod run_tests {
    use super::*;

    #[test]
    fn run_logged_streams_output_and_reports_status() {
        let mut lines = Vec::new();
        let mut cmd = Command::new("cargo");
        cmd.arg("--version");
        let ok = run_logged(cmd, Duration::from_secs(60), &mut |l| lines.push(l.to_string())).unwrap();
        assert!(ok);
        assert!(lines.iter().any(|l| l.starts_with("cargo ")), "{lines:?}");
    }
}

#[cfg(test)]
mod board_tests {
    use super::*;

    fn root() -> PathBuf {
        find_repo_root(Path::new(env!("CARGO_MANIFEST_DIR"))).unwrap()
    }

    fn tmp(name: &str) -> PathBuf {
        let d = std::env::temp_dir().join(format!("mc-{name}-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&d);
        std::fs::create_dir_all(&d).unwrap();
        d
    }

    #[test]
    fn board_selection() {
        assert_eq!(board("rp2040-pico").unwrap().rust_target, Some("thumbv6m-none-eabi"));
        assert_eq!(board("stm32f411-blackpill").unwrap().rust_target, Some("thumbv7em-none-eabihf"));
        assert!(board("esp32").unwrap_err().to_string().contains("unknown board"));
        assert!(reference_profile(&root(), "virtual").unwrap_err().to_string().contains("virtual"));
        assert_eq!(chip_for_board("stm32f411-blackpill").unwrap(), "STM32F411CEUx");
        assert!(chip_for_board("rp2040-pico").is_err(), "RP2040 flashes via UF2");
    }

    #[test]
    fn board_builds_use_reference_profile_and_own_directory() {
        let r = root();
        for (id, target, rust) in [("rp2040-pico", "rp2040", "thumbv6m-none-eabi"), ("stm32f411-blackpill", "stm32f4", "thumbv7em-none-eabihf")] {
            let profile = reference_profile(&r, id).unwrap();
            let c = configure_in(&r, &profile, None, Some(id)).unwrap();
            assert_eq!(c.dir, board_artifact_dir(&r, id));
            assert_eq!((c.lock.board.as_str(), c.lock.firmware_target.as_str(), c.lock.rust_target.as_str()), (id, target, rust));
            assert!(c.lock.required_capabilities.contains(&"stepper.step_dir".to_string()));
        }
    }

    fn uf2_artifact(dir: &Path) -> Manifest {
        // Synthetic ELF: one segment at the flash start.
        let mut elf = vec![0u8; 52 + 32];
        elf[..6].copy_from_slice(b"\x7fELF\x01\x01");
        elf[16] = 2;
        elf[18] = 40;
        elf[28] = 52;
        elf[42] = 32;
        elf[44] = 1;
        elf[52] = 1; // PT_LOAD
        elf[56..60].copy_from_slice(&84u32.to_le_bytes());
        elf[64..68].copy_from_slice(&0x1000_0000u32.to_le_bytes());
        elf[68..72].copy_from_slice(&300u32.to_le_bytes());
        elf.extend(std::iter::repeat_n(0x5A, 300));
        let uf2 = uf2::elf_to_uf2(&elf, 0xE48B_FF56, uf2::RP2040_FLASH).unwrap();
        std::fs::write(dir.join("firmware.elf"), &elf).unwrap();
        std::fs::write(dir.join("firmware.uf2"), &uf2).unwrap();
        let m = Manifest {
            board: "rp2040-pico".into(),
            firmware_target: "rp2040".into(),
            rust_target: "thumbv6m-none-eabi".into(),
            firmware_version: "0.2.0".into(),
            build_hash: "00000000".into(),
            image: "firmware.elf".into(),
            image_sha256: sha256_hex(&elf),
            toolchain: "test".into(),
            protocol_version: 1,
            required_capabilities: vec![],
            built_unix: 0,
            uf2: Some("firmware.uf2".into()),
            uf2_sha256: Some(sha256_hex(&uf2)),
        };
        std::fs::write(dir.join("manifest.json"), serde_json::to_string(&m).unwrap()).unwrap();
        m
    }

    #[test]
    fn uf2_artifacts_are_integrity_checked() {
        let dir = tmp("uf2-art");
        uf2_artifact(&dir);
        assert!(load_artifact(&dir).is_ok());
        let mut uf2 = std::fs::read(dir.join("firmware.uf2")).unwrap();
        uf2[40] ^= 1;
        std::fs::write(dir.join("firmware.uf2"), &uf2).unwrap();
        assert!(load_artifact(&dir).unwrap_err().to_string().contains("UF2 checksum"));

        // An artifact built for one board cannot pass as another.
        let mut m = uf2_artifact(&dir);
        m.board = "stm32f411-blackpill".into();
        std::fs::write(dir.join("manifest.json"), serde_json::to_string(&m).unwrap()).unwrap();
        assert!(load_artifact(&dir).unwrap_err().to_string().contains("needs"));
        std::fs::remove_dir_all(&dir).ok();
    }

    fn fake_bootsel(name: &str) -> PathBuf {
        let vol = tmp(name);
        std::fs::write(vol.join("INFO_UF2.TXT"), "UF2 Bootloader v3.0\nModel: Raspberry Pi RP2\nBoard-ID: RPI-RP2\n").unwrap();
        vol
    }

    #[test]
    fn uf2_flash_reports_failure_when_bootloader_ignores_image() {
        let dir = tmp("uf2-flash-stuck");
        uf2_artifact(&dir);
        let vol = fake_bootsel("bootsel-stuck");
        let opts = FlashOptions { uf2_volume: Some(vol.clone()), uf2_timeout: Some(Duration::from_millis(300)) };
        let err = flash_with(&dir, &opts, &mut |_| {}).unwrap_err().to_string();
        assert!(err.contains("nothing is confirmed flashed"), "{err}");
        assert!(vol.join("firmware.uf2").is_file(), "image was copied");
        std::fs::remove_dir_all(&dir).ok();
        std::fs::remove_dir_all(&vol).ok();
    }

    #[test]
    fn uf2_flash_succeeds_when_bootloader_takes_image() {
        let dir = tmp("uf2-flash-ok");
        uf2_artifact(&dir);
        let vol = fake_bootsel("bootsel-ok");
        // Simulated bootloader: once the image lands, the drive goes away.
        let v2 = vol.clone();
        let boot = std::thread::spawn(move || {
            for _ in 0..100 {
                if v2.join("firmware.uf2").is_file() {
                    std::fs::remove_file(v2.join("INFO_UF2.TXT")).unwrap();
                    return;
                }
                std::thread::sleep(Duration::from_millis(20));
            }
        });
        let mut lines = Vec::new();
        let opts = FlashOptions { uf2_volume: Some(vol.clone()), uf2_timeout: Some(Duration::from_secs(5)) };
        flash_with(&dir, &opts, &mut |l| lines.push(l.to_string())).unwrap();
        boot.join().unwrap();
        assert!(lines.iter().any(|l| l.contains("not yet verified")), "{lines:?}");
        std::fs::remove_dir_all(&dir).ok();
        std::fs::remove_dir_all(&vol).ok();
    }

    #[test]
    fn uf2_flash_rejects_a_drive_that_is_not_the_bootloader() {
        let dir = tmp("uf2-flash-wrong");
        uf2_artifact(&dir);
        let not_boot = tmp("usb-stick");
        let opts = FlashOptions { uf2_volume: Some(not_boot.clone()), uf2_timeout: None };
        assert!(flash_with(&dir, &opts, &mut |_| {}).unwrap_err().to_string().contains("not a RPI-RP2"));
        assert!(!not_boot.join("firmware.uf2").exists(), "nothing copied");
        std::fs::remove_dir_all(&dir).ok();
        std::fs::remove_dir_all(&not_boot).ok();
    }
}
