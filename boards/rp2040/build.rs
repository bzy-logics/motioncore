//! Linker setup for the RP2040 and build-hash embedding.
use std::{env, fs, path::PathBuf};

fn main() {
    let out = PathBuf::from(env::var_os("OUT_DIR").unwrap());
    fs::write(out.join("memory.x"), include_bytes!("memory.x")).unwrap();
    println!("cargo:rustc-link-search={}", out.display());
    // cortex-m-rt linker script; set here so workspace-root builds link too.
    println!("cargo:rustc-link-arg-bins=-Tlink.x");
    println!("cargo:rerun-if-changed=memory.x");
    // Set by `motioncore firmware build` from the lockfile hash; 0 for ad-hoc builds.
    println!("cargo:rerun-if-env-changed=MOTIONCORE_BUILD_HASH");
}
