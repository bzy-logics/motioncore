//! Compile the engine contract with `protox` (pure Rust, no `protoc` needed).
fn main() -> Result<(), Box<dyn std::error::Error>> {
    let fds = protox::compile(["motioncore/engine/v1/engine.proto"], ["proto"])?;
    tonic_build::configure().build_server(true).build_client(true).compile_fds(fds)?;
    println!("cargo:rerun-if-changed=proto");
    Ok(())
}
