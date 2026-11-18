# Contracts & versioning

The three repositories are released independently. Each boundary between them has one owner and one versioned contract.

| Boundary | Contract | Owner | Versioning |
|---|---|---|---|
| web ↔ server | REST + WebSocket (`ratavyr/docs/rest-api.md`) | ratavyr | v1 |
| server/CLI ↔ engine | gRPC `motioncore.engine.v1` (`crates/engine-api`) | motioncore | Tag `engine-api/v0.MINOR.PATCH`; additive within a `0.MINOR` |
| engine ↔ MCU | Binary protocol (`crates/protocol`) | motioncore | `PROTOCOL_VERSION`, checked at handshake |
| engine ↔ plugins | `crates/plugin-sdk` | motioncore | `API_VERSION`, checked per manifest |

**Incompatible peers fail safely.** The engine refuses an MCU with another protocol version or missing capabilities; ratavyr and the CLI refuse an engine whose `api_version` has a different `0.MINOR`; the plugin runtime refuses manifests with another API version.

## Engine API

The `.proto` in `crates/engine-api` is the single source of truth. There are no independently edited copies.

MotionCore is pre-1.0, and so is its engine API: `API_VERSION` is `0.MINOR.PATCH`.

- **PATCH** for additive, compatible changes: new fields, messages, RPCs, enum values. Older clients keep working; new features are detected by the presence of fields, not by comparing versions.
- **MINOR** for breaking changes and major features (coordinated MoveJ is `0.2.0`). Clients accept an engine with the same `0.MINOR` only, so ratavyr is re-released with each minor.
- Field numbers are never reused; removed fields are `reserved`.
- The proto package stays `motioncore.engine.v1`: the `v1` is the package generation, not the version.
- Releases are git tags `engine-api/v0.MINOR.PATCH`.
- From 1.0 on, the usual semver rule applies: the major version marks breaking changes.

The engine-api version is independent of MotionCore's milestones: engine-api `0.2.0` ships in MotionCore v0.3.

| engine-api | Change | Earlier number |
|---|---|---|
| 0.1.0 | Initial contract | 1.0.0 |
| 0.1.1 | `go_package` set | 1.0.1 |
| 0.1.2 | Moved to bzy-logics | 1.0.2 |
| 0.1.3 | `AxisConfig.geometry` | 1.1.0 |
| 0.1.4 | `HomeAll`, collision checking, axis `homing` | 1.2.0 |
| 0.2.0 | `MoveJ`: coordinated joint moves | — |

The API was first numbered `1.x`; it was renumbered to `0.x` because the project is pre-1.0. The old `engine-api/v1.*` tags were removed, and earlier commits still report the old numbers.

### Releasing a contract version

1. Change `engine.proto` following the rules above, and bump `API_VERSION` in `crates/engine-api/src/lib.rs` (PATCH or MINOR, as above).
2. Commit, then tag the commit and push the tag:
   ```bash
   git tag engine-api/v0.1.5
   git push origin main refs/tags/engine-api/v0.1.5
   ```
3. Consumers pin the tag. ratavyr records tag, commit and the file's SHA-256 in `contract.lock` and regenerates its Go bindings; see [ratavyr: engine contract](https://ratavyr.bzy-logics.com/development/engine-contract).

A tag is never moved once pushed: consumers verify the file hash against it.

## REST API

The browser-facing API is owned by ratavyr and versioned separately (v1). See the [REST & WebSocket API](https://ratavyr.bzy-logics.com/reference/rest-api).

## MCU protocol

`PROTOCOL_VERSION` must match exactly between engine and firmware. Any incompatible change to the messages bumps it. Because firmware is built from the same repository as the engine, a matching pair is always available: rebuild and reflash after upgrading the engine if the version changed.

## Plugin API

Each manifest declares `api_version`, which must equal the engine's plugin API version exactly (`motioncore info` prints it). It changes only on breaking changes to `crates/plugin-sdk`.

It is `"0"` while the plugin interface is unstable: so far it covers the manifest and the `activate` / `deactivate` lifecycle, and WASM plugins are not executed yet. `"1"` will mark the first stable interface. It was `"1"` before; only builtin plugins existed then, so no manifest outside this repository was affected.
