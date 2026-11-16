# Contracts & versioning

The three repositories are released independently. Each boundary between them has one owner and one versioned contract.

| Boundary | Contract | Owner | Versioning |
|---|---|---|---|
| web ↔ server | REST + WebSocket (`ratavyr/docs/rest-api.md`) | ratavyr | v1 |
| server/CLI ↔ engine | gRPC `motioncore.engine.v1` (`crates/engine-api`) | motioncore | Tag `engine-api/vX.Y.Z`; additive within v1 |
| engine ↔ MCU | Binary protocol (`crates/protocol`) | motioncore | `PROTOCOL_VERSION`, checked at handshake |
| engine ↔ plugins | `crates/plugin-sdk` | motioncore | `API_VERSION`, checked per manifest |

**Incompatible peers fail safely.** The engine refuses an MCU with another protocol version or missing capabilities; ratavyr and the CLI refuse an engine whose `api_version` is not `1.x`; the plugin runtime refuses manifests with another API version.

## Engine API

The `.proto` in `crates/engine-api` is the single source of truth. There are no independently edited copies.

Rules for v1:

- Only additive changes: new fields, messages, RPCs, enum values.
- Field numbers are never reused; removed fields are `reserved`.
- Breaking changes go in a new package, `motioncore.engine.v2`.
- Releases are git tags `engine-api/vX.Y.Z`.

### Releasing a contract version

1. Change `engine.proto` following the rules above, and bump `API_VERSION` in `crates/engine-api/src/lib.rs` when the API itself changes.
2. Commit, then tag the commit and push the tag:
   ```bash
   git tag engine-api/v1.1.0
   git push origin main refs/tags/engine-api/v1.1.0
   ```
3. Consumers pin the tag. ratavyr records tag, commit and the file's SHA-256 in `contract.lock` and regenerates its Go bindings; see [ratavyr: engine contract](https://ratavyr.bzy-logics.com/development/engine-contract).

A tag is never moved once pushed: consumers verify the file hash against it.

## REST API

The browser-facing API is owned by ratavyr and versioned separately (v1). See the [REST & WebSocket API](https://ratavyr.bzy-logics.com/reference/rest-api).

## MCU protocol

`PROTOCOL_VERSION` must match exactly between engine and firmware. Any incompatible change to the messages bumps it. Because firmware is built from the same repository as the engine, a matching pair is always available: rebuild and reflash after upgrading the engine if the version changed.

## Plugin API

Each manifest declares `api_version`, which must equal the engine's plugin API version (`motioncore info` prints it).
