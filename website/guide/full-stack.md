# Running the full stack

The engine works on its own, but most operators want a browser UI. That takes two more processes: **ratavyr**, the Go gateway that exposes REST and WebSocket, and **sindrava**, the React UI.

```text
browser ── sindrava (Vite dev server :5173) ──/api proxy──▶ ratavyr (:8080) ──gRPC──▶ motioncore engine
```

## Prerequisites

| Component | Needs |
|---|---|
| motioncore | Rust stable |
| ratavyr | Go (version in `go.mod`); all code-generation tools are pinned via `go tool` |
| sindrava | Node.js and npm |

Clone the three repositories side by side; ratavyr's scripts look for motioncore at `../motioncore` by default.

```text
work/
├── motioncore/
├── ratavyr/
└── sindrava/
```

## 1. Engine

```bash
cd motioncore
cargo build
target/debug/motioncore start --config robot-profiles/single-axis-virtual.toml
```

## 2. Gateway

> Full documentation: [ratavyr.bzy-logics.com](https://ratavyr.bzy-logics.com)

```bash
cd ratavyr
go run ./cmd/server          # API on http://127.0.0.1:8080
```

ratavyr finds the engine at the platform default address. If you started the engine elsewhere, set `MOTIONCORE_ENGINE_ADDR` (for example `127.0.0.1:7600` or `unix:///run/motioncore/engine.sock`).

With no tokens configured, authentication is **off** and ratavyr refuses to listen on anything but loopback. That is fine on a development machine; see [Deploying the gateway](https://ratavyr.bzy-logics.com/guide/deployment) before exposing it on a network.

Check it:

```bash
curl http://127.0.0.1:8080/api/health
# {"status":"ok","engine":{"connected":true}}
```

## 3. UI

> Full documentation: [sindrava.bzy-logics.com](https://sindrava.bzy-logics.com)

```bash
cd sindrava
npm install
npm run dev                  # http://localhost:5173
```

The dev server proxies `/api` (REST and the WebSocket upgrade) to `http://127.0.0.1:8080`. Open `http://localhost:5173`; the header should show the connection as **live**.

## Production layout

For production, build the UI and let ratavyr serve it, so the UI and API share one origin:

```bash
cd sindrava && npm run build                 # writes dist/
cd ../ratavyr
MOTIONCORE_WEB_DIR=../sindrava/dist \
MOTIONCORE_API_TOKEN=... MOTIONCORE_OPERATOR_TOKEN=... MOTIONCORE_VIEWER_TOKEN=... \
MOTIONCORE_API_ADDR=0.0.0.0:8080 \
  ./ratavyr
```

ratavyr serves the directory at `/` with SPA fallback and security headers. Any other static host also works, as long as `/api` — including the WebSocket upgrade on `/api/ws` — is reverse-proxied to ratavyr on the same origin.

ratavyr speaks plain HTTP. Across a network, put it behind a TLS-terminating reverse proxy. The engine's gRPC link has no authentication and must stay on a Unix socket or loopback.

## Version compatibility

| Link | Rule |
|---|---|
| ratavyr ↔ engine | Engine `api_version` must have the same `0.MINOR` as the contract ratavyr was built against (`0.1.x`). Otherwise ratavyr reports the reason in `/api/info` → `engine_error` and returns `503 engine_unavailable`. |
| sindrava ↔ ratavyr | REST API v1. sindrava also works with ratavyr < 0.3 (no `/api/session` or `/api/ws-tickets`): permissions are unknown and the WebSocket falls back to `?access_token=`. |
| engine ↔ MCU | Protocol version must match exactly; the engine refuses the MCU otherwise. |

More in [Contracts & versioning](/internals/versioning).
