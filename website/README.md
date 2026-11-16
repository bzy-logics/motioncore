# motioncore documentation site

Source of <https://motioncore.bzy-logics.com>, built with [VitePress](https://vitepress.dev). Deployed to GitHub Pages by [`.github/workflows/docs.yml`](../.github/workflows/docs.yml) on every push to `main` that touches `website/`.

```sh
cd website
npm install
npm run dev        # http://localhost:5173 with hot reload
npm run build      # static site in .vitepress/dist/; fails on dead internal links
npm run preview    # serve the built site
```

**Windows:** build from a path with an uppercase drive letter (`cd D:\…\website`). From `d:\…`, VitePress fails while rendering with `Cannot read properties of undefined (reading 'imports')`.

## Sibling sites

| Site | Covers |
|---|---|
| motioncore.bzy-logics.com (this) | Engine, firmware, hardware, CLI, gRPC API, internals |
| [ratavyr.bzy-logics.com](https://ratavyr.bzy-logics.com) | Gateway: REST/WebSocket API, auth, deployment |
| [sindrava.bzy-logics.com](https://sindrava.bzy-logics.com) | Operator UI |

Links between the sites are absolute URLs.

## Keeping it accurate

| Page | Source of truth |
|---|---|
| `reference/engine-api.md` | `crates/engine-api/proto/motioncore/engine/v1/engine.proto` |
| `reference/cli.md` | `crates/cli/src/main.rs` |
| `reference/config-schema.md` | `crates/device-graph/src/lib.rs`, `boards.rs`, `docs/robot-config.md` |
| `guide/robot-states.md` | `crates/safety/src/lib.rs` |
| `internals/mcu-protocol.md` | `crates/protocol/src` |
| `hardware/rp2040.md`, `hardware/stm32f411.md` | `docs/rp2040.md`, `docs/hardware-bringup.md` |

A board stays "unvalidated" until its validation procedure has actually been carried out on hardware.

## Deployment setup (once)

1. Repository **Settings → Pages → Source: GitHub Actions**.
2. Same page: **Custom domain** `motioncore.bzy-logics.com`, then **Enforce HTTPS** once the certificate is issued.
3. DNS: `CNAME motioncore → bzy-logics.github.io`.
