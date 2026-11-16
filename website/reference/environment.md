# Environment variables

## motioncore (engine and CLI)

| Variable | Default | Meaning |
|---|---|---|
| `MOTIONCORE_ENGINE_ADDR` | Linux: `unix:///run/motioncore/engine.sock`; elsewhere `tcp://127.0.0.1:7600` | Engine address for client commands; same as `--engine`. Also the default listen address for `motioncore start` |
| `RUST_LOG` | `info` | Engine log filter (`tracing` `EnvFilter` syntax), e.g. `debug` or `motioncore_core=debug` |

The gateway and the UI have their own settings:

- [ratavyr environment variables](https://ratavyr.bzy-logics.com/reference/environment)
- [sindrava configuration](https://sindrava.bzy-logics.com/guide/getting-started)
