---
layout: home

hero:
  name: MotionCore
  text: Robotics motion control, from config file to step pulse
  tagline: A plugin-first, hardware-independent motion engine in Rust, with shared MCU firmware for STM32F411 and RP2040, a simulator, a firmware builder, a CLI, and an optional Go gateway and web UI.
  image:
    src: /logo.svg
    alt: MotionCore
  actions:
    - theme: brand
      text: Quick start
      link: /guide/quick-start
    - theme: alt
      text: How it fits together
      link: /guide/concepts
    - theme: alt
      text: Engine API
      link: /reference/engine-api

features:
  - title: One firmware core, many boards
    details: Protocol, segment execution, homing, watchdog and fault handling are shared by every MCU target and the simulator. A board only supplies pins, timing and transport.
    link: /internals/architecture
  - title: Safety first
    details: The MCU enforces soft limits, endstops, the step-rate ceiling and a communication watchdog on its own, and latches faults that clear only once the hazard is gone.
    link: /guide/safety
  - title: Deterministic step output
    details: On RP2040, PIO state machines time every pulse with guaranteed pulse widths and direction setup/hold, and no catch-up bursts.
    link: /hardware/rp2040
  - title: Reproducible firmware
    details: Builds are pinned by a lockfile, artifacts are checksummed, and the running build is verified over the wire.
    link: /hardware/firmware
  - title: Simulation is the real thing
    details: The virtual MCU runs the same firmware runtime as the hardware, so everything you do in simulation exercises the real code paths.
    link: /guide/quick-start
  - title: Headless first
    details: The engine and CLI work on their own. The REST/WebSocket gateway (ratavyr) and the browser UI (sindrava) are optional, replaceable clients.
    link: /guide/full-stack
---

::: warning Status: v0.2 development
MotionCore runs end-to-end in simulation. The STM32F411 and RP2040 firmware compile and are host-tested, but have **not** been validated on hardware. Nothing here is a certified safety system — always use an independent hardware E-stop that removes driver power.
:::
