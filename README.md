# Mortar & Pestle

[![check](https://github.com/rnk-src/mortar-and-pestle/actions/workflows/check.yml/badge.svg)](https://github.com/rnk-src/mortar-and-pestle/actions/workflows/check.yml)

An assistive teleoperated mortar and pestle for people with tremor or weak grip.

The user moves a light dummy pestle in a sensed dummy mortar (the **Leader**).
A robot arm, an SO-101 (the **Follower**), grinds with a real pestle.
The system reads the user's intent, removes tremor, and applies the force the user cannot.

## How it works

```
Leader (ESP32, C++)                 Host (Python)                          Follower
IMU + load-cell plate  --serial-->  interpreter --> safety governor  -->  SO-101 (sim or real)
   Leader Samples                   Grind Intent    Follower Command
```

- The Leader emits calibrated **Leader Samples** at 100 Hz.
- The interpreter turns them into a **Grind Intent** (pattern, rate, radius, direction, pressure) at 10 Hz.
- A **Mode** maps each Grind Intent to a **Follower Command**.
- The safety governor clamps each Follower Command to a **Safety Envelope**.
- If Leader data goes stale, the Follower releases pressure, lifts and holds.

See [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md) for modules and seams.

## Status

Milestone 1 of 10: repo skeleton. No hardware is needed yet.

## Layout

| Path | Content |
|---|---|
| `protocol/` | Message schema, framing spec, golden vectors |
| `firmware/` | PlatformIO project for the Leader; `lib/core` is hardware-free and tested natively |
| `host/` | Python package: interpreter, safety governor, recorder, adapters, CLI |
| `sim/assets/` | SO-101 model files |
| `cad/` | Parametric CAD scripts (build123d) |
| `recordings/` | Sample sessions for replay tests |
| `docs/` | Architecture and specs |

## Quick start

Requirements: [uv](https://docs.astral.sh/uv/) and `make`.

```sh
uv tool install platformio
uv tool install clang-format==23.1.5
make check
```

`make check` runs the format check, lint, type check, Python tests, C++ native tests and an ESP32 compile.
It never uploads firmware.
