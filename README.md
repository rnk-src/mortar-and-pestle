# Mortar & Pestle

[![check](https://github.com/rnk-src/mortar-and-pestle/actions/workflows/check.yml/badge.svg)](https://github.com/rnk-src/mortar-and-pestle/actions/workflows/check.yml)

An assistive teleoperated mortar and pestle for people with tremor or weak grip.

The user moves a light dummy pestle in a sensed dummy mortar (the **Leader**).
A robot arm, an SO-101 (the **Follower**), grinds with a real pestle.
The system reads the user's intent, removes tremor, and applies the force the user cannot.

See [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md) for modules and seams.

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

Requirements: [uv](https://docs.astral.sh/uv/), `make` and a host C++17 compiler (g++ or clang) for the native tests.

```sh
uv tool install platformio==6.2.0
uv tool install clang-format==23.1.5
make check
```

`make check` runs the format check, lint, type check, Python tests, C++ native tests and an ESP32 compile.
It never uploads firmware.
