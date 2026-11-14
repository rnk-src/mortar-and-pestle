# Mortar & Pestle

[![check](https://github.com/rnk-src/mortar-and-pestle/actions/workflows/check.yml/badge.svg)](https://github.com/rnk-src/mortar-and-pestle/actions/workflows/check.yml)

An assistive teleoperated mortar and pestle for people with tremor or weak grip.

The user moves a light dummy pestle in a sensed dummy mortar (the **Leader**).
A robot arm, an SO-101 (the **Follower**), grinds with a real pestle.
The system reads the user's intent, removes tremor, and applies the force the user cannot.

See [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md) for modules and seams.

## Try it

```sh
cd host
uv run mortar-pestle sim          # you move the Leader pestle with the mouse
uv run mortar-pestle sim --auto   # a scripted hand moves it
```

The blue pestle is the Leader. The orange pestle on the SO-101 arm is the Follower.

- Move the mouse over the blue mortar. The pestle follows the cursor.
- Hold the left mouse button, or tap SPACE, to press down.
- The arm follows your pestle: circles, back-and-forth strokes, pounding at any point.
- Use the up and down arrows to push harder or softer.
- The two gauges between the mortars show the Leader force (blue) and the Follower force (orange).
- The terminal shows a live readout: forces, the estimated stroke and the simulation speed.

There are two Modes:

- `--mode mirror` (default): the Follower follows the Leader's position and force directly.
  A low-pass filter removes tremor. The delay is about 70 ms.
- `--mode interpret`: the Follower estimates the stroke (pattern, rate, radius, direction,
  pressure) over a two-second window, then executes a clean stroke. It knows circular strokes
  and pounding at one point.

Other commands:

```sh
uv run mortar-pestle replay ../recordings/circular_1p5hz_30mm.jsonl --gui
uv run mortar-pestle sim --auto --record session.jsonl
```

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
