# Mortar & Pestle

Tier: portfolio

An assistive teleoperated mortar and pestle. Read [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md) before you change a module.

## Check command
- Run `make check` before you claim a task is done. Show the output.
- `make check` runs format check, lint, type check, Python tests, C++ native tests and an ESP32 compile.
- Run `make format` to fix formatting.

## Hardware safety
- Never flash firmware. Never run `pio run -t upload`.
- Never open or write to a serial port without asking first.
- Never actuate a real robot without asking first.
- `pio run -e esp32dev` compiles only. It is safe.

## Conventions
- Use these domain terms exactly: Leader, Follower, Leader Sample, Grind Intent, Follower Command, Mode, Safety Envelope.
- Write a failing test first. Test each module through its interface only.
- Inject the clock. Do not call `time.time()` inside modules.
- Put hardware-free C++ in `firmware/lib/core/`. Put Arduino adapters in `firmware/src/`.
- Do not add dependencies without asking.

## Layout
- `protocol/`: message schema, golden vectors.
- `firmware/`: PlatformIO project for the Leader (ESP32).
- `host/`: Python package `mortar_pestle` (uv, Python 3.12).
- `sim/assets/`: robot models.
- `cad/`: build123d scripts.
- `recordings/`: sample sessions for replay tests.
- `docs/`: architecture and specs.
