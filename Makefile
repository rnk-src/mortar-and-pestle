# `make check` is the single quality gate. CI runs the same command.
# No target uploads firmware or opens a serial port.

HOST := uv run --locked --directory host
CPP_FILES := $(shell find firmware -name .pio -prune -o \( -name '*.h' -o -name '*.hpp' -o -name '*.c' -o -name '*.cc' -o -name '*.cpp' \) -print)

.PHONY: check format py-format-check py-lint py-types py-test cpp-format-check cpp-test cpp-build

check: py-format-check py-lint py-types py-test cpp-format-check cpp-test cpp-build
	@echo "check: all passed"

py-format-check:
	$(HOST) ruff format --check .

py-lint:
	$(HOST) ruff check .

py-types:
	$(HOST) mypy

py-test:
	$(HOST) pytest -q

cpp-format-check:
	@test -n "$(CPP_FILES)" || { echo "cpp-format-check: no C++ files found"; exit 1; }
	clang-format --dry-run --Werror $(CPP_FILES)

cpp-test:
	pio test -d firmware -e native

# Compile only. This target does not upload.
cpp-build:
	pio run -d firmware -e esp32dev

format:
	$(HOST) ruff format .
	$(HOST) ruff check --fix .
	@test -n "$(CPP_FILES)" || { echo "format: no C++ files found"; exit 1; }
	clang-format -i $(CPP_FILES)
