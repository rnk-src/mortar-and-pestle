"""Recorder: write and read session logs, one JSON object per tick (JSON Lines)."""

import json
from collections.abc import Iterable
from dataclasses import asdict, dataclass
from pathlib import Path
from types import TracebackType
from typing import IO, Any, Self

from mortar_pestle.types import (
    Direction,
    FollowerCommand,
    GrindIntent,
    LeaderSample,
    Pattern,
)

_DECIMALS = 4


@dataclass(frozen=True)
class TickRecord:
    """What one pipeline tick saw and decided. sample is None if no valid sample arrived."""

    t_us: int
    sample: LeaderSample | None
    intent: GrindIntent
    command: FollowerCommand


class Recorder:
    """Writes TickRecords to a JSON Lines file. Use it as a context manager."""

    def __init__(self, path: Path) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        self._file: IO[str] = path.open("w", encoding="utf-8")

    def write(self, record: TickRecord) -> None:
        self._file.write(json.dumps(_rounded(asdict(record)), separators=(",", ":")) + "\n")

    def close(self) -> None:
        self._file.close()

    def __enter__(self) -> Self:
        return self

    def __exit__(
        self,
        exc_type: type[BaseException] | None,
        exc: BaseException | None,
        traceback: TracebackType | None,
    ) -> None:
        self.close()


def read_recording(path: Path) -> list[TickRecord]:
    """Read every TickRecord of a session log."""
    with path.open(encoding="utf-8") as file:
        return [_record_from_json(json.loads(line)) for line in file if line.strip()]


def leader_samples(records: Iterable[TickRecord]) -> list[LeaderSample | None]:
    """Return the Leader Sample stream of a recording, one entry per tick."""
    return [record.sample for record in records]


def _rounded(value: Any) -> Any:
    if isinstance(value, float):
        return round(value, _DECIMALS)
    if isinstance(value, dict):
        return {key: _rounded(item) for key, item in value.items()}
    if isinstance(value, list | tuple):
        return [_rounded(item) for item in value]
    return value


def _record_from_json(data: dict[str, Any]) -> TickRecord:
    sample = data["sample"]
    intent = data["intent"]
    return TickRecord(
        t_us=data["t_us"],
        sample=None
        if sample is None
        else LeaderSample(
            timestamp_us=sample["timestamp_us"],
            quat_wxyz=tuple(sample["quat_wxyz"]),
            pressure_n=sample["pressure_n"],
            x_mm=sample["x_mm"],
            y_mm=sample["y_mm"],
            contact=sample["contact"],
        ),
        intent=GrindIntent(
            pattern=Pattern(intent["pattern"]),
            rate_hz=intent["rate_hz"],
            radius_mm=intent["radius_mm"],
            direction=Direction(intent["direction"]),
            requested_pressure_n=intent["requested_pressure_n"],
        ),
        command=FollowerCommand(**data["command"]),
    )
