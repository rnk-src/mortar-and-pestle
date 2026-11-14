import pybullet as p
import pytest

from mortar_pestle.sim.world import query


class Flaky:
    """A PyBullet-like call that fails a number of times before it answers."""

    def __init__(self, failures: list[str], answer: object = "answer") -> None:
        self._failures = list(failures)
        self._answer = answer
        self.calls = 0

    def __call__(self, *args: object, **kwargs: object) -> object:
        self.calls += 1
        if self._failures:
            if self._failures.pop(0) == "raise":
                raise p.error("failed.")
            return None
        return self._answer


def test_query_returns_the_answer_of_a_healthy_call() -> None:
    call = Flaky([])

    assert query(call, 1, key=2) == "answer"
    assert call.calls == 1


def test_query_retries_a_call_that_raises_or_returns_nothing() -> None:
    call = Flaky(["raise", "none", "raise"])

    assert query(call) == "answer"
    assert call.calls == 4


def test_query_gives_up_after_five_attempts_and_raises() -> None:
    call = Flaky(["raise"] * 10)

    with pytest.raises(p.error):
        query(call)
    assert call.calls == 5


def test_query_returns_none_when_the_call_never_answers() -> None:
    call = Flaky(["none"] * 10)

    assert query(call) is None
    assert call.calls == 5
