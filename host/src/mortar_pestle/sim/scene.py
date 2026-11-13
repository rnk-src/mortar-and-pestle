"""Scene layout and a scripted hand for the simulated Leader."""

from collections.abc import Sequence

from mortar_pestle.leaders import synthetic_samples
from mortar_pestle.sim.leader import Hand, HandTarget
from mortar_pestle.synthetic import StrokeSpec

# The arm base is at the world origin. Its mortar sits in front of it.
# The Leader mortar sits beside the Follower mortar, clear of the arm.
FOLLOWER_MORTAR_XY_M = (0.19, 0.0)
LEADER_MORTAR_XY_M = (0.19, -0.20)
LEADER_COLOUR = (0.2, 0.5, 0.9)
FOLLOWER_COLOUR = (0.85, 0.45, 0.2)


def scripted_hand(specs: Sequence[StrokeSpec], loop: bool = False) -> Hand:
    """Return a hand that plays stroke segments, one tick per call, then hovers."""
    samples = synthetic_samples(specs)
    index = 0

    def hand() -> HandTarget:
        nonlocal index
        if index >= len(samples):
            if not loop:
                return HandTarget(0.0, 0.0, False, 0.0)
            index = 0
        sample = samples[index]
        index += 1
        return HandTarget(sample.x_mm, sample.y_mm, sample.contact, sample.pressure_n)

    return hand
