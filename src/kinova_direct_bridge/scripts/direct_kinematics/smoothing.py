"""Rate limiting for jog commands; zero commands bypass the ramp."""
import math


def ramp_vector(previous, target, max_rate, dt):
    """Move towards target by at most max_rate * dt in vector norm."""
    delta = tuple(b - a for a, b in zip(previous, target))
    length = math.sqrt(sum(value * value for value in delta))
    if length == 0.0:
        return tuple(target)
    scale = min(1.0, max_rate * dt / length)
    return tuple(a + scale * change for a, change in zip(previous, delta))
