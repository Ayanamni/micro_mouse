"""Shared actuation delay model."""

from __future__ import annotations

from collections import deque


class ActuationDelayBuffer:
    """Fixed-step command delay with no off-by-one ambiguity."""

    def __init__(self, delay_us: float, dt: float):
        self.delay_us = float(delay_us)
        self.dt = float(dt)
        self.steps = max(0, int(round(self.delay_us * 1e-6 / self.dt)))
        self._queue: deque[tuple[float, float]] = deque()
        self.reset()

    @property
    def enabled(self) -> bool:
        return self.steps > 0

    def reset(self) -> None:
        self._queue.clear()
        for _ in range(self.steps):
            self._queue.append((0.0, 0.0))

    def apply(self, u_left: float, u_right: float) -> tuple[float, float]:
        if self.steps == 0:
            return float(u_left), float(u_right)
        self._queue.append((float(u_left), float(u_right)))
        return self._queue.popleft()
