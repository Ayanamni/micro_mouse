"""Finite-width line sensor — realistic photodiode array model.

Real micromouse: 8-16 photodiodes spanning ±12mm to ±20mm.
White line on dark track: 20mm wide line centered on track centerline.

Key constraint: if the line moves outside the sensor span, the car
loses the line entirely — just like on a real track.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

import numpy as np

from micromouse_sim.environment.track import TrackCenterline


@dataclass
class LineSensorConfig:
    """Configuration for a finite-width photodiode array line sensor."""
    n_leds: int = 8            # Number of photodiodes
    half_span: float = 0.012   # Half the total sensing span (m), ±12mm
    line_width: float = 0.020  # White line width (m), standard 20mm
    fwd_offset: float = 0.040  # Distance ahead of wheel axle (m)
    detect_threshold: float = 0.15  # ADC threshold for "line detected"


class LineSensor:
    """Finite-width photodiode array line sensor.

    The sensor has N photodiodes evenly spaced along the vehicle body Y axis,
    centered on the sensor mount point (fwd_offset ahead of the axle).

    Each photodiode returns an ADC value 0-1 based on overlap with the
    white line, plus noise. A centroid algorithm estimates lateral position.

    If NO photodiode exceeds the detection threshold, the line is LOST.
    """

    def __init__(self, config: LineSensorConfig = LineSensorConfig()):
        self.cfg = config
        # LED positions in body Y (m), centered on sensor mount
        self._led_y = np.linspace(
            -config.half_span, config.half_span, config.n_leds
        )
        self._rng = np.random.RandomState(0)

    # ── Public API ──────────────────────────────────────────────────────

    def read(
        self,
        pos_2d: np.ndarray,   # [x, y] of vehicle reference (world frame)
        yaw: float,           # vehicle heading (rad)
        track: TrackCenterline,
    ) -> "LineSensorReading":
        """Read the line sensor at the given vehicle pose.

        Returns a LineSensorReading with:
          - lateral_error: estimated lateral offset (m), None if line lost
          - line_detected: bool
          - curvature: track curvature at sensor position (None if lost)
          - adc: raw ADC values (0-1) for each photodiode
        """
        # ── Sensor position in world frame ──
        cos_y = np.cos(yaw)
        sin_y = np.sin(yaw)
        sensor_x = pos_2d[0] + self.cfg.fwd_offset * cos_y
        sensor_y = pos_2d[1] + self.cfg.fwd_offset * sin_y

        # ── Project sensor onto track centerline ──
        try:
            s_pos, sensor_lat, heading = track.project(
                np.array([sensor_x, sensor_y])
            )
            curvature = track.curvature_at(s_pos)
        except Exception:
            return LineSensorReading(
                lateral_error=None,
                line_detected=False,
                curvature=None,
                adc=np.zeros(self.cfg.n_leds),
                s_pos=None,
            )

        # ── Each photodiode's lateral offset from centerline ──
        # Photodiodes are at body Y positions relative to sensor
        # Their world-frame lateral offset from the centerline:
        #   led_lat = sensor_lat + led_y * cos(small_angle)
        # Approximate: cos(heading) ≈ 1 for small heading offsets
        heading_err = np.arctan2(np.sin(yaw - heading), np.cos(yaw - heading))
        body_y_to_track_lat = np.cos(heading_err)
        led_lat_offsets = sensor_lat + self._led_y * body_y_to_track_lat

        # ── ADC computation ──
        # White line is centered on track centerline, width = line_width
        # ADC = 1.0 when LED is fully over line, 0.0 when fully off
        half_lw = self.cfg.line_width / 2.0
        overlap = (half_lw - np.abs(led_lat_offsets)) / half_lw
        adc_clean = np.clip(overlap, 0.0, 1.0)

        # Add sensor noise (shot noise + ADC quantization)
        adc_noisy = np.clip(
            adc_clean + self._rng.randn(self.cfg.n_leds) * 0.02, 0.0, 1.0
        )

        # ── Line detection ──
        line_detected = np.any(adc_noisy > self.cfg.detect_threshold)

        # ── Centroid estimate of lateral position ──
        if line_detected:
            # Weighted average of LED body-frame Y positions where line is seen.
            # The line position in the sensor's body frame tells us the lateral offset:
            #   lateral_error = -(line_body_y)
            # (positive lateral_error = car left of track centerline)
            active = adc_noisy > self.cfg.detect_threshold
            weights = adc_noisy[active]
            positions = self._led_y[active]  # body-frame Y, NOT world-frame
            line_body_y = float(np.sum(weights * positions) / (np.sum(weights) + 1e-9))
            lateral_est = -line_body_y
        else:
            lateral_est = None

        return LineSensorReading(
            lateral_error=lateral_est,
            line_detected=line_detected,
            curvature=curvature if line_detected else None,
            adc=adc_noisy,
            s_pos=s_pos,
        )


@dataclass
class LineSensorReading:
    """Output of a single line sensor reading."""
    lateral_error: Optional[float]  # m, estimated lateral offset (None = lost)
    line_detected: bool             # True if at least one LED sees the line
    curvature: Optional[float]      # 1/m, track curvature at sensor (None = lost)
    adc: np.ndarray                 # Raw ADC values (0-1) for each photodiode
    s_pos: Optional[float] = None   # Arc-length position on track (debug)
