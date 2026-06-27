"""Realism perturbations for MuJoCo micromouse simulation.

Features (each independently toggleable, default OFF):
  1. Surface bumps      — small Z-force perturbations from a 2D noise field
  2. Friction variation — position-dependent wheel friction multiplier
  3. Friction degradation — mu reduction from accumulated wheel travel (dust)

All features work by modifying MuJoCo writable arrays (geom_friction)
or xfrc_applied before each mj_step() call.
"""

from dataclasses import dataclass
from typing import Optional

import numpy as np


# ═══════════════════════════════════════════════════════════════════════════════
# Noise field: precomputed 2D grid with bilinear interpolation
# ═══════════════════════════════════════════════════════════════════════════════

class NoiseField2D:
    """Precomputed uniform-noise 2D grid with bilinear interpolation."""

    def __init__(self, x_range: tuple[float, float],
                 y_range: tuple[float, float],
                 cell_size: float, seed: int):
        self.x0 = x_range[0]
        self.y0 = y_range[0]
        self.cell_size = float(cell_size)
        self.nx = max(2, int((x_range[1] - x_range[0]) / cell_size) + 1)
        self.ny = max(2, int((y_range[1] - y_range[0]) / cell_size) + 1)
        rng = np.random.RandomState(seed)
        self.grid = rng.uniform(-1.0, 1.0, (self.ny, self.nx))

    def sample(self, x: float, y: float) -> float:
        """Bilinear interpolation. Clamps to grid edges."""
        fx = (x - self.x0) / self.cell_size
        fy = (y - self.y0) / self.cell_size
        ix = int(np.floor(fx))
        iy = int(np.floor(fy))
        ix = max(0, min(self.nx - 2, ix))
        iy = max(0, min(self.ny - 2, iy))
        tx = fx - ix
        ty = fy - iy
        g = self.grid
        return float(
            (1 - ty) * ((1 - tx) * g[iy, ix]     + tx * g[iy, ix + 1]) +
            ty       * ((1 - tx) * g[iy + 1, ix] + tx * g[iy + 1, ix + 1])
        )


# ═══════════════════════════════════════════════════════════════════════════════
# Configuration
# ═══════════════════════════════════════════════════════════════════════════════

@dataclass
class RealismConfig:
    """All realism features default to OFF.  Tune via SimParams in workbench."""

    # 1. Surface bumps — tiny Z-force perturbations
    surface_bumps_enabled: bool = False
    bump_amplitude: float = 0.15         # N, Z-force amplitude
    bump_cell_size: float = 0.03         # m, noise grid resolution
    bump_seed: int = 101

    # 2. Non-uniform friction — small position-dependent variation
    friction_var_enabled: bool = False
    friction_var_amplitude: float = 0.08  # fraction of base mu (+-8%)
    friction_var_cell_size: float = 0.10  # m, smoother/larger patches
    friction_var_seed: int = 202

    # 3. Friction degradation — wheels accumulate dust
    friction_degrade_enabled: bool = False
    degrade_rate: float = 0.004           # fraction per metre travelled
    degrade_max_fraction: float = 0.20    # max mu reduction after long run

    @property
    def enabled(self) -> bool:
        return (self.surface_bumps_enabled or self.friction_var_enabled or
                self.friction_degrade_enabled)


# ═══════════════════════════════════════════════════════════════════════════════
# Realism Manager
# ═══════════════════════════════════════════════════════════════════════════════

class RealismManager:
    """Orchestrates realism perturbations applied before/after each mj_step()."""

    def __init__(self, config: RealismConfig):
        self.cfg = config
        self._total_distance: float = 0.0
        self._noise_fields: dict[str, NoiseField2D] = {}

    def set_track_bounds(self, x_min: float, x_max: float,
                         y_min: float, y_max: float):
        """Initialise noise fields covering the track bounding box + margin."""
        m = 0.3
        bounds_x = (x_min - m, x_max + m)
        bounds_y = (y_min - m, y_max + m)

        if self.cfg.surface_bumps_enabled:
            self._noise_fields['bump'] = NoiseField2D(
                bounds_x, bounds_y, self.cfg.bump_cell_size, self.cfg.bump_seed)
        if self.cfg.friction_var_enabled:
            self._noise_fields['friction'] = NoiseField2D(
                bounds_x, bounds_y, self.cfg.friction_var_cell_size,
                self.cfg.friction_var_seed)

    def pre_step(self, model, data, chassis_pos_x: float, chassis_pos_y: float,
                 wheel_L_vel: float, wheel_R_vel: float,
                 v_fwd: float, yaw_rate: float,
                 wheel_L_id: int, wheel_R_id: int,
                 base_mu_L: float, base_mu_R: float,
                 base_radius: float,
                 track_half_width: float = 0.045) -> float:
        """Modify geom_friction before mj_step(). Returns bump Z-force (N)."""
        if not self.cfg.enabled:
            return 0.0

        bump_z = 0.0

        # ── 1. Surface bumps ──
        if self.cfg.surface_bumps_enabled and 'bump' in self._noise_fields:
            noise = self._noise_fields['bump'].sample(chassis_pos_x, chassis_pos_y)
            bump_z = noise * self.cfg.bump_amplitude

        # ── 2+3. Friction spatial variation + degradation ──
        if self.cfg.friction_var_enabled or self.cfg.friction_degrade_enabled:
            for wheel_id, base_mu in [(wheel_L_id, base_mu_L),
                                       (wheel_R_id, base_mu_R)]:
                multiplier = 1.0

                if self.cfg.friction_var_enabled and 'friction' in self._noise_fields:
                    wx = data.geom_xpos[wheel_id, 0]
                    wy = data.geom_xpos[wheel_id, 1]
                    noise = self._noise_fields['friction'].sample(wx, wy)
                    multiplier *= 1.0 + noise * self.cfg.friction_var_amplitude

                if self.cfg.friction_degrade_enabled:
                    degrade = min(self.cfg.degrade_rate * self._total_distance,
                                  self.cfg.degrade_max_fraction)
                    multiplier *= 1.0 - degrade

                model.geom_friction[wheel_id, 0] = base_mu * max(0.65, multiplier)

        return bump_z

    def post_step(self, wheel_L_vel: float, wheel_R_vel: float,
                  base_radius: float, dt: float):
        """Accumulate wheel travel distance. Called AFTER mj_step()."""
        if self.cfg.friction_degrade_enabled:
            avg_omega = 0.5 * (abs(wheel_L_vel) + abs(wheel_R_vel))
            self._total_distance += avg_omega * base_radius * dt

    def reset(self):
        """Reset all accumulators (called on simulation reset)."""
        self._total_distance = 0.0
