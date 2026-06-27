"""Tire deformation model for silicone+foam micromouse tires.

Does NOT change MuJoCo geometry. Instead, models how tire deformation
affects odometry — injects systematic and random errors into the
wheel displacement measurements fed to the localization module.

Physical basis:
  - Foam inner tire compresses under load → effective rolling radius changes
  - Silicone skin has local slip at contact patch → velocity noise
  - Hysteresis in foam → slight energy loss (already captured by rolling friction)
"""

from dataclasses import dataclass

import numpy as np


@dataclass
class TireDeformationParams:
    """Parameters governing tire deformation effects on odometry."""

    # Nominal wheel radius (m)
    r_nom: float = 0.0105

    # Deformation stiffness: radius decreases linearly with normal force
    # r_eff = r_nom * (1 - k * F_normal)
    # For foam tire: k ≈ 0.01-0.03 per N → ~0.1mm radius change at 5N
    deformation_k: float = 2e-5  # m/N → 0.1mm at 5N

    # Scale factor error from tire deformation (systematic, per-wheel)
    # Captures: uneven tire wear, foam density variation, mounting asymmetry
    scale_error_std: float = 0.002  # 0.2% radius uncertainty

    # Slip-induced velocity noise (random, per step)
    # Captures: local silicone slip, foam hysteresis, contact patch migration
    slip_noise_std: float = 0.0015  # 0.15% per-step velocity noise

    # Minimum normal force for contact (N) — below this, wheel may lose contact
    min_contact_force: float = 0.01


class TireDeformationModel:
    """
    Models odometry errors from tire deformation.

    Two error sources:
      1. Systematic scale error: effective radius ≠ nominal radius
         → accumulates as distance-proportional error
      2. Random slip noise: per-step velocity perturbation
         → accumulates as random-walk error (∼√t)
    """

    def __init__(self, params: TireDeformationParams | None = None):
        self.p = params or TireDeformationParams()
        # Per-wheel scale factors (fixed for a simulation run)
        self._scale_L = 1.0 + np.random.randn() * self.p.scale_error_std
        self._scale_R = 1.0 + np.random.randn() * self.p.scale_error_std

    def get_effective_radius(self, normal_force: float) -> float:
        """
        Effective rolling radius under given normal force.

        Foam compresses: r_eff < r_nom under load.
        """
        if normal_force < self.p.min_contact_force:
            return self.p.r_nom  # no meaningful contact
        return self.p.r_nom * (1.0 - self.p.deformation_k * normal_force)

    def compute_odometry_delta(
        self,
        wheel_angle_delta: float,    # rad, wheel joint rotation this step
        normal_force: float,          # N, approximate normal load on this wheel
        side: str = "L",
    ) -> float:
        """
        Convert wheel angle delta to distance traveled (m), with errors.

        Args:
            wheel_angle_delta: Change in wheel joint angle this step (rad).
            normal_force: Approximate normal force on this wheel (N).
            side: "L" or "R" — selects which wheel's scale factor to use.

        Returns:
            Distance traveled by this wheel's contact patch (m), with errors.
        """
        p = self.p

        # Effective radius under load
        r_eff = self.get_effective_radius(normal_force)

        # Apply wheel-specific scale factor
        scale = self._scale_L if side == "L" else self._scale_R
        r_eff *= scale

        # Base distance
        distance = r_eff * wheel_angle_delta

        # Slip noise: proportional to distance traveled this step
        slip = np.random.randn() * p.slip_noise_std * abs(distance)
        distance += slip

        return distance

    def reset(self, seed: int | None = None) -> None:
        """Resample wheel scale factors (for simulation reset)."""
        if seed is not None:
            np.random.seed(seed)
        self._scale_L = 1.0 + np.random.randn() * self.p.scale_error_std
        self._scale_R = 1.0 + np.random.randn() * self.p.scale_error_std
