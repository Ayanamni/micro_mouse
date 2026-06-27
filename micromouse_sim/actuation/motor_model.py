"""Motor model for Maxon ECX SPEED 13 L 18V with 4:1 gear reduction.

Models the full voltage→torque chain:
  u_norm → U_eff → I (limited by back-EMF and R) → τ_motor → τ_wheel
  + cogging torque + gear noise
"""

from dataclasses import dataclass
from typing import Tuple

import numpy as np


@dataclass
class MotorParams:
    """电机+减速箱参数，基于 Maxon ECX SPEED 13 L 18V 真实数据表。"""

    # ==== 电机本体：Maxon ECX SPEED 13 L, 18V 绕组 ====
    R: float = 0.344              # 相间电阻 (Ω)
    L: float = 1.28e-5            # 相间电感 (H) = 0.0128 mH
    Kt: float = 0.00241           # 扭矩常数 (Nm/A)
    Ke: float = 0.00241           # 反电动势常数 (V/(rad/s))，SI下等于Kt
    V_bus: float = 11.1          # 母线电压 (V) = 3S LiPo 标称 (3×3.7)。满电 12.6V，
                                 #   大电流下垂到 ~10V。电机绕组是 18V 款，但实际供电是
                                 #   3S 电池——电压前馈/动态力矩上限用的是这个供电电压。
    I_cont: float = 3.22          # 持续电流限制 (A)
    I_peak: float = 10.0          # 峰值电流限制 (A)，短时爆发
    rotor_inertia: float = 3.15e-8  # 转子转动惯量 (kg·m²)

    # ==== 减速箱：4:1 行星减速 ====
    gear_ratio: float = 4.0       # 减速比，电机4圈=轮子1圈
    efficiency: float = 0.88      # 传动效率，典型行星减速箱值

    # ==== 扰动（改这些模拟真实电机的不完美）====
    cogging_amplitude: float = 0.0    # 齿槽转矩幅值 (Nm)，非零时每步施加正弦扰动
    cogging_slots: int = 6            # 齿槽数（6对极）
    gear_noise_std: float = 0.0       # 齿轮啮合噪声标准差 (Nm)，非零时每步施加随机扰动

    # ==== 限制 ====
    tau_max_wheel: float = 0.05       # 轮端扭矩硬限制 (Nm)，改这个改变最大加速度


class MotorModel:
    """
    Single motor + gearbox model.

    Converts normalized control signal u ∈ [-1, 1] and wheel angular velocity
    to actual wheel torque, accounting for:
      - Back-EMF voltage drop at speed
      - Ohmic current limit
      - Continuous/peak current limits
      - Gear efficiency
      - Cogging torque ripple
      - Random gear mesh noise
    """

    def __init__(self, params: MotorParams | None = None):
        self.p = params or MotorParams()
        # Per-motor state: accumulated rotor angle for cogging phase
        self._theta_rotor: float = 0.0

    def compute_torque(self, u_norm: float, omega_wheel: float, dt: float) -> float:
        """
        Convert normalized command to actual wheel torque.

        Args:
            u_norm: Normalized control [-1, 1] from controller.
            omega_wheel: Wheel angular velocity (rad/s).
            dt: Timestep (s).

        Returns:
            Wheel torque τ_wheel (Nm), ready to feed to data.ctrl.
        """
        p = self.p

        # 1. Effective voltage
        U_eff = u_norm * p.V_bus

        # 2. Motor speed (reflected through gearbox)
        omega_motor = omega_wheel * p.gear_ratio

        # 3. Back-EMF voltage
        V_bemf = p.Ke * omega_motor

        # 4. Current from Ohm's law: I = (U - V_bemf) / R
        #    inductance L/R time constant = 3.7e-5 s = 37 µs ≈ 2 physics steps
        #    → negligible lag, treat as instantaneous for 50kHz sim
        if abs(U_eff - V_bemf) < 1e-9:
            I = 0.0
        else:
            I = (U_eff - V_bemf) / p.R

        # 5. Current limits (asymmetric — negative allows regenerative braking)
        #    Use peak limit for short bursts; the control algorithm should
        #    respect continuous limits for sustained operation.
        I = float(np.clip(I, -p.I_peak, p.I_peak))

        # 6. Motor electromagnetic torque
        tau_motor = p.Kt * I

        # 7. Reflect through gearbox to wheel
        tau_wheel = tau_motor * p.gear_ratio * p.efficiency

        # 8. Cogging torque (at wheel, based on rotor electrical angle)
        self._theta_rotor += omega_motor * dt
        tau_cog = p.cogging_amplitude * np.sin(
            p.cogging_slots * self._theta_rotor
        )

        # 9. Gear mesh noise (independent per step)
        tau_noise = np.random.randn() * p.gear_noise_std

        # 10. Combine and clamp
        tau_total = tau_wheel + tau_cog + tau_noise
        tau_total = float(np.clip(tau_total, -p.tau_max_wheel, p.tau_max_wheel))

        return tau_total

    def reset(self) -> None:
        """Reset rotor angle accumulator (for simulation reset)."""
        self._theta_rotor = 0.0

    def max_available_torque(self, omega_wheel: float) -> Tuple[float, float]:
        """
        Returns (max_forward, max_reverse) torque at given wheel speed.

        Useful for control algorithms to know actuator limits.
        """
        p = self.p
        omega_motor = omega_wheel * p.gear_ratio
        V_bemf = p.Ke * omega_motor

        # Forward: +V_bus, +I_cont
        I_fwd = float(np.clip((p.V_bus - V_bemf) / p.R, 0.0, p.I_cont))
        tau_fwd = I_fwd * p.Kt * p.gear_ratio * p.efficiency

        # Reverse: -V_bus, -I_cont
        I_rev = float(np.clip((-p.V_bus - V_bemf) / p.R, -p.I_cont, 0.0))
        tau_rev = I_rev * p.Kt * p.gear_ratio * p.efficiency

        return (tau_fwd, tau_rev)
