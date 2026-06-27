"""MuJoCo physics engine wrapper for micromouse simulation.

All tunable physics parameters are defined here as module-level constants.
Import from this module in all scripts — no per-script copies.
"""

from dataclasses import dataclass
import logging
from pathlib import Path
from typing import Optional

import mujoco
import numpy as np

from .realism import RealismConfig, RealismManager

logger = logging.getLogger(__name__)

# ═══════════════════════════════════════════════════════════════════════════════
# ★ 所有可调物理参数统一在这里修改！三个模式共用。
# ═══════════════════════════════════════════════════════════════════════════════

# ── 车体几何 ──
WHEEL_RADIUS   = 0.0105   # 轮子半径 (m)，21mm 直径
TRACK_WIDTH    = 0.090    # 左右轮距 (m)，对应 base.xml wheel pos Y=±0.045

# ── 编码器（仿真中编码器值即弧度，非真实脉冲） ──
PULSES_PER_M   = 1.0 / WHEEL_RADIUS  # ≈95.24 弧度/米（= 1/轮半径）

# ── PhysicsEngine 默认参数 ──
DEFAULT_DOWNFORCE = 5.0    # 下压力 (N)，负压风扇
DEFAULT_SKIRT_MU  = 0.03   # 裙边摩擦系数（无量纲），特氟龙-赛道 ≈0.03-0.08
DEFAULT_SKIRT_R   = 0.04   # 裙边有效摩擦半径 (m)，≈车体半对角线


@dataclass
class SimulationState:
    """Complete snapshot of ground-truth simulation state at one timestep."""
    time: float
    # Chassis state (world frame)
    pos: np.ndarray      # (3,) x, y, z
    quat: np.ndarray     # (4,) w, x, y, z
    linvel: np.ndarray   # (3,) vx, vy, vz (world frame)
    angvel: np.ndarray   # (3,) wx, wy, wz (world frame)
    # Wheel joints
    wheel_L_pos: float   # rad
    wheel_L_vel: float   # rad/s
    wheel_R_pos: float   # rad
    wheel_R_vel: float   # rad/s
    # IMU (raw, in chassis body frame — before noise)
    gyro: np.ndarray     # (3,) rad/s
    accel: np.ndarray    # (3,) m/s²

    @property
    def yaw(self) -> float:
        """Extract yaw angle from quaternion."""
        w, x, y, z = self.quat
        return np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))

    @property
    def forward_velocity(self) -> float:
        """Forward velocity in body frame (m/s)."""
        yaw = self.yaw
        return (self.linvel[0] * np.cos(yaw) +
                self.linvel[1] * np.sin(yaw))

    @property
    def lateral_velocity(self) -> float:
        """Lateral velocity in body frame (m/s)."""
        yaw = self.yaw
        return (-self.linvel[0] * np.sin(yaw) +
                self.linvel[1] * np.cos(yaw))


class PhysicsEngine:
    """
    Wraps MuJoCo model and data for the micromouse simulation.

    External forces (applied every step before mj_step):
      - Downforce: constant -Z force on chassis (negative-pressure fan)
      - Skirt friction (Coulomb): constant force opposing chassis velocity
      - Skirt friction (yaw): constant torque opposing yaw rotation

    MuJoCo handles wheel-ground contact natively via elliptic-cone model.
    """

    def __init__(
        self,
        model_xml: str,
        downforce: float = DEFAULT_DOWNFORCE,
        skirt_mu: float = DEFAULT_SKIRT_MU,
        skirt_R: float = DEFAULT_SKIRT_R,
        realism_config: RealismConfig | None = None,
    ):
        self._model = mujoco.MjModel.from_xml_string(model_xml)
        self._data = mujoco.MjData(self._model)

        self._downforce = downforce
        self._skirt_mu = skirt_mu
        self._skirt_R = skirt_R

        # 预计算 Coulomb 裙边力/力矩 (每步不变，避免重复乘法)
        self._F_skirt = skirt_mu * downforce          # 平移恒力 (N)
        self._tau_skirt = skirt_mu * downforce * skirt_R  # 旋转恒力矩 (Nm)

        # ── Wheel geom IDs & base properties (for realism module) ──
        self._wheel_L_geom_id = mujoco.mj_name2id(
            self._model, mujoco.mjtObj.mjOBJ_GEOM, "wheel_L_geom")
        self._wheel_R_geom_id = mujoco.mj_name2id(
            self._model, mujoco.mjtObj.mjOBJ_GEOM, "wheel_R_geom")
        self._base_friction = self._model.geom_friction.copy()
        self._base_wheel_radius = float(
            self._model.geom_size[self._wheel_L_geom_id, 0])

        # ── Realism manager ──
        self._realism_cfg = realism_config or RealismConfig()
        self.realism = RealismManager(self._realism_cfg)

        # Initialise ctrl to zero
        self._data.ctrl[:] = 0.0

        # Cache body/sensor/actuator IDs for fast step()-time access
        self._chassis_body_id = mujoco.mj_name2id(
            self._model, mujoco.mjtObj.mjOBJ_BODY, "chassis"
        )

        # Sensor addresses — tuples for get_state, scalars for step()
        self._linvel_addr = self._sensor_addr("linvel")     # (start, end) tuple
        self._linvel_adr = self._linvel_addr[0]             # start index for step()
        self._gyro_addr = self._sensor_addr("gyro")
        self._accel_addr = self._sensor_addr("accel")
        self._pos_addr = self._sensor_addr("pos")
        self._quat_addr = self._sensor_addr("quat")
        self._angvel_addr = self._sensor_addr("angvel")
        self._enc_L_pos_addr = self._sensor_addr("enc_L_pos")
        self._enc_L_vel_addr = self._sensor_addr("enc_L_vel")
        self._enc_R_pos_addr = self._sensor_addr("enc_R_pos")
        self._enc_R_vel_addr = self._sensor_addr("enc_R_vel")

        # Actuator IDs
        self._motor_L_id = mujoco.mj_name2id(
            self._model, mujoco.mjtObj.mjOBJ_ACTUATOR, "motor_L"
        )
        self._motor_R_id = mujoco.mj_name2id(
            self._model, mujoco.mjtObj.mjOBJ_ACTUATOR, "motor_R"
        )

        self._n_steps = 0

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    @property
    def model(self) -> mujoco.MjModel:
        return self._model

    @property
    def data(self) -> mujoco.MjData:
        return self._data

    @property
    def timestep(self) -> float:
        return self._model.opt.timestep

    @property
    def time(self) -> float:
        return self._data.time

    @property
    def n_steps(self) -> int:
        return self._n_steps

    def step(self) -> None:
        """Advance physics by one timestep.

        External forces applied before mj_step:
          1. Downforce — constant -Z force on chassis
          2. Coulomb skirt friction — constant force opposing chassis velocity
          3. Coulomb skirt yaw friction — constant torque opposing yaw rate

        MuJoCo ground contact (wheel-floor, skid-floor) is handled natively.
        """
        # ── Chassis velocity (cached address, no name lookup) ──
        adr = self._linvel_adr
        vx = self._data.sensordata[adr]
        vy = self._data.sensordata[adr + 1]
        speed = np.sqrt(vx * vx + vy * vy)

        # ── 1. Downforce ──
        fz = -self._downforce

        # ── 2. Coulomb skirt friction — translation ──
        # F = μ×F_N, direction opposes velocity.
        # Smooth linear ramp below v_thresh to prevent chatter at standstill.
        v_thresh = 0.01  # m/s
        if speed > v_thresh:
            fx = -self._F_skirt * vx / speed
            fy = -self._F_skirt * vy / speed
        else:
            fx = -self._F_skirt * vx / v_thresh
            fy = -self._F_skirt * vy / v_thresh

        self._data.xfrc_applied[self._chassis_body_id, :] = [
            fx, fy, fz, 0.0, 0.0, 0.0
        ]

        # ── 3. Coulomb skirt friction — yaw rotation ──
        # τ = μ×F_N×R_eff, direction opposes yaw rate.
        yaw_rate = self._data.qvel[5]  # chassis yaw DOF (world-frame ω_z)
        ω_thresh = 0.05  # rad/s
        if abs(yaw_rate) > ω_thresh:
            self._data.qfrc_applied[5] = -self._tau_skirt * np.sign(yaw_rate)
        else:
            self._data.qfrc_applied[5] = -self._tau_skirt * yaw_rate / ω_thresh

        # ── 4. Realism perturbations (bumps, friction var, degradation) ──
        if self.realism.cfg.enabled:
            pos_adr = self._pos_addr[0]
            w_L_vel = float(self._data.sensordata[self._enc_L_vel_addr[0]])
            w_R_vel = float(self._data.sensordata[self._enc_R_vel_addr[0]])
            yaw_rate = float(self._data.qvel[5])

            bump_fz = self.realism.pre_step(
                self._model, self._data,
                float(self._data.sensordata[pos_adr]),
                float(self._data.sensordata[pos_adr + 1]),
                w_L_vel, w_R_vel, 0.0, yaw_rate,
                self._wheel_L_geom_id, self._wheel_R_geom_id,
                self._base_friction[self._wheel_L_geom_id, 0],
                self._base_friction[self._wheel_R_geom_id, 0],
                self._base_wheel_radius,
            )
            if bump_fz != 0.0:
                self._data.xfrc_applied[self._chassis_body_id, 2] += bump_fz

        mujoco.mj_step(self._model, self._data)

        # ── Post-step realism (distance accumulation) ──
        if self.realism.cfg.enabled:
            w_L_vel = float(self._data.sensordata[self._enc_L_vel_addr[0]])
            w_R_vel = float(self._data.sensordata[self._enc_R_vel_addr[0]])
            self.realism.post_step(w_L_vel, w_R_vel, self._base_wheel_radius,
                                   self._model.opt.timestep)

        self._n_steps += 1

    def set_control(self, tau_left: float, tau_right: float) -> None:
        """Set wheel torques (Nm). Called before step()."""
        self._data.ctrl[self._motor_L_id] = tau_left
        self._data.ctrl[self._motor_R_id] = tau_right

    def get_state(self) -> SimulationState:
        """Read all sensors and return a complete ground-truth state snapshot."""
        def _scalar(addr) -> float:
            """Read a scalar sensor value."""
            return float(self._data.sensordata[addr])

        def _vec3(start, end) -> np.ndarray:
            """Read a 3D vector sensor value."""
            return self._data.sensordata[start:end].copy()

        return SimulationState(
            time=self._data.time,
            pos=_vec3(*self._pos_addr),
            quat=_vec3(*self._quat_addr),
            linvel=_vec3(*self._linvel_addr),
            angvel=_vec3(*self._angvel_addr),
            wheel_L_pos=_scalar(self._enc_L_pos_addr[0]),
            wheel_L_vel=_scalar(self._enc_L_vel_addr[0]),
            wheel_R_pos=_scalar(self._enc_R_pos_addr[0]),
            wheel_R_vel=_scalar(self._enc_R_vel_addr[0]),
            gyro=_vec3(*self._gyro_addr),
            accel=_vec3(*self._accel_addr),
        )

    def get_chassis_pose_2d(self) -> tuple[float, float, float]:
        """Return (x, y, yaw) in world frame — convenience for 2D planning."""
        pos = self._data.sensordata[self._pos_addr[0]:self._pos_addr[1]]
        quat = self._data.sensordata[self._quat_addr[0]:self._quat_addr[1]]
        w, x, y, z = quat
        yaw = np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
        return float(pos[0]), float(pos[1]), float(yaw)

    def reset(self) -> None:
        """Reset simulation to initial state."""
        mujoco.mj_resetData(self._model, self._data)
        # Restore base friction/size (realism module may have modified them)
        self._model.geom_friction[:] = self._base_friction
        self._model.geom_size[self._wheel_L_geom_id, 0] = self._base_wheel_radius
        self._model.geom_size[self._wheel_R_geom_id, 0] = self._base_wheel_radius
        self.realism.reset()
        self._n_steps = 0

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _sensor_addr(self, name: str) -> tuple[int, int]:
        """Return (start, end) address tuple for a sensor in sensordata."""
        sid = mujoco.mj_name2id(self._model, mujoco.mjtObj.mjOBJ_SENSOR, name)
        if sid < 0:
            raise ValueError(f"Sensor '{name}' not found in model")
        adr = self._model.sensor_adr[sid]
        dim = self._model.sensor_dim[sid]
        return (adr, adr + dim)
