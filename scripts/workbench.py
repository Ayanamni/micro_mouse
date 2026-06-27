#!/usr/bin/env python3
"""
Micromouse Simulation Workbench — 5kHz pipeline + real-time dashboard.

One script. All you need for algorithm development and parameter tuning.

Usage:
    python scripts/workbench.py                                    # defaults: robotena, 2 m/s
    python scripts/workbench.py --track 2019kansai --speed 3.0     # different track/speed
    python scripts/workbench.py --preset my-tune                   # load saved config
    python scripts/workbench.py --with-viewer                       # MuJoCo 3D + dashboard
    python scripts/workbench.py --no-render                        # headless batch mode

Keyboard shortcuts:
    Space       Pause / Resume
    R           Reset (clear waveforms + restart)
    Ctrl+S      Save current parameters as preset
    Ctrl+L      Load preset
    F12         Screenshot
    Q / Esc     Quit
"""

from __future__ import annotations

import argparse
import logging
import os
import sys
import threading
import time
from collections import deque
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, Optional

import numpy as np
import yaml

# ── Project setup ───────────────────────────────────────────────────────────
PROJECT_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(PROJECT_ROOT))

from micromouse_sim.physics.engine import (
    PhysicsEngine, SimulationState,
    WHEEL_RADIUS, TRACK_WIDTH, PULSES_PER_M,
)
from micromouse_sim.environment.loader import build_model_xml, load_track
from micromouse_sim.environment.track import TrackCenterline
from micromouse_sim.actuation.delay_buffer import ActuationDelayBuffer
from micromouse_sim.actuation.motor_model import MotorModel
from micromouse_sim.config.vw_controller_config import (
    VW_DEFAULTS,
    push_vw_params as push_vw_controller_params,
)
from micromouse_sim.sensors.line_sensor import LineSensor, LineSensorConfig
from micromouse_sim import localize_core, control_core

logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s: %(message)s")
logger = logging.getLogger("workbench")

# ── Paths ───────────────────────────────────────────────────────────────────
BASE_XML = PROJECT_ROOT / "mujoco_models" / "micromouse" / "base.xml"
_TRACK_DIR = Path(r"C:\Users\chj15\Desktop\RobotRace\路径优化\robotrace-shortcut-path-main\data")
if not _TRACK_DIR.exists():
    _TRACK_DIR = PROJECT_ROOT.parent / "路径优化" / "robotrace-shortcut-path-main" / "data"
TRACK_DATA_DIR = _TRACK_DIR
PRESETS_DIR = PROJECT_ROOT / "presets"
PRESETS_DIR.mkdir(exist_ok=True)

# ── 可调物理参数（改这里！）──────────────────────────────────────────────────
PHYSICS_DT   = 2e-5       # 物理步长 = 50kHz
IMU_RATE     = 5000       # IMU频率 Hz（不改）
IMU_STEPS    = int(1.0 / (IMU_RATE * PHYSICS_DT))   # 10
IMU_DT       = 1.0 / IMU_RATE                        # 200 µs
CTRL_RATE    = 1000       # 控制频率 Hz（不改）
CTRL_STEPS   = int(1.0 / (CTRL_RATE * PHYSICS_DT))  # 50
CTRL_DT      = 1.0 / CTRL_RATE                        # 1 ms

# ── 默认参数（仪表盘滑块和预设文件都改这里）──────────────────────────────────
@dataclass
class SimParams:
    """所有可调参数的唯一数据源。"""
    # ==== 赛道 & 速度 ====
    track: str = "2019kansai"            # 赛道: robotena(4.8m) | 2019kansai(14.8m)
    target_speed: float = 3.5           # 目标线速度 (m/s)
    track_width: float = 0.20           # 赛道宽度 (m)，用于渲染+传感器参考

    # ==== v-ω 解耦控制（主控方案）====
    # ── 车辆几何 ──
    vw_wheel_r: float = VW_DEFAULTS.vw_wheel_r        # m，轮半径
    vw_track_B: float = VW_DEFAULTS.vw_track_B         # m，轮距
    # ── ω 环（角速度内环，5kHz，前馈：τ_ff=(Jz*r/B)*α+(Dw*r/B)*ω）──
    vw_Jz: float = VW_DEFAULTS.vw_Jz             # kg·m²，Z轴转动惯量（物理常数，base.xml推导）
    vw_Dw: float = VW_DEFAULTS.vw_Dw             # Nm/(rad/s)，偏航粘性阻尼系数
    vw_w_Kp: float = VW_DEFAULTS.vw_w_Kp             # (Nm)/(rad/s)，ω 环比例
    vw_w_Ki: float = VW_DEFAULTS.vw_w_Ki             # (Nm)/(rad·s)，ω 环积分（抗饱和回算已实现，kaw_w=0.5）
    vw_w_Kd: float = VW_DEFAULTS.vw_w_Kd              # (Nm)/(rad/s²)，ω 环微分（暂关，Ki先行；待库仑FF后再评估Kd需求）
    vw_w_max: float = VW_DEFAULTS.vw_w_max            # Nm，ω 环力矩限幅
    vw_beta_w: float = VW_DEFAULTS.vw_beta_w             # 2DOF设定值权重，<1可降低ω阶跃超调
    vw_kaw_w: float = VW_DEFAULTS.vw_kaw_w              # ω环抗饱和回算增益
    # ── v 环（线速度外环，1kHz，前馈：τ_ff=(m_eq*r/2)*a+(D_v*r/2)*v+C_frict）──
    vw_m_eq: float = VW_DEFAULTS.vw_m_eq             # kg，等效平移质量
    vw_D_v: float = VW_DEFAULTS.vw_D_v               # N/(m/s)，平移粘性阻尼余项（反电势已由电压前馈处理，仅剩微小气动+机械粘性）
    vw_C_frict: float = VW_DEFAULTS.vw_C_frict        # Nm，库仑摩擦补偿（μ·downforce·r/2 = 0.05·6·0.0105/2 ≈ 0.00158，留积分兜底）
    vw_v_Kp: float = VW_DEFAULTS.vw_v_Kp              # (Nm)/(m/s)，v 环比例
    vw_v_Ki: float = VW_DEFAULTS.vw_v_Ki              # (Nm)/m，v 环积分
    vw_v_max: float = VW_DEFAULTS.vw_v_max            # Nm，v 环力矩限幅
    vw_beta_v: float = VW_DEFAULTS.vw_beta_v             # v环2DOF设定值权重
    vw_kaw_v: float = VW_DEFAULTS.vw_kaw_v              # v环抗饱和回算增益
    # ── 巡线控制器（纯光电管反馈）──
    vw_lat_Kp: float = VW_DEFAULTS.vw_lat_Kp          # (rad/s)/m，横向误差→ω_ref
    vw_lat_Ki: float = VW_DEFAULTS.vw_lat_Ki           # (rad/s)/m²，横向积分（消除稳态漂移）
    vw_lat_Kd: float = VW_DEFAULTS.vw_lat_Kd            # (rad/s)/(m/s)，横向变化率阻尼
    vw_w_Cfrict: float = VW_DEFAULTS.vw_w_Cfrict        # Nm，ω环库仑摩擦前馈（μ·downforce·skirt_R理论0.009，留余量避免超调）

    # ==== 侧向 PID（legacy，保留做 A/B 对比）====
    lat_Kp: float = 0.0
    lat_Kd: float = 1.1
    lat_Ki: float = 0.1
    lat_Kff: float = 0.0
    lat_ema_alpha: float = 0.25

    # ==== 速度 PI（legacy）====
    spd_Kp: float = 0.0
    spd_Ki: float = 0.5

    # ==== Kalman 定位器噪声参数 ====
    sigma_accel: float = 0.01            # 加速度计噪声标准差 (m/s²)，越大越不信IMU
    sigma_enc_dist: float = 2.5e-4      # 编码器距离噪声标准差 (m)，越大越不信编码器

    # ==== 滑移检测 ====
    thresh_lon: float = 0.10            # 纵向滑移检测阈值 (m/s)，v_imu与v_enc差值超此值触发
    thresh_lat: float = 0.5             # 侧向滑移检测阈值 (m/s²)
    k_slip: float = 0.3                 # 滑移缩放系数：1+k*(残差-阈值)，越大触发越猛

    # ==== 物理 ====
    downforce: float = 6.0              # 下压力 (N)，负压风扇
    skirt_mu: float = 0.05              # 裙边 Coulomb 摩擦系数（特氟龙-赛道 ≈0.03-0.08）
    skirt_R: float = 0.03               # 裙边有效摩擦半径 (m)，≈车体半对角线

    # ==== Realism 真实感（全部默认关闭）====
    surface_bumps: bool = False          # 赛道轻微颠簸
    bump_amplitude: float = 9.15         # 颠簸 Z 力幅值 (N)
    friction_var: bool = False            # 摩擦不均匀
    friction_var_amplitude: float = 0.1 # 摩擦变化幅度 (相对 μ 的 ±8%)
    friction_degrade: bool = False       # 摩擦衰减（轮胎粘灰）
    degrade_rate: float = 0.004          # 每米衰减率

    # ==== 电机 ====
    ideal_motor: bool = False           # True=理想电机(无模型) False=真实电机模型
    motor_I_peak: float = VW_DEFAULTS.motor_I_peak          # A，峰值电流，用于控制器动态力矩上限

    # ==== 线传感器 ====
    sensor_n_leds: int = 16             # 光电管数量
    sensor_half_span: float = 0.070     # 传感器半宽 (m)，±70mm 检测范围
    sensor_line_width: float = 0.020    # 白线宽度 (m)，标准20mm
    sensor_fwd_offset: float = 0.040    # 传感器前瞻距离 (m)，与base.xml LED位置一致

    # ==== 延迟注入（真实度）====
    act_delay_us: float = 0.0           # 执行延迟 (us)，0=无延迟；用CLI/预设显式打开

    # ==== 显示 ====
    buffer_seconds: float = 20.0        # 波形缓冲区时长 (s)
    dashboard_hz: float = 20.0          # 仪表盘刷新率 (Hz)

    def to_dict(self) -> dict:
        return {k: v for k, v in self.__dict__.items()}

    @classmethod
    def from_dict(cls, d: dict) -> "SimParams":
        valid = {f.name for f in cls.__dataclass_fields__.values()}
        return cls(**{k: v for k, v in d.items() if k in valid})


# ╔══════════════════════════════════════════════════════════════════════════════╗
# ║                          SIMULATION RUNNER                                   ║
# ╚══════════════════════════════════════════════════════════════════════════════╝

class SimRunner:
    """Background-thread simulation. Thread-safe data access for GUI."""

    def __init__(self, params: SimParams, seed: int = 42, with_viewer: bool = False):
        self.params = params
        self.seed = seed
        self.with_viewer = with_viewer
        self._viewer = None
        self._lock = threading.Lock()
        self._running = False
        self._paused = False
        self._quit = False
        self._reset_pending = False  # GUI sets this; sim thread handles reset safely

        # ── Latest snapshot data (populated by sim thread, read by GUI) ──
        self._snap: Dict[str, Any] = {}
        self._lap_count = 0
        self._max_lat = 0.0
        self._sim_time = 0.0
        self._origin_x = 0.0  # ground-truth position at start, for trajectory alignment
        self._origin_y = 0.0

        # ── Buffers (GUI-side, populated in get_snapshot) ──
        self.buf_time: deque = deque(maxlen=int(params.buffer_seconds * params.dashboard_hz))
        self.buf_lat_mm: deque = deque(maxlen=self.buf_time.maxlen)
        self.buf_u_L: deque = deque(maxlen=self.buf_time.maxlen)
        self.buf_u_R: deque = deque(maxlen=self.buf_time.maxlen)
        self.buf_v_fwd: deque = deque(maxlen=self.buf_time.maxlen)
        self.buf_v_target: deque = deque(maxlen=self.buf_time.maxlen)
        self.buf_innovation: deque = deque(maxlen=self.buf_time.maxlen)
        self.buf_slip_scale: deque = deque(maxlen=self.buf_time.maxlen)
        self.buf_P00: deque = deque(maxlen=self.buf_time.maxlen)
        self.buf_bias: deque = deque(maxlen=self.buf_time.maxlen)
        self.buf_loc_err: deque = deque(maxlen=self.buf_time.maxlen)  # 定位误差 (mm)
        self.buf_throttle: deque = deque(maxlen=self.buf_time.maxlen)
        self.buf_steer: deque = deque(maxlen=self.buf_time.maxlen)

        # ── Trajectories (unbounded, with downsampling) ──
        self.gt_traj_x: list = []
        self.gt_traj_y: list = []
        self.est_traj_x: list = []
        self.est_traj_y: list = []
        self._traj_counter = 0

        # ── Line sensor state ──
        self._line_lost = False
        self._last_lat = 0.0
        self._line_lost_time = 0.0
        self._sensor_warned = False
        self._estopped = False
        self._lat_filtered = 0.0

        # ── Engine / track / motors (created in _init_simulation) ──
        self.engine: Optional[PhysicsEngine] = None
        self.track: Optional[TrackCenterline] = None
        self.motor_L: Optional[MotorModel] = None
        self.motor_R: Optional[MotorModel] = None

    # ── Public API ────────────────────────────────────────────────────────

    def start(self):
        """Launch simulation in background thread."""
        self._running = True
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self):
        """Signal quit and wait for thread."""
        self._quit = True
        self._running = False
        if hasattr(self, "_thread") and self._thread.is_alive():
            self._thread.join(timeout=2.0)

    def pause(self):   self._paused = True
    def resume(self):  self._paused = False
    def toggle_pause(self):
        self._paused = not self._paused
        return self._paused

    @property
    def paused(self) -> bool: return self._paused
    @property
    def sim_time(self) -> float: return self._sim_time
    @property
    def lap_count(self) -> int: return self._lap_count
    @property
    def max_lat(self) -> float: return self._max_lat

    def get_snapshot(self) -> dict:
        """Thread-safe read of latest simulation data + buffer copies.
        Called from GUI thread at dashboard refresh rate (~30 Hz)."""
        with self._lock:
            snap = dict(self._snap)  # shallow copy
            # Copy trajectory references (lists are thread-safe for append+read)
            snap["gt_traj_x"] = list(self.gt_traj_x)
            snap["gt_traj_y"] = list(self.gt_traj_y)
            snap["est_traj_x"] = list(self.est_traj_x)
            snap["est_traj_y"] = list(self.est_traj_y)
            snap["lap_count"] = self._lap_count
            snap["max_lat"] = self._max_lat
            snap["sim_time"] = self._sim_time
            snap["paused"] = self._paused
            # Copy latest buffer values (thread-safe copies)
            snap["bufs"] = {
                "time":       list(self.buf_time),
                "lat_mm":     list(self.buf_lat_mm),
                "u_L":        list(self.buf_u_L),
                "u_R":        list(self.buf_u_R),
                "v_fwd":      list(self.buf_v_fwd),
                "v_target":   list(self.buf_v_target),
                "innovation": list(self.buf_innovation),
                "slip_scale": list(self.buf_slip_scale),
                "P00":        list(self.buf_P00),
                "bias":       list(self.buf_bias),
                "loc_err":    list(self.buf_loc_err),
                "throttle":   list(self.buf_throttle),
                "steer":      list(self.buf_steer),
            }
            return snap

    # ── Parameter push from GUI sliders ───────────────────────────────────

    def push_lateral_gains(self):
        control_core.set_lateral_gains(
            self.params.lat_Kp, self.params.lat_Kd,
            self.params.lat_Ki, self.params.lat_Kff)

    def push_speed_gains(self):
        control_core.set_speed_gains(self.params.spd_Kp, self.params.spd_Ki)

    def push_vw_params(self):
        """Push all v-ω controller params to C++."""
        push_vw_controller_params(control_core, self.params)

    def push_kalman_params(self):
        localize_core.set_calibration(
            pulses_per_m_L=PULSES_PER_M,
            pulses_per_m_R=PULSES_PER_M,
            accel_noise_std=self.params.sigma_accel,
            enc_dist_noise=self.params.sigma_enc_dist,
            track_width=TRACK_WIDTH,
            gyro_bias_init=self._gyro_bias_init,
        )

    def push_slip_params(self):
        localize_core.set_slip_params(
            self.params.thresh_lon, self.params.thresh_lat, self.params.k_slip)

    def push_all_params(self):
        """Push all current params to C++ cores."""
        self.push_vw_params()
        self.push_lateral_gains()
        self.push_speed_gains()
        self.push_kalman_params()
        self.push_slip_params()

    def request_reset(self):
        """Signal the sim thread to reset at next safe point."""
        self._reset_pending = True

    def _do_reset(self):
        """Perform actual reset — called from sim thread only."""
        self._lap_count = 0
        self._max_lat = 0.0
        self._sim_time = 0.0
        self._traj_counter = 0
        self._line_lost = False
        self._last_lat = 0.0
        self._line_lost_time = 0.0
        self._sensor_warned = False
        self._estopped = False
        self._lat_filtered = 0.0

        # Reset C++ cores
        localize_core.reset(self.seed)
        control_core.reset()
        self.push_all_params()

        # Reset physics
        if self.engine is not None:
            self.engine.reset()
        if self.motor_L is not None:
            self.motor_L.reset()
        if self.motor_R is not None:
            self.motor_R.reset()

        # Clear buffers (thread-safe — sim thread owns these)
        for attr in list(dir(self)):
            if attr.startswith("buf_") and isinstance(getattr(self, attr), deque):
                getattr(self, attr).clear()
        self.gt_traj_x.clear(); self.gt_traj_y.clear()
        self.est_traj_x.clear(); self.est_traj_y.clear()

        # Re-calibrate gyro bias
        if self.engine is not None:
            self._settle_and_calibrate()
            self.push_kalman_params()  # update gyro_bias_init in C++

        # Record origin for trajectory offset correction
        st = self.engine.get_state()
        self._origin_x = float(st.pos[0])
        self._origin_y = float(st.pos[1])

    # ── Internal ──────────────────────────────────────────────────────────

    def _settle_and_calibrate(self):
        """Settle physics + measure gyro bias."""
        for _ in range(5000):  # 100ms settle
            self.engine.step()
        gyro_samples = []
        for _ in range(10000):  # 200ms gyro sampling
            self.engine.step()
            gyro_samples.append(self.engine.get_state().gyro[2])
        self._gyro_bias_init = float(np.mean(gyro_samples))
        logger.info("Gyro bias: %.6f rad/s", self._gyro_bias_init)
        # Reset realism accumulators (settle phase creates false distance)
        self.engine.realism.reset()

    def _run(self):
        """Main simulation loop (runs in background thread)."""
        try:
            self._init_simulation()
            self._settle_and_calibrate()
            self._init_cores()

            st = self.engine.get_state()
            logger.info("Settled: z=%.1fmm", st.pos[2] * 1000)

            # Record ground-truth origin for trajectory alignment
            self._origin_x = float(st.pos[0])
            self._origin_y = float(st.pos[1])

            # ── MuJoCo 3D viewer (optional) ──
            if self.with_viewer:
                try:
                    import mujoco.viewer
                    self._viewer = mujoco.viewer.launch_passive(
                        self.engine.model, self.engine.data,
                        show_left_ui=False, show_right_ui=False)
                    self._viewer.cam.azimuth = 135; self._viewer.cam.elevation = -35
                    self._viewer.cam.distance = 1.5
                    self._viewer.cam.lookat[:] = [0.3, -0.3, 0.01]
                    logger.info("MuJoCo viewer opened")
                except Exception as e:
                    logger.warning("MuJoCo viewer failed: %s", e)
                    self._viewer = None

            # ── Loop state ──
            u_L, u_R = 0.0, 0.0
            gyro_acc  = np.zeros(3)
            accel_acc = np.zeros(3)

            # ── Actuation delay buffer ──
            act_delay = ActuationDelayBuffer(self.params.act_delay_us, PHYSICS_DT)
            imu_raw_count = 0
            step_count = 0
            prev_s = -1.0
            lap_high = False
            last_dashboard_push = 0.0
            dashboard_interval = 1.0 / self.params.dashboard_hz

            while not self._quit:
                # ── Handle reset request from GUI thread ──
                if self._reset_pending:
                    self._do_reset()
                    self._reset_pending = False
                    # Reset loop state + delay buffer
                    u_L, u_R = 0.0, 0.0
                    act_delay.reset()
                    gyro_acc[:] = 0.0; accel_acc[:] = 0.0
                    imu_raw_count = 0
                    step_count = 0
                    prev_s = -1.0; lap_high = False
                    last_dashboard_push = 0.0
                    self._line_lost = False; self._last_lat = 0.0; self._line_lost_time = 0.0; self._estopped = False; self._lat_filtered = 0.0
                    continue

                # Handle pause
                if self._paused:
                    time.sleep(0.01)
                    continue

                # ── (1) Motor torque (with optional actuation delay) ──
                st = self.engine.get_state()
                u_delayed_L, u_delayed_R = act_delay.apply(u_L, u_R)

                if self.params.ideal_motor:
                    tau_L = u_delayed_L * 0.05
                    tau_R = u_delayed_R * 0.05
                else:
                    tau_L = self.motor_L.compute_torque(u_delayed_L, st.wheel_L_vel, PHYSICS_DT)
                    tau_R = self.motor_R.compute_torque(u_delayed_R, st.wheel_R_vel, PHYSICS_DT)
                self.engine.set_control(tau_L, tau_R)

                # ── (2) Physics step ──
                self.engine.step()
                st = self.engine.get_state()

                # ── Viewer sync (every step to avoid lag) ──
                if self._viewer is not None:
                    if self._viewer.is_running():
                        self._viewer.cam.lookat[:] = [st.pos[0], st.pos[1], 0.02]
                        if step_count % 10 == 0:  # sync every 10 physics steps
                            self._viewer.sync()
                    else:
                        self._viewer = None  # viewer was closed by user

                # ── (3) IMU accumulation ──
                gyro_acc  += st.gyro
                accel_acc += st.accel
                imu_raw_count += 1

                # ── (4) 5kHz pipeline push + v-ω omega step ──
                step_in_ctrl = step_count % CTRL_STEPS
                if step_in_ctrl % IMU_STEPS == 0 and imu_raw_count > 0:
                    gyro_avg  = gyro_acc / imu_raw_count
                    accel_avg = accel_acc / imu_raw_count
                    noisy = localize_core.imu_step(gyro_avg, accel_avg, IMU_DT)
                    localize_core.push_imu(
                        float(noisy["gyro"][2]),
                        float(noisy["accel"][0]),
                        float(noisy["accel"][1]),
                        IMU_DT,
                    )
                    # ── v-ω: ω 环步进（5kHz，用含噪陀螺仪原始值）──
                    control_core.vw_omega_step(float(noisy["gyro"][2]), IMU_DT)
                    gyro_acc[:] = 0.0; accel_acc[:] = 0.0
                    imu_raw_count = 0

                # ── (5) 1kHz encoder push + control ──
                if step_in_ctrl == 0:
                    self._sim_time = st.time

                    # Encoder
                    localize_core.push_encoder(st.wheel_L_pos, st.wheel_R_pos, CTRL_DT)
                    pose = localize_core.read_pose()
                    dbg  = localize_core.get_debug_state()

                    # ── Line sensor (finite-width photodiode array) ──
                    reading = self.line_sensor.read(st.pos[:2], st.yaw, self.track)

                    if not reading.line_detected:
                        # LINE LOST → immediate emergency stop
                        if not getattr(self, '_estopped', False):
                            logger.error(
                                "LINE LOST at t=%.2fs! EMERGENCY STOP — motors cut. Press R to reset.",
                                st.time
                            )
                            self._estopped = True
                        u_L, u_R = 0.0, 0.0
                        throttle, steer = 0.0, 0.0
                        lateral_error = 0.0
                        curvature = 0.0
                    else:
                        # Line detected — normal control
                        self._estopped = False

                        # Line detected — pure photodiode sensor feedback
                        raw_lat = reading.lateral_error if reading.lateral_error is not None else 0.0
                        self._lat_filtered += self.params.lat_ema_alpha * (raw_lat - self._lat_filtered)
                        lateral_error = self._lat_filtered
                        curvature = reading.curvature if reading.curvature is not None else 0.0

                        # Lap detection: cross from >90% track to <10% track
                        s_pos = reading.s_pos if reading.s_pos is not None else -1.0
                        if s_pos > 0:
                            total = self.track.total_length
                            finish_zone = total * 0.90
                            start_zone = total * 0.10
                            if prev_s > finish_zone and s_pos < start_zone:
                                self._lap_count += 1
                                logger.info("LAP %d @ t=%.1fs", self._lap_count, st.time)
                            prev_s = s_pos

                        self._max_lat = max(self._max_lat, abs(lateral_error))

                        # Control — v-ω 解耦控制器
                        tgt_spd = self.params.target_speed
                        control_core.vw_set_wheel_omega(st.wheel_L_vel, st.wheel_R_vel)
                        cmd = control_core.vw_control_tick(
                            lateral_error, curvature,
                            float(pose["v_fwd"]), tgt_spd, CTRL_DT)  # Kalman estimate
                        u_L = float(cmd["u_L"])
                        u_R = float(cmd["u_R"])
                        throttle = float(cmd["throttle"])
                        steer = float(cmd["steer"])

                    # ── Update snapshot for GUI ──
                    with self._lock:
                        vw_dbg = control_core.vw_get_debug()
                        self._snap = {
                            "time": st.time,
                            "x_gt": float(st.pos[0]), "y_gt": float(st.pos[1]),
                            "yaw_gt": float(st.yaw),
                            "v_fwd_gt": float(st.forward_velocity),
                            "x_est": float(pose["x"]), "y_est": float(pose["y"]),
                            "yaw_est": float(pose["yaw"]),
                            "v_fwd_est": float(pose["v_fwd"]),
                            "w_z": float(pose["w_z"]),
                            "lateral_error_mm": float(lateral_error * 1000),
                            "u_L": u_L, "u_R": u_R,
                            "throttle": throttle, "steer": steer,
                            # v-ω debug
                            "omega_meas": float(vw_dbg["omega_meas"]),
                            "omega_ref": float(vw_dbg["omega_ref"]),
                            "omega_tau_ff": float(vw_dbg["omega_tau_ff"]),
                            "omega_tau_fb": float(vw_dbg["omega_tau_fb"]),
                            "v_tau_ff": float(vw_dbg["v_tau_ff"]),
                            "v_tau_fb": float(vw_dbg["v_tau_fb"]),
                            "tau_v": float(vw_dbg["tau_v"]),
                            "tau_omega": float(vw_dbg["tau_omega"]),
                            # Kalman debug
                            "innovation": float(dbg["innovation"]),
                            "P00": float(dbg["P00"]),
                            "P11": float(dbg["P11"]),
                            "accel_bias": float(dbg["accel_bias"]),
                            "slip_scale": float(pose["slip_scale"]),
                            "slip_lon": float(dbg["slip_lon"]),
                            "slip_lat": float(dbg["slip_lat"]),
                            "cov_v": float(pose["cov_v"]),
                            "cov_bias": float(pose["cov_bias"]),
                            "target_speed": self.params.target_speed,
                            "line_lost": self._line_lost,
                            "estopped": self._estopped,
                        }

                        # Trajectory (downsample, skip Kalman transient, apply origin offset)
                        self._traj_counter += 1
                        if self._traj_counter % 10 == 0 and st.time > 0.3:
                            self.gt_traj_x.append(float(st.pos[0]) - self._origin_x)
                            self.gt_traj_y.append(float(st.pos[1]) - self._origin_y)
                            self.est_traj_x.append(float(pose["x"]))
                            self.est_traj_y.append(float(pose["y"]))
                            if len(self.gt_traj_x) > 20000:
                                # Downsample
                                self.gt_traj_x = self.gt_traj_x[::2]
                                self.gt_traj_y = self.gt_traj_y[::2]
                                self.est_traj_x = self.est_traj_x[::2]
                                self.est_traj_y = self.est_traj_y[::2]
                                self._traj_counter = 0

                    # ── Push to GUI buffers at dashboard rate ──
                    if st.time - last_dashboard_push >= dashboard_interval:
                        last_dashboard_push = st.time
                        with self._lock:
                            self.buf_time.append(st.time)
                            self.buf_lat_mm.append(float(lateral_error * 1000))
                            self.buf_u_L.append(u_L)
                            self.buf_u_R.append(u_R)
                            self.buf_v_fwd.append(float(st.forward_velocity))
                            self.buf_v_target.append(self.params.target_speed)
                            self.buf_innovation.append(float(dbg["innovation"]) * 1000)  # mm
                            self.buf_slip_scale.append(float(pose["slip_scale"]))
                            self.buf_P00.append(float(dbg["P00"]))
                            self.buf_bias.append(float(dbg["accel_bias"]))
                            loc_err = np.sqrt(
                                (float(pose["x"]) - float(st.pos[0]))**2 +
                                (float(pose["y"]) - float(st.pos[1]))**2
                            )
                            self.buf_loc_err.append(loc_err * 1000)  # mm
                            self.buf_throttle.append(throttle)
                            self.buf_steer.append(steer)

                step_count += 1

        except Exception:
            logger.exception("Simulation thread crashed")
        finally:
            logger.info("Simulation thread exiting")
            self._running = False

    def _init_simulation(self):
        p = self.params
        track = load_track(str(TRACK_DATA_DIR / f"{p.track}_points.txt"))
        self.track = track
        logger.info("Track: %.2f m, %d pts", track.total_length, track.waypoints.shape[0])
        model_xml = build_model_xml(str(BASE_XML), track=track, track_width=p.track_width)

        # ── Realism config ──
        from micromouse_sim.physics.realism import RealismConfig
        realism_cfg = RealismConfig(
            surface_bumps_enabled=p.surface_bumps,
            bump_amplitude=p.bump_amplitude,
            friction_var_enabled=p.friction_var,
            friction_var_amplitude=p.friction_var_amplitude,
            friction_degrade_enabled=p.friction_degrade,
            degrade_rate=p.degrade_rate,
        )

        # ── 物理引擎参数全部来自 SimParams ──
        self.engine = PhysicsEngine(
            model_xml=model_xml,
            downforce=p.downforce,
            skirt_mu=p.skirt_mu,
            skirt_R=p.skirt_R,
            realism_config=realism_cfg,
        )

        # Set track bounds for noise fields
        if realism_cfg.enabled:
            pts = track.waypoints
            self.engine.realism.set_track_bounds(
                float(pts[:, 0].min()), float(pts[:, 0].max()),
                float(pts[:, 1].min()), float(pts[:, 1].max()),
            )
        self.motor_L = MotorModel()
        self.motor_R = MotorModel()
        # ── 线传感器参数全部来自 SimParams ──
        self.line_sensor = LineSensor(LineSensorConfig(
            n_leds=p.sensor_n_leds,
            half_span=p.sensor_half_span,
            line_width=p.sensor_line_width,
            fwd_offset=p.sensor_fwd_offset,
        ))

    def _init_cores(self):
        localize_core.reset(self.seed)
        control_core.reset()
        control_core.vw_reset()
        # push_all_params pushes kalman params with calibrated gyro_bias_init
        self.push_all_params()


# ╔══════════════════════════════════════════════════════════════════════════════╗
# ║                            PRESET MANAGER                                    ║
# ╚══════════════════════════════════════════════════════════════════════════════╝

class PresetManager:
    """Save/load parameter presets as YAML files."""

    @staticmethod
    def save(params: SimParams, name: str):
        path = PRESETS_DIR / f"{name}.yaml"
        data = {
            "name": name,
            "saved_at": time.strftime("%Y-%m-%d %H:%M:%S"),
            "params": params.to_dict(),
        }
        with open(path, "w", encoding="utf-8") as f:
            yaml.dump(data, f, default_flow_style=False, allow_unicode=True, sort_keys=False)
        logger.info("Preset saved: %s", path)

    @staticmethod
    def load(name: str) -> Optional[SimParams]:
        path = PRESETS_DIR / f"{name}.yaml"
        if not path.exists():
            logger.warning("Preset not found: %s", path)
            return None
        with open(path, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f)
        return SimParams.from_dict(data.get("params", {}))

    @staticmethod
    def list_presets() -> list:
        return sorted([p.stem for p in PRESETS_DIR.glob("*.yaml")])


# ╔══════════════════════════════════════════════════════════════════════════════╗
# ║                          MONITORING PANELS                                   ║
# ╚══════════════════════════════════════════════════════════════════════════════╝

class MonitorPanel:
    """Base class for all monitoring panels."""
    name: str = "base"

    def __init__(self, ax, title: str):
        self.ax = ax
        self.ax.set_title(title, fontsize=9, fontweight="bold")
        self.ax.tick_params(labelsize=7)

    def update(self, snap: dict, bufs):
        """Update panel with latest data. Override in subclasses."""
        pass

    def reset(self):
        """Clear panel data. Override in subclasses."""
        pass


class TrackingPanel(MonitorPanel):
    """Lateral error waveform."""
    name = "tracking"

    def __init__(self, ax):
        super().__init__(ax, "Lateral Error (mm)")
        self.ax.set_ylabel("mm", fontsize=7)
        self.ax.axhline(y=0, color="gray", linestyle="--", linewidth=0.5, alpha=0.5)
        self.line, = self.ax.plot([], [], "b-", linewidth=1.0)
        self.ax.set_ylim(-120, 120)

    def update(self, snap, bufs):
        t = list(bufs["time"])
        y = list(bufs["lat_mm"])
        self.line.set_data(t, y)
        if len(t) > 1:
            self.ax.set_xlim(max(0, t[-1] - 20), max(t[-1], 0.1))
        # Dynamic y-axis
        if y:
            ym = max(abs(min(y)), abs(max(y))) * 1.2
            self.ax.set_ylim(-max(ym, 5), max(ym, 5))

    def reset(self):
        self.line.set_data([], [])


class MapPanel(MonitorPanel):
    """Position map: track centerline + estimated trajectory + ground truth."""
    name = "map"

    def __init__(self, ax, track: TrackCenterline):
        super().__init__(ax, "Position Map")
        # Precompute track centerline
        n_pts = 2000
        s_vals = np.linspace(0, track.total_length, n_pts)
        pts = np.array([track.point_at(s) for s in s_vals])
        self.ax.plot(pts[:, 0], pts[:, 1], "r-", linewidth=0.6, alpha=0.7, label="Track")
        self.line_est, = self.ax.plot([], [], "b-", linewidth=1.2, label="Estimate")
        self.line_gt, = self.ax.plot([], [], "gray", linewidth=0.6, linestyle="--", alpha=0.5, label="Ground Truth")
        self.dot_est, = self.ax.plot([], [], "bo", markersize=5, label="Current")
        self.ax.set_aspect("equal")
        self.ax.legend(fontsize=6, loc="upper right")
        self.ax.tick_params(labelsize=6)

    def update(self, snap, bufs):
        ex = snap.get("est_traj_x", [])
        ey = snap.get("est_traj_y", [])
        gx = snap.get("gt_traj_x", [])
        gy = snap.get("gt_traj_y", [])
        self.line_est.set_data(ex, ey)
        self.line_gt.set_data(gx, gy)
        if ex:
            self.dot_est.set_data([ex[-1]], [ey[-1]])
            mx, my = ex[-1], ey[-1]
            self.ax.set_xlim(mx - 1.5, mx + 1.5)
            self.ax.set_ylim(my - 1.5, my + 1.5)

    def reset(self):
        self.line_est.set_data([], [])
        self.line_gt.set_data([], [])
        self.dot_est.set_data([], [])


class KalmanPanel(MonitorPanel):
    """Kalman filter diagnostics + localization error."""
    name = "kalman"

    def __init__(self, ax):
        super().__init__(ax, "Kalman + Localization Error")
        self.ax.set_ylabel("innovation (mm)", fontsize=7)
        self.line_innov, = self.ax.plot([], [], "b-", linewidth=1.0, label="Innov (mm)")
        self.line_bias, = self.ax.plot([], [], "r-", linewidth=1.0, label="Bias (m/s²)")
        self.ax.legend(fontsize=6, loc="upper left")
        self.ax.axhline(y=0, color="gray", linewidth=0.5, alpha=0.3)

        # Twin axis for localization error
        self.ax2 = self.ax.twinx()
        self.ax2.set_ylabel("loc err (mm)", fontsize=7, color="orange")
        self.line_loc, = self.ax2.plot([], [], "orange", linewidth=1.5, label="Loc Err (mm)")
        self.ax2.tick_params(axis='y', labelcolor="orange", labelsize=7)

    def update(self, snap, bufs):
        t = list(bufs["time"])
        self.line_innov.set_data(t, list(bufs["innovation"]))
        self.line_bias.set_data(t, list(bufs["bias"]))
        self.line_loc.set_data(t, list(bufs["loc_err"]))
        if len(t) > 1:
            self.ax.set_xlim(max(0, t[-1] - 20), max(t[-1], 0.1))
        # Dynamic y for innovation/bias
        all_y = list(bufs["innovation"]) + list(bufs["bias"])
        if all_y:
            ym = max(abs(min(all_y)), abs(max(all_y))) * 1.3
            self.ax.set_ylim(-max(ym, 0.1), max(ym, 0.1))
        # Dynamic y for loc_err
        loc = list(bufs["loc_err"])
        if loc:
            self.ax2.set_ylim(-5, max(50, max(loc) * 1.2))

    def reset(self):
        for line in [self.line_innov, self.line_bias, self.line_loc]:
            line.set_data([], [])


class MotorPanel(MonitorPanel):
    """Motor commands: u_L, u_R, throttle, steer."""
    name = "motor"

    def __init__(self, ax):
        super().__init__(ax, "Motor Commands")
        self.ax.set_ylabel("normalized [-1, 1]", fontsize=7)
        self.ax.set_ylim(-1.15, 1.15)
        self.ax.axhline(y=0, color="gray", linewidth=0.5, alpha=0.3)
        self.line_uL, = self.ax.plot([], [], "r-", linewidth=0.8, alpha=0.7, label="u_L")
        self.line_uR, = self.ax.plot([], [], "b-", linewidth=0.8, alpha=0.7, label="u_R")
        self.line_thr, = self.ax.plot([], [], "k-", linewidth=1.2, label="throttle")
        self.line_str, = self.ax.plot([], [], "g--", linewidth=0.8, label="steer")
        self.ax.legend(fontsize=6, loc="upper right", ncol=2)

    def update(self, snap, bufs):
        t = list(bufs["time"])
        self.line_uL.set_data(t, list(bufs["u_L"]))
        self.line_uR.set_data(t, list(bufs["u_R"]))
        self.line_thr.set_data(t, list(bufs["throttle"]))
        self.line_str.set_data(t, list(bufs["steer"]))
        if len(t) > 1:
            self.ax.set_xlim(max(0, t[-1] - 20), max(t[-1], 0.1))

    def reset(self):
        for line in [self.line_uL, self.line_uR, self.line_thr, self.line_str]:
            line.set_data([], [])


class SpeedPanel(MonitorPanel):
    """Speed tracking: v_fwd vs target, slip scale."""
    name = "speed"

    def __init__(self, ax):
        super().__init__(ax, "Speed & Slip")
        self.line_v, = self.ax.plot([], [], "b-", linewidth=1.2, label="v_fwd (m/s)")
        self.line_tgt, = self.ax.plot([], [], "r--", linewidth=1.0, label="target")
        self.ax.set_ylabel("m/s", fontsize=7)
        # Twin axis for slip scale
        self.ax2 = self.ax.twinx()
        self.ax2.set_ylabel("slip", fontsize=7, color="orange")
        self.line_slip, = self.ax2.plot([], [], "orange", linewidth=1.0, alpha=0.7, label="slip")
        # Combined legend
        lines = [self.line_v, self.line_tgt, self.line_slip]
        labels = ["v_fwd", "target", "slip"]
        self.ax.legend(lines, labels, fontsize=6, loc="upper right")
        self.ax2.set_ylim(0.9, 21)

    def update(self, snap, bufs):
        t = list(bufs["time"])
        self.line_v.set_data(t, list(bufs["v_fwd"]))
        self.line_tgt.set_data(t, list(bufs["v_target"]))
        self.line_slip.set_data(t, list(bufs["slip_scale"]))
        if len(t) > 1:
            self.ax.set_xlim(max(0, t[-1] - 20), max(t[-1], 0.1))
        self.ax.set_ylim(-0.1, max(4.0, max(bufs["v_fwd"]) * 1.3 if bufs["v_fwd"] else 1.0))

    def reset(self):
        for line in [self.line_v, self.line_tgt, self.line_slip]:
            line.set_data([], [])


# ╔══════════════════════════════════════════════════════════════════════════════╗
# ║                              DASHBOARD GUI                                   ║
# ╚══════════════════════════════════════════════════════════════════════════════╝

# Import mpl only when this module is loaded (defer until GUI is actually needed)
import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
from matplotlib.widgets import Button, Slider
from matplotlib.gridspec import GridSpec


class Dashboard:
    """Matplotlib-based real-time dashboard with parameter sliders."""

    def __init__(self, runner: SimRunner):
        self.runner = runner

        # ── Create figure ──
        plt.ion()
        self.fig = plt.figure("Micromouse Simulation Workbench", figsize=(16, 10))
        self.fig.canvas.manager.set_window_title("Micromouse Workbench")

        # ── Layout: GridSpec ──
        #   Row 0: tracking | speed
        #   Row 1: tracking | speed
        #   Row 2: map (spans 2 cols, 2 rows)
        #   Row 3: map (cont.)
        #   Row 4: kalman | motor
        #   Row 5: kalman | motor
        #   Row 6: sliders (spans 2 cols)
        #   Row 7: status bar (spans 2 cols)
        gs = GridSpec(8, 2, figure=self.fig,
                      height_ratios=[1, 1, 1.5, 1.5, 1, 1, 1.2, 0.15],
                      hspace=0.45, wspace=0.35,
                      left=0.05, right=0.98, top=0.96, bottom=0.03)

        # ── Panels ──
        self.panels: Dict[str, MonitorPanel] = {}

        # Row 0-1 left: Tracking (rowspan 2)
        ax_track = self.fig.add_subplot(gs[0:2, 0])
        self.panels["tracking"] = TrackingPanel(ax_track)

        # Row 0-1 right: Speed (rowspan 2)
        ax_speed = self.fig.add_subplot(gs[0:2, 1])
        self.panels["speed"] = SpeedPanel(ax_speed)

        # Row 2-3 full width: Map
        ax_map = self.fig.add_subplot(gs[2:4, :])
        self.panels["map"] = MapPanel(ax_map, runner.track)

        # Row 4-5 left: Kalman
        ax_kalman = self.fig.add_subplot(gs[4:6, 0])
        self.panels["kalman"] = KalmanPanel(ax_kalman)

        # Row 4-5 right: Motor
        ax_motor = self.fig.add_subplot(gs[4:6, 1])
        self.panels["motor"] = MotorPanel(ax_motor)

        # ── Sliders (Row 6) ──
        self._build_sliders(gs[6, :])

        # ── Buttons (top of figure) ──
        self._build_buttons()

        # ── Status bar (Row 7) ──
        ax_status = self.fig.add_subplot(gs[7, :])
        ax_status.set_axis_off()
        self.status_text = ax_status.text(
            0.01, 0.5, "", transform=ax_status.transAxes,
            fontfamily="monospace", fontsize=8, verticalalignment="center")

        # ── State ──
        self._animating = False
        self._timer = None

        # ── Connect keyboard ──
        self.fig.canvas.mpl_connect("key_press_event", self._on_key)

        logger.info("Dashboard initialized")

    # ── Slider construction ────────────────────────────────────────────────

    def _build_sliders(self, gs_slider):
        """Build parameter sliders in a sub-grid."""
        gs = gs_slider.subgridspec(3, 4, hspace=0.6, wspace=0.4)

        sliders = [
            # (row, col, label, attr, vmin, vmax, vinit, fmt)
            # Row 0: 巡线控制器
            (0, 0, "lat Kp",   "vw_lat_Kp", 10.0, 400.0, self.runner.params.vw_lat_Kp, "%.0f"),
            (0, 1, "lat Ki",   "vw_lat_Ki",  0.0, 50.0,  self.runner.params.vw_lat_Ki, "%.1f"),
            (0, 2, "lat Kd",   "vw_lat_Kd",  0.0, 30.0,  self.runner.params.vw_lat_Kd, "%.1f"),
            (0, 3, "w Kp",     "vw_w_Kp",    0.0, 0.15,  self.runner.params.vw_w_Kp,   "%.3f"),
            # Row 1: v 环
            (1, 0, "v Kp",     "vw_v_Kp",    0.0, 3.0,   self.runner.params.vw_v_Kp,   "%.1f"),
            (1, 1, "v Ki",     "vw_v_Ki",    0.0, 1.0,   self.runner.params.vw_v_Ki,   "%.2f"),
            (1, 2, "Jz (×1e-4)", "vw_Jz",    1.0, 20.0,  self.runner.params.vw_Jz*1e4, "%.1f"),
            (1, 3, "Dw (x1e-3)", "vw_Dw",    0.0, 10.0,  self.runner.params.vw_Dw*1e3, "%.1f"),
            # Row 2: 速度+物理
            (2, 0, "speed m/s", "target_speed", 0.2, 5.0, self.runner.params.target_speed, "%.1f"),
            (2, 1, "w max",    "vw_w_max",   0.0, 0.1,   self.runner.params.vw_w_max,  "%.3f"),
            (2, 2, "v max",    "vw_v_max",   0.0, 0.1,   self.runner.params.vw_v_max,  "%.3f"),
            (2, 3, "k_slip",   "k_slip",     0.1, 5.0,   self.runner.params.k_slip,    "%.1f"),
        ]

        self._slider_widgets = {}
        for row, col, label, attr, vmin, vmax, vinit, fmt in sliders:
            ax = self.fig.add_subplot(gs[row, col])
            slider = Slider(ax, label, vmin, vmax, valinit=vinit, valfmt=fmt)
            slider.label.set_size(7)
            slider.valtext.set_size(6)

            # Capture attr by closure
            def make_callback(a):
                def cb(val):
                    # Handle scaled sliders: store actual un-scaled value
                    actual = val
                    if a == "vw_Jz":
                        actual = val * 1e-4
                    elif a == "vw_Dw":
                        actual = val * 1e-3
                    setattr(self.runner.params, a, actual)
                    self._on_param_changed(a, val)
                return cb

            slider.on_changed(make_callback(attr))
            self._slider_widgets[attr] = slider

    def _on_param_changed(self, attr: str, val: float):
        """Push parameter change to the appropriate C++ core."""
        vw_params = {"vw_Jz", "vw_Dw", "vw_w_Kp", "vw_w_Ki", "vw_w_Kd", "vw_w_max",
                     "vw_beta_w", "vw_kaw_w",
                     "vw_m_eq", "vw_D_v", "vw_C_frict",
                     "vw_v_Kp", "vw_v_Ki", "vw_v_max", "vw_beta_v", "vw_kaw_v",
                     "vw_lat_Kp", "vw_lat_Ki", "vw_lat_Kd",
                     "vw_w_Cfrict", "motor_I_peak"}
        lat_gains = {"lat_Kp", "lat_Kd", "lat_Ki", "lat_Kff"}
        spd_gains = {"spd_Kp", "spd_Ki"}
        kalman = {"sigma_accel", "sigma_enc_dist"}
        slip = {"thresh_lon", "thresh_lat", "k_slip"}

        if attr in vw_params:
            self.runner.push_vw_params()
        elif attr in lat_gains:
            self.runner.push_lateral_gains()
        elif attr in spd_gains:
            self.runner.push_speed_gains()
        elif attr in kalman:
            self.runner.push_kalman_params()
        elif attr in slip:
            self.runner.push_slip_params()

    # ── Buttons ────────────────────────────────────────────────────────────

    def _build_buttons(self):
        """Add Reset, Pause, Save, Load buttons at top."""
        btn_height = 0.03
        btn_width = 0.06
        y_pos = 0.975

        # Reset button
        ax_reset = self.fig.add_axes([0.12, y_pos, btn_width, btn_height])
        self.btn_reset = Button(ax_reset, "Reset (R)")
        self.btn_reset.on_clicked(self._on_reset)
        self.btn_reset.label.set_size(7)

        # Pause button
        ax_pause = self.fig.add_axes([0.20, y_pos, btn_width, btn_height])
        self.btn_pause = Button(ax_pause, "Pause (Spc)")
        self.btn_pause.on_clicked(self._on_pause)
        self.btn_pause.label.set_size(7)

        # Save button
        ax_save = self.fig.add_axes([0.28, y_pos, btn_width, btn_height])
        self.btn_save = Button(ax_save, "Save (C-S)")
        self.btn_save.on_clicked(self._on_save)
        self.btn_save.label.set_size(7)

        # Load button
        ax_load = self.fig.add_axes([0.36, y_pos, btn_width, btn_height])
        self.btn_load = Button(ax_load, "Load (C-L)")
        self.btn_load.on_clicked(self._on_load)
        self.btn_load.label.set_size(7)

    def _on_reset(self, event=None):
        logger.info("Reset requested")
        self.runner.request_reset()
        for panel in self.panels.values():
            panel.reset()

    def _on_pause(self, event=None):
        paused = self.runner.toggle_pause()
        self.btn_pause.label.set_text("Resume (Spc)" if paused else "Pause (Spc)")

    def _on_save(self, event=None):
        import tkinter.simpledialog as sd
        import tkinter as tk
        root = tk.Tk(); root.withdraw()
        name = sd.askstring("Save Preset", "Preset name:", parent=root)
        root.destroy()
        if name:
            PresetManager.save(self.runner.params, name)

    def _on_load(self, event=None):
        presets = PresetManager.list_presets()
        if not presets:
            logger.warning("No presets found")
            return
        import tkinter.simpledialog as sd
        import tkinter as tk
        root = tk.Tk(); root.withdraw()
        name = sd.askstring("Load Preset", f"Available: {', '.join(presets[:10])}\nEnter name:", parent=root)
        root.destroy()
        if name:
            params = PresetManager.load(name)
            if params:
                self.runner.params = params
                self.runner.request_reset()
                for panel in self.panels.values():
                    panel.reset()
                self._sync_sliders_from_params()
                logger.info("Loaded preset: %s", name)

    def _sync_sliders_from_params(self):
        """Update all slider positions to match current params."""
        for attr, slider in self._slider_widgets.items():
            val = getattr(self.runner.params, attr)
            # Handle scaled sliders
            if attr == "vw_Jz":
                val = val * 1e4
            elif attr == "vw_Dw":
                val = val * 1e3
            slider.set_val(val)

    # ── Keyboard handling ──────────────────────────────────────────────────

    def _on_key(self, event):
        if event.key == " ":
            self._on_pause()
        elif event.key.lower() == "r":
            self._on_reset()
        elif event.key.lower() == "q" or event.key == "escape":
            self.stop()
        elif event.key == "f12":
            path = PROJECT_ROOT / f"screenshot_{time.strftime('%Y%m%d_%H%M%S')}.png"
            self.fig.savefig(path, dpi=150)
            logger.info("Screenshot saved: %s", path)
        elif event.key == "ctrl+s":
            self._on_save()
        elif event.key == "ctrl+l":
            self._on_load()

    # ── Main loop ──────────────────────────────────────────────────────────

    def start(self):
        """Start the dashboard refresh loop."""
        self._animating = True
        self._timer = self.fig.canvas.new_timer(interval=33)  # ~30 Hz
        self._timer.add_callback(self._update)
        self._timer.start()
        logger.info("Dashboard running — press Space to pause, R to reset, Q to quit")
        plt.show(block=True)

    def stop(self):
        """Stop dashboard and simulation."""
        self._animating = False
        if self._timer:
            self._timer.stop()
        self.runner.stop()
        plt.close("all")
        logger.info("Dashboard stopped")

    def _update(self):
        """Called at 30 Hz by the matplotlib timer."""
        if not self._animating or not self.runner._running:
            return

        try:
            snap = self.runner.get_snapshot()
            bufs = snap.pop("bufs", {})

            # Update all panels
            for panel in self.panels.values():
                panel.update(snap, bufs)

            # Update status bar
            if snap:
                status = (
                    f"t={snap.get('sim_time', 0):.1f}s  "
                    f"v={snap.get('v_fwd_gt', 0):.2f}m/s  "
                    f"lat={snap.get('lateral_error_mm', 0):.0f}mm  "
                    f"lap={snap.get('lap_count', 0)}  "
                    f"loc_err={np.sqrt((snap.get('x_est',0)-snap.get('x_gt',0))**2 + (snap.get('y_est',0)-snap.get('y_gt',0))**2)*1000:.1f}mm  "
                    f"innov={snap.get('innovation', 0)*1000:.2f}mm  "
                    f"bias={snap.get('accel_bias', 0):.3f}m/s²  "
                    f"slip={snap.get('slip_scale', 1):.1f}  "
                    f"{'[E-STOP! Press R] ' if snap.get('estopped') else ''}"
                    f"{'[LINE LOST!] ' if snap.get('line_lost') and not snap.get('estopped') else ''}"
                    f"{'[PAUSED]' if snap.get('paused') else '[RUNNING]' if not snap.get('estopped') else '[STOPPED]'}"
                )
                self.status_text.set_text(status)

            self.fig.canvas.draw_idle()

        except Exception:
            logger.exception("Dashboard update error")


# ╔══════════════════════════════════════════════════════════════════════════════╗
# ║                                 MAIN                                         ║
# ╚══════════════════════════════════════════════════════════════════════════════╝

def main():
    parser = argparse.ArgumentParser(description="Micromouse Simulation Workbench")
    parser.add_argument("--track", type=str, default=None)
    parser.add_argument("--speed", type=float, default=None)
    parser.add_argument("--preset", type=str, default=None, help="Load parameter preset")
    parser.add_argument("--no-render", action="store_true", help="Headless batch mode")
    parser.add_argument("--duration", type=float, default=30.0)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--ideal", action="store_true", help="Bypass motor model")
    parser.add_argument("--with-viewer", action="store_true",
                        help="Show MuJoCo 3D viewer alongside dashboard")
    parser.add_argument("--viewer-only", action="store_true",
                        help="Only show MuJoCo 3D viewer (no dashboard, for batch-ish runs)")
    parser.add_argument("--no-viewer", action="store_true",
                        help="Dashboard only, no 3D viewer (default)")
    parser.add_argument("--surface-bumps", action="store_true",
                        help="Enable track surface bumps")
    parser.add_argument("--friction-var", action="store_true",
                        help="Enable non-uniform friction")
    parser.add_argument("--friction-degrade", action="store_true",
                        help="Enable friction degradation (wheel dust)")
    parser.add_argument("--act-delay-us", type=float, default=None,
                        help="Actuation command delay in microseconds")
    args = parser.parse_args()

    # ── Load parameters ──
    if args.preset:
        params = PresetManager.load(args.preset)
        if params is None:
            logger.error("Preset '%s' not found. Using defaults.", args.preset)
            params = SimParams()
    else:
        params = SimParams()

    # Override from CLI (only when user explicitly provides the flag)
    if args.track is not None:
        params.track = args.track
    if args.speed is not None:
        params.target_speed = args.speed
    params.ideal_motor = args.ideal
    if args.surface_bumps:   params.surface_bumps = True
    if args.friction_var:    params.friction_var = True
    if args.friction_degrade: params.friction_degrade = True
    if args.act_delay_us is not None:
        params.act_delay_us = args.act_delay_us

    # ── Create SimRunner ──
    show_viewer = args.with_viewer or args.viewer_only
    runner = SimRunner(params, seed=args.seed, with_viewer=show_viewer)
    runner.start()

    # Wait for simulation to initialize
    time.sleep(1.0)
    if not runner._running:
        logger.error("Simulation failed to start")
        sys.exit(1)

    # ── Viewer-only mode (MuJoCo 3D, no dashboard) ──
    if args.viewer_only:
        logger.info("Viewer-only mode: running for %.1f seconds (Ctrl+C to stop)", args.duration)
        try:
            while runner._running and runner.sim_time < args.duration:
                time.sleep(0.1)
        except KeyboardInterrupt:
            pass
        finally:
            runner.stop()
        logger.info("Done: laps=%d, max_lat=%.0fmm", runner.lap_count, runner.max_lat * 1000)
        return

    # ── Headless mode ──
    if args.no_render:
        logger.info("Headless mode: running for %.1f seconds", args.duration)
        try:
            while runner._running and runner.sim_time < args.duration:
                time.sleep(0.1)
        except KeyboardInterrupt:
            pass
        finally:
            runner.stop()
        logger.info("Done: laps=%d, max_lat=%.0fmm", runner.lap_count, runner.max_lat * 1000)
        return

    # ── GUI mode ──
    try:
        dashboard = Dashboard(runner)
        dashboard.start()
    except KeyboardInterrupt:
        pass
    finally:
        runner.stop()
        logger.info("Workbench closed")


if __name__ == "__main__":
    main()
