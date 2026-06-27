#!/usr/bin/env python3
"""
Headless evaluation script for the v/ω decoupled controller.

Measures what matters for single-variable iteration:
  - max/mean lateral error while line detected
  - line loss time (or "never lost")
  - v tracking: mean speed, RMS error vs target
  - ω tracking: RMS error ω_ref vs ω_meas
  - τ saturation ratio: fraction of time at torque limit
  - lap count

Control loop mirrors workbench.py SimRunner._run() exactly.
Any change to the control path MUST be reflected in both files.

Usage:
    python scripts/eval_vw.py --track 2019kansai --speed 3.5 --duration 12
    python scripts/eval_vw.py --sweep --track 2019kansai --speed-min 1.0 --speed-max 5.0 --step 0.5
    python scripts/eval_vw.py --omega-step --track 2019kansai --speed 1.0
"""

from __future__ import annotations

import argparse
import os
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import numpy as np

# Must be before any matplotlib import (workbench imports mpl at module level)
os.environ["MPLBACKEND"] = "Agg"

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
    VW_PARAM_NAMES,
    push_vw_params as push_vw_controller_params,
)
from micromouse_sim.sensors.line_sensor import LineSensor, LineSensorConfig
from micromouse_sim import localize_core, control_core

# ── Paths ───────────────────────────────────────────────────────────────────
BASE_XML = PROJECT_ROOT / "mujoco_models" / "micromouse" / "base.xml"
_TRACK_DIR = Path(r"C:\Users\chj15\Desktop\RobotRace\路径优化\robotrace-shortcut-path-main\data")
if not _TRACK_DIR.exists():
    _TRACK_DIR = PROJECT_ROOT.parent / "路径优化" / "robotrace-shortcut-path-main" / "data"
TRACK_DATA_DIR = _TRACK_DIR

# ── Timing constants (mirror workbench.py) ──────────────────────────────────
PHYSICS_DT = 2e-5       # 50kHz
IMU_RATE    = 5000      # Hz
IMU_STEPS   = int(1.0 / (IMU_RATE * PHYSICS_DT))   # 10
IMU_DT      = 1.0 / IMU_RATE                        # 200 µs
CTRL_RATE   = 1000      # Hz
CTRL_STEPS  = int(1.0 / (CTRL_RATE * PHYSICS_DT))  # 50
CTRL_DT     = 1.0 / CTRL_RATE                        # 1 ms


# ═══════════════════════════════════════════════════════════════════════════════
# Parameter dataclass (subset of workbench.SimParams needed for headless eval)
# ═══════════════════════════════════════════════════════════════════════════════

@dataclass
class EvalParams:
    """Subset of SimParams needed for headless evaluation."""
    track: str = "2019kansai"
    target_speed: float = 3.5
    track_width: float = 0.20

    # v-ω controller
    vw_wheel_r: float = VW_DEFAULTS.vw_wheel_r
    vw_track_B: float = VW_DEFAULTS.vw_track_B
    vw_Jz: float = VW_DEFAULTS.vw_Jz
    vw_Dw: float = VW_DEFAULTS.vw_Dw
    vw_w_Kp: float = VW_DEFAULTS.vw_w_Kp
    vw_w_Ki: float = VW_DEFAULTS.vw_w_Ki
    vw_w_Kd: float = VW_DEFAULTS.vw_w_Kd
    vw_w_max: float = VW_DEFAULTS.vw_w_max
    vw_beta_w: float = VW_DEFAULTS.vw_beta_w
    vw_kaw_w: float = VW_DEFAULTS.vw_kaw_w
    vw_m_eq: float = VW_DEFAULTS.vw_m_eq
    vw_D_v: float = VW_DEFAULTS.vw_D_v
    vw_C_frict: float = VW_DEFAULTS.vw_C_frict
    vw_v_Kp: float = VW_DEFAULTS.vw_v_Kp
    vw_v_Ki: float = VW_DEFAULTS.vw_v_Ki
    vw_v_max: float = VW_DEFAULTS.vw_v_max
    vw_beta_v: float = VW_DEFAULTS.vw_beta_v
    vw_kaw_v: float = VW_DEFAULTS.vw_kaw_v
    vw_lat_Kp: float = VW_DEFAULTS.vw_lat_Kp
    vw_lat_Ki: float = VW_DEFAULTS.vw_lat_Ki
    vw_lat_Kd: float = VW_DEFAULTS.vw_lat_Kd
    vw_w_Cfrict: float = VW_DEFAULTS.vw_w_Cfrict
    motor_I_peak: float = VW_DEFAULTS.motor_I_peak

    # Kalman
    sigma_accel: float = 0.01
    sigma_enc_dist: float = 2.5e-4

    # Slip
    thresh_lon: float = 0.10
    thresh_lat: float = 0.5
    k_slip: float = 0.3

    # Physics
    downforce: float = 6.0
    skirt_mu: float = 0.05
    skirt_R: float = 0.03

    # Motor
    ideal_motor: bool = False

    # Line sensor
    sensor_n_leds: int = 16
    sensor_half_span: float = 0.070
    sensor_line_width: float = 0.020
    sensor_fwd_offset: float = 0.040

    # Realism
    act_delay_us: float = 0.0


# ═══════════════════════════════════════════════════════════════════════════════
# Core evaluation
# ═══════════════════════════════════════════════════════════════════════════════

@dataclass
class EvalResult:
    """Structured evaluation metrics."""
    track: str
    target_speed: float
    duration: float
    # Lateral tracking
    max_lat_mm: float           # max |lateral_error| while line detected (mm)
    mean_lat_mm: float          # mean |lateral_error| while detected (mm)
    rms_lat_mm: float           # RMS lateral error (mm)
    # Line loss
    line_lost: bool
    line_lost_time: Optional[float]  # None if never lost
    # Velocity
    mean_v: float               # mean forward velocity (m/s)
    max_v: float                # max forward velocity
    rms_v_err: float            # RMS(v - target)
    # Omega
    rms_w_err: float            # RMS(ω_ref - ω_meas) rad/s
    max_w_z: float              # max |w_z| (rad/s)
    # Torque saturation
    tau_v_sat_frac: float       # fraction of 1kHz ticks where |τ_v| hit limit
    tau_w_sat_frac: float       # fraction where |τ_ω| hit limit
    # Progress
    lap_count: int
    max_distance_m: float       # max arc-length reached on track
    track_total_m: float        # total track length

    def summary(self) -> str:
        lines = [
            f"{'='*60}",
            f"  EVALUATION RESULTS",
            f"{'='*60}",
            f"  Track:       {self.track} ({self.track_total_m:.1f}m)",
            f"  Target speed:{self.target_speed:.2f} m/s",
            f"  Duration:    {self.duration:.1f}s",
            f"  Laps:        {self.lap_count}",
            f"  Max distance:{self.max_distance_m:.1f}m",
            f"{'-'*60}",
        ]
        if self.line_lost:
            lines.append(f"  LINE LOST at t={self.line_lost_time:.2f}s")
        else:
            lines.append(f"  LINE: NEVER LOST [OK]")
        lines.extend([
            f"{'-'*60}",
            f"  Lateral error:",
            f"    max |lat| = {self.max_lat_mm:.1f} mm",
            f"    mean |lat|= {self.mean_lat_mm:.1f} mm",
            f"    RMS lat   = {self.rms_lat_mm:.1f} mm",
            f"{'-'*60}",
            f"  Velocity:",
            f"    mean v = {self.mean_v:.2f} m/s  (target {self.target_speed:.2f})",
            f"    max v  = {self.max_v:.2f} m/s",
            f"    RMS err = {self.rms_v_err:.3f} m/s",
            f"{'-'*60}",
            f"  Omega:",
            f"    RMS err = {self.rms_w_err:.3f} rad/s",
            f"    max |w_z| = {self.max_w_z:.1f} rad/s",
            f"{'-'*60}",
            f"  Torque saturation:",
            f"    tau_v sat = {self.tau_v_sat_frac*100:.1f}%",
            f"    tau_w sat = {self.tau_w_sat_frac*100:.1f}%",
            f"{'='*60}",
        ])
        return "\n".join(lines)


def run_eval(params: EvalParams, duration: float, seed: int = 42,
             verbose: bool = True) -> EvalResult:
    """
    Run a single headless evaluation with the v/ω controller.

    Control loop mirrors workbench.py SimRunner._run() — keep in sync.
    """
    # ── Init simulation ──
    track = load_track(str(TRACK_DATA_DIR / f"{params.track}_points.txt"))
    if verbose:
        print(f"Track: {track.total_length:.2f}m, {track.waypoints.shape[0]} pts")

    model_xml = build_model_xml(str(BASE_XML), track=track, track_width=params.track_width)

    engine = PhysicsEngine(
        model_xml=model_xml,
        downforce=params.downforce,
        skirt_mu=params.skirt_mu,
        skirt_R=params.skirt_R,
    )

    motor_L = MotorModel()
    motor_R = MotorModel()
    line_sensor = LineSensor(LineSensorConfig(
        n_leds=params.sensor_n_leds,
        half_span=params.sensor_half_span,
        line_width=params.sensor_line_width,
        fwd_offset=params.sensor_fwd_offset,
    ))

    # ── Settle + calibrate gyro ──
    for _ in range(5000):
        engine.step()
    gyro_samples = []
    for _ in range(10000):
        engine.step()
        gyro_samples.append(engine.get_state().gyro[2])
    gyro_bias_init = float(np.mean(gyro_samples))
    if verbose:
        print(f"Gyro bias: {gyro_bias_init:.6f} rad/s")

    # ── Init C++ cores ──
    localize_core.reset(seed)
    control_core.reset()
    control_core.vw_reset()

    # Push calibration
    localize_core.set_calibration(
        pulses_per_m_L=PULSES_PER_M,
        pulses_per_m_R=PULSES_PER_M,
        accel_noise_std=params.sigma_accel,
        enc_dist_noise=params.sigma_enc_dist,
        track_width=TRACK_WIDTH,
        gyro_bias_init=gyro_bias_init,
    )
    localize_core.set_slip_params(params.thresh_lon, params.thresh_lat, params.k_slip)
    push_vw_controller_params(control_core, params)

    # ── Record origin ──
    st = engine.get_state()
    origin_x = float(st.pos[0])
    origin_y = float(st.pos[1])

    # ── Loop state ──
    u_L, u_R = 0.0, 0.0
    gyro_acc  = np.zeros(3)
    accel_acc = np.zeros(3)
    imu_raw_count = 0
    step_count = 0
    target_speed = params.target_speed
    act_delay = ActuationDelayBuffer(params.act_delay_us, PHYSICS_DT)

    # Metrics accumulators
    lat_errors_mm: list = []
    v_fwd_vals: list = []
    v_ref_vals: list = []
    w_errors: list = []     # ω_ref - ω_meas
    w_z_vals: list = []
    tau_v_sat_count = 0
    tau_w_sat_count = 0
    ctrl_tick_count = 0
    line_lost_time: Optional[float] = None
    max_distance = 0.0
    lap_count = 0
    prev_s = -1.0
    estopped = False
    lat_filtered = 0.0
    lat_ema_alpha = 0.25  # match workbench default

    # ── Main loop ──
    total_physics_steps = int(duration / PHYSICS_DT)

    for _ in range(total_physics_steps):
        # ── (1) Motor torque ──
        st = engine.get_state()
        u_delayed_L, u_delayed_R = act_delay.apply(u_L, u_R)
        if params.ideal_motor:
            tau_L_cmd = u_delayed_L * 0.05
            tau_R_cmd = u_delayed_R * 0.05
        else:
            tau_L_cmd = motor_L.compute_torque(u_delayed_L, st.wheel_L_vel, PHYSICS_DT)
            tau_R_cmd = motor_R.compute_torque(u_delayed_R, st.wheel_R_vel, PHYSICS_DT)
        engine.set_control(tau_L_cmd, tau_R_cmd)

        # ── (2) Physics step ──
        engine.step()
        st = engine.get_state()

        # ── (3) IMU accumulation ──
        gyro_acc  += st.gyro
        accel_acc += st.accel
        imu_raw_count += 1

        # ── (4) 5kHz pipeline + v-ω omega step ──
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
            control_core.vw_omega_step(float(noisy["gyro"][2]), IMU_DT)
            gyro_acc[:] = 0.0; accel_acc[:] = 0.0
            imu_raw_count = 0

        # ── (5) 1kHz encoder push + control ──
        if step_in_ctrl == 0:
            ctrl_tick_count += 1
            sim_time = st.time

            localize_core.push_encoder(st.wheel_L_pos, st.wheel_R_pos, CTRL_DT)
            pose = localize_core.read_pose()

            # Line sensor
            reading = line_sensor.read(st.pos[:2], st.yaw, track)

            if not reading.line_detected:
                if not estopped:
                    if verbose:
                        print(f"  LINE LOST at t={sim_time:.2f}s")
                    if line_lost_time is None:
                        line_lost_time = sim_time
                    estopped = True
                u_L, u_R = 0.0, 0.0
                lateral_error = 0.0
            else:
                estopped = False
                raw_lat = reading.lateral_error if reading.lateral_error is not None else 0.0
                lat_filtered += lat_ema_alpha * (raw_lat - lat_filtered)
                lateral_error = lat_filtered

                # Lap detection
                s_pos = reading.s_pos if reading.s_pos is not None else -1.0
                if s_pos > 0:
                    total = track.total_length
                    finish_zone = total * 0.90
                    start_zone = total * 0.10
                    if prev_s > finish_zone and s_pos < start_zone:
                        lap_count += 1
                        if verbose:
                            print(f"  LAP {lap_count} @ t={sim_time:.1f}s")
                    prev_s = s_pos
                    max_distance = max(max_distance, s_pos)

                # v-ω control (Kalman estimate)
                control_core.vw_set_wheel_omega(st.wheel_L_vel, st.wheel_R_vel)
                cmd = control_core.vw_control_tick(
                    lateral_error, 0.0,  # curvature=0 (pure photodiode)
                    float(pose["v_fwd"]), params.target_speed, CTRL_DT,
                )
                u_L = float(cmd["u_L"])
                u_R = float(cmd["u_R"])

                # ── Collect metrics ──
                vw_dbg = control_core.vw_get_debug()
                lat_errors_mm.append(lateral_error * 1000)
                v_fwd_vals.append(st.forward_velocity)
                v_ref_vals.append(params.target_speed)
                w_errors.append(float(vw_dbg["omega_ref"]) - float(vw_dbg["omega_meas"]))
                w_z_vals.append(float(pose["w_z"]))

                tau_v = float(vw_dbg["tau_v"])
                tau_w = float(vw_dbg["tau_omega"])
                if bool(vw_dbg.get("sat_v", False)) or abs(tau_v) >= params.vw_v_max * 0.99:
                    tau_v_sat_count += 1
                if bool(vw_dbg.get("sat_w", False)) or abs(tau_w) >= params.vw_w_max * 0.99:
                    tau_w_sat_count += 1

        step_count += 1

    # ── Build result ──
    la = np.array(lat_errors_mm) if lat_errors_mm else np.array([0.0])
    va = np.array(v_fwd_vals) if v_fwd_vals else np.array([0.0])
    wa = np.array(w_errors) if w_errors else np.array([0.0])
    wz = np.array(w_z_vals) if w_z_vals else np.array([0.0])

    return EvalResult(
        track=params.track,
        target_speed=params.target_speed,
        duration=duration,
        max_lat_mm=float(np.max(np.abs(la))),
        mean_lat_mm=float(np.mean(np.abs(la))),
        rms_lat_mm=float(np.sqrt(np.mean(la**2))),
        line_lost=line_lost_time is not None,
        line_lost_time=line_lost_time,
        mean_v=float(np.mean(va)),
        max_v=float(np.max(va)),
        rms_v_err=float(np.sqrt(np.mean((va - params.target_speed)**2))),
        rms_w_err=float(np.sqrt(np.mean(wa**2))),
        max_w_z=float(np.max(np.abs(wz))),
        tau_v_sat_frac=tau_v_sat_count / max(ctrl_tick_count, 1),
        tau_w_sat_frac=tau_w_sat_count / max(ctrl_tick_count, 1),
        lap_count=lap_count,
        max_distance_m=max_distance,
        track_total_m=track.total_length,
    )


# ═══════════════════════════════════════════════════════════════════════════════
# CLI entry points
# ═══════════════════════════════════════════════════════════════════════════════

def _apply_cli_overrides(params: EvalParams, args) -> EvalParams:
    for key in [*VW_PARAM_NAMES, "downforce", "skirt_mu", "act_delay_us"]:
        if hasattr(args, key) and getattr(args, key) is not None:
            setattr(params, key, getattr(args, key))
    return params


def cmd_single(args):
    """Run a single evaluation at fixed speed."""
    params = EvalParams(
        track=args.track,
        target_speed=args.speed,
        downforce=args.downforce,
        skirt_mu=args.skirt_mu,
        ideal_motor=args.ideal,
    )
    _apply_cli_overrides(params, args)
    _apply_cli_overrides(params, args)

    result = run_eval(params, args.duration, seed=args.seed)
    print(result.summary())

    # Machine-readable one-liner
    status = "LOST" if result.line_lost else "OK"
    print(f"JSON: {{"
          f"\"status\":\"{status}\", "
          f"\"speed\":{result.target_speed}, "
          f"\"max_lat_mm\":{result.max_lat_mm:.1f}, "
          f"\"mean_lat_mm\":{result.mean_lat_mm:.1f}, "
          f"\"rms_lat_mm\":{result.rms_lat_mm:.1f}, "
          f"\"line_lost_t\":{result.line_lost_time if result.line_lost_time else 'null'}, "
          f"\"mean_v\":{result.mean_v:.2f}, "
          f"\"rms_v_err\":{result.rms_v_err:.3f}, "
          f"\"rms_w_err\":{result.rms_w_err:.3f}, "
          f"\"max_w_z\":{result.max_w_z:.1f}, "
          f"\"tau_v_sat%\":{result.tau_v_sat_frac*100:.1f}, "
          f"\"tau_w_sat%\":{result.tau_w_sat_frac*100:.1f}, "
          f"\"laps\":{result.lap_count}"
          f"}}")

    return 0 if not result.line_lost else 1


def cmd_sweep(args):
    """Sweep speed from min to max, find max survivable constant speed."""
    speeds = np.arange(args.speed_min, args.speed_max + args.step * 0.5, args.step)
    results = []
    for spd in speeds:
        print(f"\n{'-'*40}\n  Speed = {spd:.2f} m/s\n{'-'*40}")
        params = EvalParams(
            track=args.track,
            target_speed=float(spd),
            downforce=args.downforce,
            skirt_mu=args.skirt_mu,
            ideal_motor=args.ideal,
        )
        _apply_cli_overrides(params, args)
        params.target_speed = float(spd)
        result = run_eval(params, args.duration, seed=args.seed, verbose=False)
        print(f"  {result.max_lat_mm:.0f}mm max lat, "
              f"{'LOST' if result.line_lost else 'OK'} "
              f"@ t={result.line_lost_time if result.line_lost_time else result.duration:.1f}s, "
              f"{result.lap_count} laps")
        results.append(result)

    # Summary table
    print(f"\n{'='*70}")
    print(f"  SPEED SWEEP SUMMARY — {args.track}")
    print(f"{'='*70}")
    print(f"  {'Speed':>6s}  {'Status':>5s}  {'Max|lat|':>8s}  {'Mean|lat|':>9s}  {'v mean':>7s}  {'τ_v sat':>7s}  {'τ_ω sat':>7s}  {'Laps':>4s}")
    print(f"  {'-'*6}  {'-'*5}  {'-'*8}  {'-'*9}  {'-'*7}  {'-'*7}  {'-'*7}  {'-'*4}")
    for r in results:
        status = "LOST" if r.line_lost else "  OK"
        print(f"  {r.target_speed:6.2f}  {status:>5s}  {r.max_lat_mm:7.1f}mm  {r.mean_lat_mm:8.1f}mm  {r.mean_v:6.2f}m/s  {r.tau_v_sat_frac*100:6.1f}%  {r.tau_w_sat_frac*100:6.1f}%  {r.lap_count:4d}")

    # Find max survivable
    survivors = [r for r in results if not r.line_lost]
    if survivors:
        best = max(survivors, key=lambda r: r.target_speed)
        print(f"\n  ** Max survivable constant speed: {best.target_speed:.2f} m/s")
        print(f"    max|lat|={best.max_lat_mm:.0f}mm, {best.lap_count} laps")

        # Friction-circle theoretical bound
        # a_y = v²·κ_max ≤ μ·F_N/m_eq
        # Track curvature bound: we'd need to scan the track for max κ
        print(f"    (friction-circle upper bound requires track κ_max scan — add later)")
    else:
        print(f"\n  ** No speed survived! Lowest speed {speeds[0]:.2f} m/s already lost line.")

    return 0


def cmd_omega_step(args):
    """Inject an ω_ref step while running straight, measure ω-loop response."""
    print("Omega-step test: running at fixed speed, injecting ω_ref step...")

    params = EvalParams(
        track=args.track,
        target_speed=args.speed,
        downforce=args.downforce,
        skirt_mu=args.skirt_mu,
        ideal_motor=args.ideal,
    )

    # For omega-step, we need a modified run that injects the step
    track = load_track(str(TRACK_DATA_DIR / f"{params.track}_points.txt"))
    model_xml = build_model_xml(str(BASE_XML), track=track, track_width=params.track_width)

    engine = PhysicsEngine(model_xml=model_xml, downforce=params.downforce,
                           skirt_mu=params.skirt_mu, skirt_R=params.skirt_R)
    motor_L = MotorModel(); motor_R = MotorModel()
    line_sensor = LineSensor(LineSensorConfig(
        n_leds=params.sensor_n_leds, half_span=params.sensor_half_span,
        line_width=params.sensor_line_width, fwd_offset=params.sensor_fwd_offset))

    # Settle
    for _ in range(5000): engine.step()
    gyro_bias = float(np.mean([engine.get_state().gyro[2] for _ in range(10000) if not engine.step()]))

    # Init cores
    localize_core.reset(args.seed); control_core.reset(); control_core.vw_reset()
    localize_core.set_calibration(PULSES_PER_M, PULSES_PER_M, params.sigma_accel,
                                  params.sigma_enc_dist, TRACK_WIDTH, gyro_bias)
    localize_core.set_slip_params(params.thresh_lon, params.thresh_lat, params.k_slip)
    push_vw_controller_params(control_core, params)

    u_L = u_R = 0.0
    gyro_acc = np.zeros(3); accel_acc = np.zeros(3)
    imu_raw_count = 0; step_count = 0
    act_delay = ActuationDelayBuffer(params.act_delay_us, PHYSICS_DT)
    step_inject_at = int(0.5 / PHYSICS_DT)    # inject at t=0.5s (after startup ramp)
    omega_step_magnitude = args.omega_step_value  # rad/s
    step_injected = False
    records: list = []  # (t, ω_ref, ω_meas, τ_ff, τ_fb)

    total_steps = int(args.duration / PHYSICS_DT)
    for _ in range(total_steps):
        st = engine.get_state()
        u_delayed_L, u_delayed_R = act_delay.apply(u_L, u_R)
        if params.ideal_motor:
            engine.set_control(u_delayed_L * 0.05, u_delayed_R * 0.05)
        else:
            engine.set_control(
                motor_L.compute_torque(u_delayed_L, st.wheel_L_vel, PHYSICS_DT),
                motor_R.compute_torque(u_delayed_R, st.wheel_R_vel, PHYSICS_DT))
        engine.step(); st = engine.get_state()

        gyro_acc += st.gyro; accel_acc += st.accel; imu_raw_count += 1
        step_in_ctrl = step_count % CTRL_STEPS

        if step_in_ctrl % IMU_STEPS == 0 and imu_raw_count > 0:
            gyro_avg = gyro_acc / imu_raw_count; accel_avg = accel_acc / imu_raw_count
            noisy = localize_core.imu_step(gyro_avg, accel_avg, IMU_DT)
            localize_core.push_imu(float(noisy["gyro"][2]), float(noisy["accel"][0]),
                                   float(noisy["accel"][1]), IMU_DT)
            control_core.vw_omega_step(float(noisy["gyro"][2]), IMU_DT)
            gyro_acc[:] = 0.0; accel_acc[:] = 0.0; imu_raw_count = 0

        if step_in_ctrl == 0:
            localize_core.push_encoder(st.wheel_L_pos, st.wheel_R_pos, CTRL_DT)
            _pose = localize_core.read_pose()
            reading = line_sensor.read(st.pos[:2], st.yaw, track)

            if reading.line_detected and reading.lateral_error is not None:
                lateral_error = reading.lateral_error
            else:
                lateral_error = 0.0

            # Inject omega step override
            if step_count >= step_inject_at and not step_injected:
                control_core.vw_override_omega_ref(omega_step_magnitude)
                step_injected = True
                print(f"  Step injected: ω_ref = {omega_step_magnitude} rad/s @ t={st.time:.3f}s")

            control_core.vw_set_wheel_omega(st.wheel_L_vel, st.wheel_R_vel)
            cmd = control_core.vw_control_tick(
                lateral_error, 0.0, float(_pose["v_fwd"]), params.target_speed, CTRL_DT)
            u_L = float(cmd["u_L"]); u_R = float(cmd["u_R"])

            vw_dbg = control_core.vw_get_debug()
            records.append({
                't': st.time,
                'w_ref': float(vw_dbg["omega_ref"]),
                'w_meas': float(vw_dbg["omega_meas"]),
                'tau_ff': float(vw_dbg["omega_tau_ff"]),
                'tau_fb': float(vw_dbg["omega_tau_fb"]),
            })

        step_count += 1

    # ── Analyze step response ──
    control_core.vw_clear_omega_override()

    if not records:
        print("ERROR: No records collected")
        return 1

    r = {k: np.array([d[k] for d in records]) for k in records[0].keys()}
    t = r['t']; w_ref = r['w_ref']; w_meas = r['w_meas']

    # Find step onset
    step_idx = int(np.argmax(np.abs(w_ref) > abs(omega_step_magnitude) * 0.1))
    if step_idx < 1:
        print("ERROR: Step not detected in records")
        return 1

    t_step = t[step_idx:]
    w_ref_step = w_ref[step_idx:]
    w_meas_step = w_meas[step_idx:]
    t_rel = t_step - t_step[0]

    # Steady-state (last 25% of post-step data)
    ss_n = max(1, len(w_meas_step) // 4)
    ss_val = np.mean(w_meas_step[-ss_n:])
    ss_err = abs(ss_val - omega_step_magnitude) / abs(omega_step_magnitude) * 100

    # Overshoot
    peak_val = np.max(w_meas_step) if omega_step_magnitude > 0 else np.min(w_meas_step)
    overshoot_pct = (peak_val - omega_step_magnitude) / abs(omega_step_magnitude) * 100

    # Rise time (10%→90%)
    y10 = omega_step_magnitude * 0.1
    y90 = omega_step_magnitude * 0.9
    t10 = t_rel[np.argmax(np.abs(w_meas_step) >= abs(y10))]
    t90 = t_rel[np.argmax(np.abs(w_meas_step) >= abs(y90))]
    rise_time = (t90 - t10) * 1000  # ms

    print(f"\n{'='*50}")
    print(f"  OMEGA STEP RESPONSE")
    print(f"{'='*50}")
    print(f"  Step magnitude:  {omega_step_magnitude:.2f} rad/s")
    print(f"  Steady-state:    {ss_val:.3f} rad/s  (err={ss_err:.1f}%)")
    print(f"  Overshoot:       {overshoot_pct:.1f}%")
    print(f"  Rise time 10-90: {rise_time:.1f} ms")
    print(f"  Samples post-step: {len(w_meas_step)}")
    print(f"{'='*50}")

    return 0


# ═══════════════════════════════════════════════════════════════════════════════
# Main
# ═══════════════════════════════════════════════════════════════════════════════

def main():
    parser = argparse.ArgumentParser(description="v-ω Controller Evaluation")
    sub = parser.add_subparsers(dest="mode", help="Evaluation mode")

    # ── single ──
    p_single = sub.add_parser("single", help="Single fixed-speed run")
    p_single.add_argument("--track", default="2019kansai")
    p_single.add_argument("--speed", type=float, default=3.5)
    p_single.add_argument("--duration", type=float, default=12.0)
    p_single.add_argument("--seed", type=int, default=42)
    p_single.add_argument("--downforce", type=float, default=6.0)
    p_single.add_argument("--skirt-mu", type=float, default=0.05)
    p_single.add_argument("--ideal", action="store_true")
    # Quick overrides for tuning
    p_single.add_argument("--vw-lat-Kp", type=float)
    p_single.add_argument("--vw-lat-Ki", type=float)
    p_single.add_argument("--vw-lat-Kd", type=float)
    p_single.add_argument("--vw-w-Kp", type=float)
    p_single.add_argument("--vw-w-Kd", type=float)
    p_single.add_argument("--vw-w-Ki", type=float)
    p_single.add_argument("--vw-w-max", dest="vw_w_max", type=float)
    p_single.add_argument("--vw-beta-w", dest="vw_beta_w", type=float)
    p_single.add_argument("--vw-kaw-w", dest="vw_kaw_w", type=float)
    p_single.add_argument("--vw-w-Cfrict", dest="vw_w_Cfrict", type=float)
    p_single.add_argument("--vw-v-Kp", type=float)
    p_single.add_argument("--vw-v-Ki", type=float)
    p_single.add_argument("--vw-v-max", dest="vw_v_max", type=float)
    p_single.add_argument("--vw-beta-v", dest="vw_beta_v", type=float)
    p_single.add_argument("--vw-kaw-v", dest="vw_kaw_v", type=float)
    p_single.add_argument("--vw-Jz", type=float)
    p_single.add_argument("--vw-Dw", type=float)
    p_single.add_argument("--motor-I-peak", dest="motor_I_peak", type=float)
    p_single.add_argument("--act-delay-us", dest="act_delay_us", type=float)

    # ── sweep ──
    p_sweep = sub.add_parser("sweep", help="Speed sweep to find max survivable speed")
    p_sweep.add_argument("--track", default="2019kansai")
    p_sweep.add_argument("--speed-min", type=float, default=1.0)
    p_sweep.add_argument("--speed-max", type=float, default=5.0)
    p_sweep.add_argument("--step", type=float, default=0.5)
    p_sweep.add_argument("--duration", type=float, default=12.0)
    p_sweep.add_argument("--seed", type=int, default=42)
    p_sweep.add_argument("--downforce", type=float, default=6.0)
    p_sweep.add_argument("--skirt-mu", type=float, default=0.05)
    p_sweep.add_argument("--ideal", action="store_true")
    p_sweep.add_argument("--vw-beta-w", dest="vw_beta_w", type=float)
    p_sweep.add_argument("--vw-kaw-w", dest="vw_kaw_w", type=float)
    p_sweep.add_argument("--vw-w-Cfrict", dest="vw_w_Cfrict", type=float)
    p_sweep.add_argument("--motor-I-peak", dest="motor_I_peak", type=float)
    p_sweep.add_argument("--act-delay-us", dest="act_delay_us", type=float)

    # ── omega-step ──
    p_ostep = sub.add_parser("omega-step", help="Omega step response test")
    p_ostep.add_argument("--track", default="2019kansai")
    p_ostep.add_argument("--speed", type=float, default=1.0)
    p_ostep.add_argument("--duration", type=float, default=3.0)
    p_ostep.add_argument("--seed", type=int, default=42)
    p_ostep.add_argument("--omega-step-value", type=float, default=5.0,
                         help="ω_ref step magnitude (rad/s)")
    p_ostep.add_argument("--downforce", type=float, default=6.0)
    p_ostep.add_argument("--skirt-mu", type=float, default=0.05)
    p_ostep.add_argument("--ideal", action="store_true")
    p_ostep.add_argument("--vw-beta-w", dest="vw_beta_w", type=float)
    p_ostep.add_argument("--vw-kaw-w", dest="vw_kaw_w", type=float)
    p_ostep.add_argument("--vw-w-Cfrict", dest="vw_w_Cfrict", type=float)
    p_ostep.add_argument("--motor-I-peak", dest="motor_I_peak", type=float)
    p_ostep.add_argument("--act-delay-us", dest="act_delay_us", type=float)

    args = parser.parse_args()

    if args.mode == "sweep":
        sys.exit(cmd_sweep(args))
    elif args.mode == "omega-step":
        sys.exit(cmd_omega_step(args))
    elif args.mode == "single":
        sys.exit(cmd_single(args))
    else:
        # Default: single run
        sys.exit(cmd_single(args))


if __name__ == "__main__":
    main()
