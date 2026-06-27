#!/usr/bin/env python3
"""
Closed-loop micromouse simulation with C++ 5kHz pipeline localizer.

Architecture:
  Physics: 50kHz (MuJoCo)
  IMU noise injection + pipeline push: 5kHz (every 10 physics steps)
  Encoder push (Kalman update): 1kHz (every 50 physics steps)
  Control: 1kHz

Localization pipeline (C++ localize_core.pyd):
  push_imu(gz, ax, ay, dt_200us) @ 5kHz
    → pure gyro yaw integration
    → v_fwd a_x feedforward + Kalman predict
    → 5kHz position extrapolation
  push_encoder(enc_L, enc_R, dt_1ms) @ 1kHz
    → encoder common-mode distance
    → velocity-layer slip detection (no encoder 2nd derivative!)
    → Kalman Joseph update
    → position fine-correction

Usage:
    python scripts/run_sim.py --track robotena --speed 2.0
    python scripts/run_sim.py --track robotena --speed 3.0 --no-render --duration 20
"""

import argparse
import logging
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import numpy as np

from micromouse_sim.physics.engine import (
    PhysicsEngine, WHEEL_RADIUS, TRACK_WIDTH, PULSES_PER_M,
)
from micromouse_sim.environment.loader import build_model_xml, load_track
from micromouse_sim.actuation.motor_model import MotorModel
from micromouse_sim.physics.tire_deformation import TireDeformationModel

# C++ modules
from micromouse_sim import localize_core, control_core

logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s: %(message)s")
logger = logging.getLogger(__name__)

# ---- Paths ----
PROJECT_ROOT = Path(__file__).resolve().parent.parent
BASE_XML = PROJECT_ROOT / "mujoco_models" / "micromouse" / "base.xml"
_tp = Path(r"C:\Users\chj15\Desktop\RobotRace\路径优化\robotrace-shortcut-path-main\data")
if not _tp.exists():
    _tp = PROJECT_ROOT.parent / "路径优化" / "robotrace-shortcut-path-main" / "data"
TRACK_DATA_DIR = _tp

# ---- Timing ----
PHYSICS_DT   = 2e-5       # 50 kHz physics
IMU_RATE     = 5000       # Hz — pipeline IMU push rate
IMU_STEPS    = int(1.0 / (IMU_RATE * PHYSICS_DT))    # 10 physics steps per IMU push
IMU_DT       = 1.0 / IMU_RATE                         # 200 us
CTRL_RATE    = 1000       # Hz — encoder push + control rate
CTRL_STEPS   = int(1.0 / (CTRL_RATE * PHYSICS_DT))   # 50 physics steps per ctrl tick
CTRL_DT      = 1.0 / CTRL_RATE                         # 1 ms



def parse_args():
    p = argparse.ArgumentParser(description="Micromouse Closed-Loop Simulation (5kHz Pipeline)")
    p.add_argument("--track", type=str, default="robotena")
    p.add_argument("--duration", type=float, default=10.0)
    p.add_argument("--speed", type=float, default=2.0, help="Target speed m/s")
    p.add_argument("--no-render", action="store_true")
    p.add_argument("--seed", type=int, default=42)
    p.add_argument("--ideal", action="store_true",
                   help="Bypass motor model — use direct torque (for control tuning)")
    return p.parse_args()


def main():
    args = parse_args()

    # ---- Load track ----
    track = load_track(str(TRACK_DATA_DIR / f"{args.track}_points.txt"))
    logger.info("Track: %.2f m, %d pts", track.total_length, track.waypoints.shape[0])

    # ---- Build model ----
    model_xml = build_model_xml(str(BASE_XML), track=track, track_width=0.180)
    engine = PhysicsEngine(model_xml=model_xml, downforce=5.0)

    # ---- Motor models (one per wheel) ----
    motor_L = MotorModel()
    motor_R = MotorModel()

    # ---- Tire deformation (not used directly — odometry is in C++) ----
    tire_def = TireDeformationModel()

    # ---- Settle & calibrate gyro bias ----
    logger.info("Settling + gyro bias calibration (2s)...")
    gyro_z_samples = []
    for _ in range(5000):  # 5000 * 20us = 100ms settle
        engine.step()
    for _ in range(10000):  # 10000 * 20us = 200ms gyro sampling
        engine.step()
        st = engine.get_state()
        gyro_z_samples.append(st.gyro[2])
    gyro_bias_init = float(np.mean(gyro_z_samples))
    gyro_noise_std = float(np.std(gyro_z_samples))
    logger.info("Gyro bias: %.6f rad/s, noise std: %.6f rad/s", gyro_bias_init, gyro_noise_std)

    # ---- Initialize C++ cores ----
    localize_core.reset(args.seed)
    control_core.reset()

    # Calibration (simulation → real-world equivalent parameters)
    # accel_noise_std: from static accel test (ICM-42688 ~0.001 m/s^2, degraded)
    # enc_dist_noise: encoder distance noise ~0.1mm per 1ms sample
    localize_core.set_calibration(
        pulses_per_m_L=PULSES_PER_M,
        pulses_per_m_R=PULSES_PER_M,
        accel_noise_std=0.005,       # m/s^2, slightly higher than real for margin
        enc_dist_noise=1.0e-4,       # m, ~0.1mm
        track_width=TRACK_WIDTH,
        gyro_bias_init=gyro_bias_init,
    )

    # Slip detection (can tune these)
    localize_core.set_slip_params(
        thresh_lon=0.03,    # m/s velocity residual deadzone
        thresh_lat=0.5,     # m/s^2 lateral accel residual deadzone
        k_slip=5.0,         # gain
    )

    # Control gains
    control_core.set_lateral_gains(Kp=3.0, Kd=1.0, Ki=0.05, Kff=0.03)
    control_core.set_speed_gains(Kp=1.0, Ki=0.3)

    # ---- State ----
    st = engine.get_state()
    logger.info("Settled: z=%.1fmm", st.pos[2] * 1000)

    # ---- Viewer ----
    viewer = None
    if not args.no_render:
        try:
            import mujoco.viewer
            viewer = mujoco.viewer.launch_passive(
                engine.model, engine.data,
                show_left_ui=False, show_right_ui=False)
            viewer.cam.azimuth = 135; viewer.cam.elevation = -35
            viewer.cam.distance = 1.5; viewer.cam.lookat[:] = [0.3, -0.3, 0.01]
        except Exception as e:
            logger.warning("Viewer: %s", e)

    # ---- Accumulators ----
    u_L, u_R = 0.0, 0.0
    lateral_error = 0.0
    gyro_acc  = np.zeros(3)
    accel_acc = np.zeros(3)
    imu_raw_count = 0
    max_lat = 0.0
    lap_count = 0
    prev_s = -1.0
    lap_high = False

    n_total = int(args.duration / PHYSICS_DT)
    log_interval = int(0.5 / PHYSICS_DT)   # print every 0.5s

    logger.info("Running %.1fs, target %.1f m/s, pipeline @ 5kHz...", args.duration, args.speed)

    for step_i in range(n_total):
        # ---- (1) Motor torque ----
        st = engine.get_state()
        if args.ideal:
            tau_L = u_L * 0.05
            tau_R = u_R * 0.05
        else:
            tau_L = motor_L.compute_torque(u_L, st.wheel_L_vel, PHYSICS_DT)
            tau_R = motor_R.compute_torque(u_R, st.wheel_R_vel, PHYSICS_DT)
        engine.set_control(tau_L, tau_R)

        # ---- (2) Physics step ----
        engine.step()
        st = engine.get_state()

        # ---- (3) Accumulate IMU raw values (every physics step) ----
        gyro_acc  += st.gyro
        accel_acc += st.accel
        imu_raw_count += 1

        # ---- (4) 5kHz pipeline push ----
        if step_i % IMU_STEPS == 0 and imu_raw_count > 0:
            # Average IMU over this 200us window (anti-aliasing)
            gyro_avg  = gyro_acc / imu_raw_count
            accel_avg = accel_acc / imu_raw_count

            # External noise injection (simulates real sensor output)
            noisy = localize_core.imu_step(gyro_avg, accel_avg, IMU_DT)
            gyro_z  = noisy["gyro"][2]
            accel_x = noisy["accel"][0]
            accel_y = noisy["accel"][1]

            # Push to pipeline: yaw integration + Kalman predict + position extrapolation
            localize_core.push_imu(gyro_z, accel_x, accel_y, IMU_DT)

            gyro_acc[:] = 0.0
            accel_acc[:] = 0.0
            imu_raw_count = 0

        # ---- (5) 1kHz encoder push + control ----
        if step_i % CTRL_STEPS == 0:
            # Encoder push → Kalman update
            localize_core.push_encoder(st.wheel_L_pos, st.wheel_R_pos, CTRL_DT)

            # Read latest pose from pipeline
            pose = localize_core.read_pose()

            # ---- Line sensor: project lookahead point onto spline ----
            lx = st.pos[0] + 0.050 * np.cos(st.yaw)
            ly = st.pos[1] + 0.050 * np.sin(st.yaw)
            try:
                s_pos, lateral_error, _ = track.project(np.array([lx, ly]))
                curvature = track.curvature_at(s_pos)
                # Lap detection
                half_t = track.total_length * 0.5
                if prev_s > 0:
                    if prev_s < half_t and s_pos > half_t:
                        lap_high = True
                    elif prev_s > half_t and s_pos < half_t and lap_high:
                        lap_count += 1
                        lap_high = False
                        logger.info("LAP %d @ t=%.1fs", lap_count, st.time)
                prev_s = s_pos
            except Exception:
                lateral_error = 0.0
                curvature = 0.0

            # ---- Control ----
            cmd = control_core.step(lateral_error, curvature,
                                    st.forward_velocity, args.speed, CTRL_DT)
            u_L = cmd["u_L"]
            u_R = cmd["u_R"]
            max_lat = max(max_lat, abs(lateral_error))

        # ---- (6) Logging ----
        if step_i % log_interval == 0:
            pose = localize_core.read_pose()
            loc_x, loc_y = pose["x"], pose["y"]
            loc_err = np.sqrt((loc_x - st.pos[0])**2 + (loc_y - st.pos[1])**2)
            dbg = localize_core.get_debug_state()
            logger.info(
                "t=%.1f v=%.1f lat=%.0fmm u=(%+.2f,%+.2f) | "
                "loc=(%.3f,%.3f) err=%.3fm | "
                "v_fwd=%.2f bias=%.4f innov=%.4f slip=%.1f",
                st.time, st.forward_velocity, lateral_error * 1000,
                u_L, u_R,
                loc_x, loc_y, loc_err,
                dbg["v_fwd"], dbg["accel_bias"], dbg["innovation"], dbg["slip_lon"],
            )

        # ---- (7) Render (follow-cam) ----
        if viewer is not None and viewer.is_running():
            st = engine.get_state()
            viewer.cam.lookat[:] = [st.pos[0], st.pos[1], 0.02]
            viewer.sync()
        elif viewer is not None and not viewer.is_running():
            break

    # ---- Summary ----
    st = engine.get_state()
    pose = localize_core.read_pose()
    loc_err = np.sqrt((pose["x"] - st.pos[0])**2 + (pose["y"] - st.pos[1])**2)
    logger.info("DONE: v=%.1f lat_max=%.0fmm laps=%d loc_err=%.3fm",
                st.forward_velocity, max_lat * 1000, lap_count, loc_err)
    if viewer:
        viewer.close()


if __name__ == "__main__":
    main()
