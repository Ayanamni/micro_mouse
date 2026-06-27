#!/usr/bin/env python3
"""
ω 环带宽分析与自动调参工具
改下面 CONFIG 区的变量切换模式，然后直接 Run 这个文件。
"""

from __future__ import annotations

import sys
from pathlib import Path
from dataclasses import dataclass

PROJECT_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(PROJECT_ROOT))

import numpy as np
import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
from scipy import signal as sig

from micromouse_sim.physics.engine import PhysicsEngine, PULSES_PER_M, TRACK_WIDTH
from micromouse_sim.environment.loader import build_model_xml, load_track
from micromouse_sim.actuation.motor_model import MotorModel
from micromouse_sim.config.vw_controller_config import push_vw_params as push_vw_controller_params
from micromouse_sim import localize_core, control_core

# ═══════════════════════════════════════════════════════════════
# CONFIG — 改这里切换模式、调增益
# ═══════════════════════════════════════════════════════════════

# 模式: "step" | "chirp" | "both" | "auto_tune"
MODE = "step"

# ω 环增益
Kp = 0.5
Ki = 0.0
Kd = 0.001

# 阶跃测试参数
STEP_AMPLITUDE = 20.0   # rad/s
STEP_TIME      = 0.5    # s

# 扫频参数
CHIRP_AMPLITUDE = 5.0   # rad/s
CHIRP_F0        = 0.5   # Hz
CHIRP_F1        = 50.0  # Hz
CHIRP_DURATION  = 3.0   # s

# Auto-tune 扫描范围
KP_RANGE = (0.01, 0.15, 8)   # (min, max, steps)
KD_RANGE = (0.0, 0.005, 6)

# 物理参数（不动）
JZ  = 6.5e-5    # kg·m²
BW  = 1.95e-4    # Nm/(rad/s)，实测最优
W_MAX = 110.05    # Nm

# ═══════════════════════════════════════════════════════════
# ENGINE
# ═══════════════════════════════════════════════════════════

TRACK_DATA_DIR = Path(r"C:\Users\chj15\Desktop\RobotRace\路径优化\robotrace-shortcut-path-main\data")
BASE_XML = PROJECT_ROOT / "mujoco_models" / "micromouse" / "base.xml"

# Keep this legacy helper aligned with the deployed v/omega defaults.
JZ = 7.9e-5
BW = 2.1e-3
W_MAX = 0.05

_g_engine = None
_g_motor_L = None
_g_motor_R = None
_g_gyro_bias = 0.0


def _init_engine():
    global _g_engine, _g_motor_L, _g_motor_R, _g_gyro_bias
    if _g_engine is not None:
        return
    track = load_track(str(TRACK_DATA_DIR / "robotena_points.txt"))
    _g_engine = PhysicsEngine(
        build_model_xml(str(BASE_XML), track=track, track_width=0.180),
        downforce=6.0, skirt_mu=0.05, skirt_R=0.03,
    )
    _g_motor_L = MotorModel()
    _g_motor_R = MotorModel()
    for _ in range(5000):
        _g_engine.step()
    _g_gyro_bias = float(np.mean([_g_engine.get_state().gyro[2] for _ in range(10000) if not _g_engine.step()]))
    # Reset engine to post-settle state
    _g_engine.reset = lambda: None  # don't re-settle


def _reset_engine():
    """Reset engine and cores for a fresh test."""
    _init_engine()
    # 重建引擎，保证每次测试从静止开始
    track = load_track(str(TRACK_DATA_DIR / "robotena_points.txt"))
    global _g_engine, _g_motor_L, _g_motor_R
    _g_engine = PhysicsEngine(
        build_model_xml(str(BASE_XML), track=track, track_width=0.180),
        downforce=6.0, skirt_mu=0.05, skirt_R=0.03,
    )
    for _ in range(5000):
        _g_engine.step()
    _g_motor_L = MotorModel()
    _g_motor_R = MotorModel()
    localize_core.reset(42)
    control_core.reset()
    control_core.vw_reset()
    localize_core.set_calibration(PULSES_PER_M, PULSES_PER_M, 0.01, 2.5e-4, TRACK_WIDTH, _g_gyro_bias)
    localize_core.set_slip_params(0.10, 0.5, 0.3)


# ═══════════════════════════════════════════════════════════
# CONFIG
# ═══════════════════════════════════════════════════════════

CTRL_DT = 1e-3   # 1 kHz
IMU_DT  = 2e-4   # 5 kHz


def _push_gains(kp, ki, kd):
    push_vw_controller_params(control_core, {
        "vw_Jz": JZ,
        "vw_Dw": BW,
        "vw_w_Kp": kp,
        "vw_w_Ki": ki,
        "vw_w_Kd": kd,
        "vw_w_max": W_MAX,
    })


@dataclass
class StepResult:
    """Metrics from a step response test."""
    rise_time_ms: float
    overshoot_pct: float
    settling_time_ms: float
    steady_state: float
    steady_state_error_pct: float
    t: np.ndarray = None
    w_ref: np.ndarray = None
    w_meas: np.ndarray = None
    tau: np.ndarray = None


# ═══════════════════════════════════════════════════════════
# CORE: Run one test
# ═══════════════════════════════════════════════════════════

def _run_test(omega_ref_fn, duration: float, settle_time: float = 0.3):
    """
    Run simulation with given ω_ref(t) function.
    Returns (t, ω_ref, ω_meas, τ_ω) arrays logged at 1 kHz.
    """
    _reset_engine()
    engine = _g_engine
    ml, mr = _g_motor_L, _g_motor_R

    steps = int(duration / 2e-5)
    t_log, wref_log, wmeas_log, tau_log = [], [], [], []
    gyro_acc = np.zeros(3)
    accel_acc = np.zeros(3)
    imu_count = 0
    u_L = u_R = 0.0

    for step in range(steps):
        st = engine.get_state()
        engine.set_control(
            ml.compute_torque(u_L, st.wheel_L_vel, 2e-5),
            mr.compute_torque(u_R, st.wheel_R_vel, 2e-5),
        )
        engine.step()
        st = engine.get_state()

        gyro_acc += st.gyro
        accel_acc += st.accel
        imu_count += 1
        sic = step % 50

        if sic % 10 == 0 and imu_count > 0:
            n = localize_core.imu_step(gyro_acc / imu_count, accel_acc / imu_count, IMU_DT)
            localize_core.push_imu(float(n["gyro"][2]), float(n["accel"][0]), float(n["accel"][1]), IMU_DT)
            t = st.time
            w_ref = omega_ref_fn(t) if t >= settle_time else 0.0
            control_core.vw_override_omega_ref(w_ref)
            control_core.vw_omega_step(float(n["gyro"][2]), IMU_DT)
            gyro_acc[:] = 0.0
            accel_acc[:] = 0.0
            imu_count = 0

        if sic == 0:
            localize_core.push_encoder(st.wheel_L_pos, st.wheel_R_pos, CTRL_DT)
            t = st.time
            w_ref = omega_ref_fn(t) if t >= settle_time else 0.0
            control_core.vw_override_omega_ref(w_ref)
            cmd = control_core.vw_control_tick(0.0, 0.0, 0.0, 0.0, CTRL_DT)
            u_L = float(cmd["u_L"])
            u_R = float(cmd["u_R"])

            dbg = control_core.vw_get_debug()
            t_log.append(t)
            wref_log.append(dbg["omega_ref"])
            wmeas_log.append(dbg["omega_meas"])
            tau_log.append(dbg["tau_omega"])

    return (
        np.array(t_log),
        np.array(wref_log),
        np.array(wmeas_log),
        np.array(tau_log),
    )


# ═══════════════════════════════════════════════════════════
# STEP RESPONSE ANALYSIS
# ═══════════════════════════════════════════════════════════

def analyze_step(amplitude: float = 10.0,
                 step_time: float = 0.5, duration: float = 2.0) -> StepResult:
    """Run step response and compute metrics."""

    def step_fn(t):
        return amplitude if t >= step_time else 0.0

    _push_gains(Kp, Ki, Kd)  # use global CONFIG
    t, wref, wmeas, tau = _run_test(step_fn, duration, settle_time=0.3)

    # Trim to post-step
    idx_step = np.searchsorted(t, step_time)
    t_post = t[idx_step:] - step_time
    w_post = wmeas[idx_step:]

    # Steady state (last 200ms)
    w_ss = np.mean(w_post[-200:])
    ss_err = abs(w_ss - amplitude) / abs(amplitude) * 100 if amplitude != 0 else 0

    # Rise time 10%->90%
    w_10 = 0.1 * w_ss
    w_90 = 0.9 * w_ss
    try:
        t_10 = t_post[np.where(w_post >= w_10)[0][0]]
        t_90 = t_post[np.where(w_post >= w_90)[0][0]]
        rise_time = (t_90 - t_10) * 1000  # ms
    except IndexError:
        rise_time = float("nan")

    # Overshoot
    w_max = np.max(w_post)
    overshoot = (w_max - w_ss) / abs(w_ss) * 100 if abs(w_ss) > 0.1 else 0

    # Settling time (±5%)
    tol = 0.05 * abs(w_ss)
    try:
        settling_idx = np.where(np.abs(w_post - w_ss) < tol)[0]
        settling_time = t_post[settling_idx[-1]] * 1000 if len(settling_idx) > 0 else float("nan")
    except Exception:
        settling_time = float("nan")

    return StepResult(
        rise_time_ms=rise_time,
        overshoot_pct=overshoot,
        settling_time_ms=settling_time,
        steady_state=w_ss,
        steady_state_error_pct=ss_err,
        t=t, w_ref=wref, w_meas=wmeas, tau=tau,
    )


# ═══════════════════════════════════════════════════════════
# CHIRP (FREQUENCY SWEEP) ANALYSIS
# ═══════════════════════════════════════════════════════════

@dataclass
class ChirpResult:
    freq: np.ndarray
    magnitude_db: np.ndarray
    phase_deg: np.ndarray
    coherence: np.ndarray
    f_coh: np.ndarray
    bw_3db_hz: float
    dc_gain: float
    t: np.ndarray
    w_ref: np.ndarray
    w_meas: np.ndarray


def analyze_chirp(amplitude: float = 5.0,
                  f0: float = 0.5, f1: float = 50.0,
                  chirp_duration: float = 3.0) -> ChirpResult:
    """Run chirp (logarithmic sine sweep) and compute transfer function."""

    settle = 0.3

    def chirp_fn(t):
        if t < settle:
            return 0.0
        t_rel = t - settle
        if t_rel > chirp_duration:
            return 0.0
        T = chirp_duration
        beta = np.log(f1 / f0) / T
        phase = 2 * np.pi * f0 / beta * (np.exp(beta * t_rel) - 1)
        return amplitude * np.sin(phase)

    _push_gains(Kp, Ki, Kd)
    t, wref, wmeas, tau = _run_test(chirp_fn, chirp_duration + settle + 0.5, settle_time=settle)

    # Transfer function estimation
    mask = (t >= settle) & (t <= settle + chirp_duration)
    t_c = t[mask] - settle
    u = wref[mask]
    y = wmeas[mask]

    fs = 1000.0
    nperseg = min(1024, max(256, len(t_c) // 8))
    f, Pxx = sig.csd(u, u, fs=fs, nperseg=nperseg, scaling="density")
    f, Pxy = sig.csd(u, y, fs=fs, nperseg=nperseg, scaling="density")

    mask_valid = (Pxx > 1e-12) & (f > 0)
    H = np.zeros_like(Pxy, dtype=complex)
    H[mask_valid] = Pxy[mask_valid] / Pxx[mask_valid]
    f_valid = f[mask_valid]
    H_valid = H[mask_valid]

    mag = np.abs(H_valid)
    mag_db = 20 * np.log10(mag + 1e-12)
    ph = np.angle(H_valid, deg=True)

    # Coherence
    f_coh, Cxy = sig.coherence(u, y, fs=fs, nperseg=nperseg)
    f_coh_valid = f_coh[f_coh > 0]
    Cxy_valid = Cxy[:len(f_coh_valid)]

    # -3dB bandwidth
    try:
        # Find frequencies between 1-50 Hz where coherence is good
        valid_range = (f_valid >= 1.0) & (f_valid <= 50.0) & (Cxy[:len(f_valid)] > 0.6)
        if np.any(valid_range):
            f_band = f_valid[valid_range]
            mag_band = mag[valid_range]
            dc = np.mean(mag_band[f_band < 3.0]) if np.any(f_band < 3.0) else mag_band[0]
            idx_3db = np.where(mag_band < dc / np.sqrt(2))[0]
            bw = f_band[idx_3db[0]] if len(idx_3db) > 0 else float("nan")
        else:
            dc = mag[0] if len(mag) > 0 else 0
            bw = float("nan")
    except Exception:
        dc = mag[0] if len(mag) > 0 else 0
        bw = float("nan")

    return ChirpResult(
        freq=f_valid, magnitude_db=mag_db, phase_deg=ph,
        coherence=Cxy_valid, f_coh=f_coh_valid,
        bw_3db_hz=bw, dc_gain=dc,
        t=t, w_ref=wref, w_meas=wmeas,
    )


# ═══════════════════════════════════════════════════════════
# AUTO-TUNE
# ═══════════════════════════════════════════════════════════

def auto_tune():
    kp_vals = np.linspace(*KP_RANGE)
    kd_vals = np.linspace(*KD_RANGE)

    print(f"\n{'='*60}")
    print(f"AUTO-TUNE: {len(kp_vals)}x{len(kd_vals)} = {len(kp_vals)*len(kd_vals)} combos")
    print(f"  Kp: {kp_vals[0]:.3f} -> {kp_vals[-1]:.3f}")
    print(f"  Kd: {kd_vals[0]:.4f} -> {kd_vals[-1]:.4f}")
    print(f"{'='*60}")

    best_cost = float("inf")
    best = (0.0, 0.0)
    best_result = None

    for kp in kp_vals:
        for kd in kd_vals:
            global Kp, Ki, Kd; Kp, Ki, Kd = kp, 0.0, kd  # 临时覆盖全局
            try:
                r = analyze_step(amplitude=10.0, step_time=0.5, duration=1.5)
            except Exception as e:
                print(f"  Kp={kp:.3f} Kd={kd:.4f} -> FAIL ({e})")
                continue

            cost = r.rise_time_ms + 8.0 * r.overshoot_pct + 10.0 * r.steady_state_error_pct
            if r.steady_state_error_pct > 30 or r.overshoot_pct > 95:
                cost = float("inf")
            status = "*" if cost < best_cost else " "
            if cost < best_cost:
                best_cost = cost; best = (kp, kd); best_result = r
            print(f"  {status}Kp={kp:.3f} Kd={kd:.4f} Tr={r.rise_time_ms:.0f}ms OS={r.overshoot_pct:.0f}% SSerr={r.steady_state_error_pct:.0f}% cost={cost:.0f}")

    Kp, Ki, Kd = best[0], 0.0, best[1]  # 恢复全局为最优
    print(f"\n  BEST: Kp={best[0]:.3f} Kd={best[1]:.4f} cost={best_cost:.0f}")
    return best_result


# ═══════════════════════════════════════════════════════════
# PLOTTING
# ═══════════════════════════════════════════════════════════

def plot_results(step: StepResult, chirp: ChirpResult | None,
                 title: str = "ω-Loop Bandwidth Analysis"):
    """Generate comprehensive plot."""
    if chirp is not None:
        fig, axes = plt.subplots(2, 3, figsize=(16, 10))
    else:
        fig, axes = plt.subplots(1, 3, figsize=(16, 5))
        axes = axes.reshape(1, -1)

    fig.suptitle(title, fontsize=13)

    # Step response
    ax0 = axes[0, 0]
    ax0.plot(step.t, step.w_ref, "k--", lw=1, alpha=0.5, label="ω_ref")
    ax0.plot(step.t, step.w_meas, "b-", lw=1.2, label="ω_meas")
    ax0.axvline(0.5, color="gray", ls=":", alpha=0.5)
    ax0.set_xlabel("Time (s)")
    ax0.set_ylabel("ω (rad/s)")
    ax0.set_title("Step Response (10 rad/s)")
    ax0.legend(fontsize=7)
    ax0.grid(True, alpha=0.3)
    ax0.text(0.02, 0.95,
             f"Rise: {step.rise_time_ms:.0f}ms\nOS: {step.overshoot_pct:.0f}%\n"
             f"Settle: {step.settling_time_ms:.0f}ms\nSS err: {step.steady_state_error_pct:.1f}%\n"
             f"BW≈{350/step.rise_time_ms:.0f}Hz" if step.rise_time_ms > 0 else "N/A",
             transform=ax0.transAxes, va="top", fontsize=8,
             bbox=dict(facecolor="w", alpha=0.7))

    # Rise detail
    ax1 = axes[0, 1]
    t0, t1 = 0.48, 0.65
    mask_z = (step.t >= t0) & (step.t <= t1)
    if np.any(mask_z):
        ax1.plot(step.t[mask_z], step.w_ref[mask_z], "k--", lw=1, alpha=0.5)
        ax1.plot(step.t[mask_z], step.w_meas[mask_z], "b-", lw=1.5)
        ax1.axhline(step.steady_state, color="g", ls=":", alpha=0.5, label=f"SS={step.steady_state:.1f}")
    ax1.set_xlabel("Time (s)")
    ax1.set_ylabel("ω (rad/s)")
    ax1.set_title("Rise Detail")
    ax1.legend(fontsize=7)
    ax1.grid(True, alpha=0.3)

    # τ_ω
    ax2 = axes[0, 2]
    mask_tau = (step.t >= 0.4) & (step.t <= 0.8)
    ax2.plot(step.t[mask_tau], step.tau[mask_tau] * 1000, "r-", lw=1)
    lim = 0.05 * 1000  # w_max in mNm
    ax2.axhline(lim, color="gray", ls=":", alpha=0.5)
    ax2.axhline(-lim, color="gray", ls=":", alpha=0.5)
    ax2.set_xlabel("Time (s)")
    ax2.set_ylabel("τ_ω (mNm)")
    ax2.set_title("Steering Torque")
    ax2.grid(True, alpha=0.3)

    if chirp is not None:
        # Bode magnitude
        ax3 = axes[1, 0]
        ax3.semilogx(chirp.freq, chirp.magnitude_db, "b-", lw=1, label="|H(f)|")
        has_legend = True
        if not np.isnan(chirp.bw_3db_hz):
            ax3.axvline(chirp.bw_3db_hz, color="r", ls="--", alpha=0.5,
                        label=f"-3dB @ {chirp.bw_3db_hz:.1f}Hz")
        ax3.axhline(-3, color="gray", ls=":", alpha=0.5)
        ax3.set_xlabel("Frequency (Hz)")
        ax3.set_ylabel("Magnitude (dB)")
        ax3.set_title("Bode: w_ref -> w_meas")
        ax3.legend(fontsize=7)
        ax3.grid(True, alpha=0.3, which="both")
        ax3.set_xlim([0.5, 60])

        # Phase
        ax4 = axes[1, 1]
        ax4.semilogx(chirp.freq, chirp.phase_deg, "b-", lw=1)
        ax4.axhline(-90, color="gray", ls=":", alpha=0.5)
        ax4.set_xlabel("Frequency (Hz)")
        ax4.set_ylabel("Phase (deg)")
        ax4.set_title("Phase Response")
        ax4.grid(True, alpha=0.3, which="both")
        ax4.set_xlim([0.5, 60])

        # Coherence
        ax5 = axes[1, 2]
        ax5.semilogx(chirp.f_coh, chirp.coherence, "g-", lw=1)
        ax5.axhline(0.8, color="gray", ls=":", alpha=0.5)
        ax5.set_xlabel("Frequency (Hz)")
        ax5.set_ylabel("γ²")
        ax5.set_title("Coherence")
        ax5.set_ylim([0, 1.05])
        ax5.grid(True, alpha=0.3)
        ax5.set_xlim([0.5, 60])

    plt.tight_layout()
    out_path = str(PROJECT_ROOT / "scripts" / "omega_bandwidth.png")
    plt.savefig(out_path, dpi=150)
    print(f"Plot saved: {out_path}")


# ═══════════════════════════════════════════════════════════
# MAIN
# ═══════════════════════════════════════════════════════════

if __name__ == "__main__":
    _init_engine()

    print(f"\n{'='*60}")
    print(f"MODE={MODE}  Kp={Kp:.3f} Ki={Ki:.3f} Kd={Kd:.4f}  Jz={JZ:.1e} Bw={BW:.1e}")
    print(f"{'='*60}")

    if MODE == "auto_tune":
        step_result = auto_tune()
        chirp_result = analyze_chirp(CHIRP_AMPLITUDE, CHIRP_F0, CHIRP_F1, CHIRP_DURATION)
    elif MODE == "chirp":
        step_result = None
        chirp_result = analyze_chirp(CHIRP_AMPLITUDE, CHIRP_F0, CHIRP_F1, CHIRP_DURATION)
        print(f"  DC gain: {chirp_result.dc_gain:.3f}  -3dB BW: {chirp_result.bw_3db_hz:.1f} Hz")
    elif MODE == "step":
        step_result = analyze_step(STEP_AMPLITUDE, STEP_TIME)
        chirp_result = None
        print(f"  Rise: {step_result.rise_time_ms:.1f}ms  OS: {step_result.overshoot_pct:.1f}%  SSerr: {step_result.steady_state_error_pct:.1f}%")
    else:  # "both"
        print("\n--- Step Response ---")
        step_result = analyze_step(STEP_AMPLITUDE, STEP_TIME)
        print(f"  Rise: {step_result.rise_time_ms:.1f}ms  OS: {step_result.overshoot_pct:.1f}%  SSerr: {step_result.steady_state_error_pct:.1f}%")
        print("\n--- Chirp Sweep ---")
        chirp_result = analyze_chirp(CHIRP_AMPLITUDE, CHIRP_F0, CHIRP_F1, CHIRP_DURATION)
        print(f"  DC gain: {chirp_result.dc_gain:.3f}  -3dB BW: {chirp_result.bw_3db_hz:.1f} Hz")

    if step_result is not None:
        title = f"w-Loop: Kp={Kp:.3f} Ki={Ki:.3f} Kd={Kd:.4f}"
        plot_results(step_result, chirp_result, title)
