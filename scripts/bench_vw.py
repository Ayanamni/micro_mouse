#!/usr/bin/env python3
"""
bench_vw.py — Dedicated performance bench for the v/omega decoupled controller.

WHY THIS EXISTS (vs eval_vw.py):
    eval_vw.py measures END-TO-END line-following on a track (lateral error, line
    loss, laps). It cannot isolate the controller's own dynamics, because the
    reference is whatever the line sensor produces on that particular track.

    bench_vw.py BYPASSES the line sensor and injects reference signals DIRECTLY
    into the controller:
        - omega_ref  via control_core.vw_override_omega_ref(...)
        - v_ref      via the v_ref argument of vw_control_tick(...)
    The car runs on the (non-collidable) track floor and we measure how the
    closed loop responds. This isolates *controller quality* from *track shape*.

WHAT IT MEASURES (the things that actually define a good v/omega controller):
    1. omega-step  : rise time / overshoot / settling / steady-state error of the
                     angular-velocity loop  (the inner 5 kHz loop — most critical).
    2. v-step      : same metrics for the linear-velocity loop (1 kHz).
    3. decouple    : CROSS-TALK. Hold v const, step omega -> how much does v dip?
                     Hold omega=0, step v -> how much spurious yaw appears?
                     This is THE figure of merit for a *decoupled* architecture.
    4. bw (sweep)  : stepped-sine frequency response -> closed-loop -3 dB bandwidth
                     and phase lag, for both omega and v loops.
    5. disturb     : load-disturbance rejection. Step the skirt drag mid-run ->
                     peak deviation + recovery time.
    6. all         : run the full battery, print a consolidated report card.

The simulation harness (BenchSim) mirrors the 50 kHz / 5 kHz / 1 kHz multi-rate
control loop of workbench.py exactly, and pulls the SAME deployed parameters
(see BenchParams defaults == workbench.SimParams defaults). Keep them in sync.

Usage:
    python scripts/bench_vw.py omega-step --mag 5 10 15
    python scripts/bench_vw.py v-step --mag 1.0 2.0 3.0
    python scripts/bench_vw.py decouple --v 2.0 --omega 8.0
    python scripts/bench_vw.py bw --loop omega
    python scripts/bench_vw.py bw --loop v
    python scripts/bench_vw.py disturb --loop omega
    python scripts/bench_vw.py all          # full report card
    python scripts/bench_vw.py all --json out.json
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
from dataclasses import dataclass, field, asdict
from pathlib import Path
from typing import Callable, Optional

import numpy as np

os.environ["MPLBACKEND"] = "Agg"  # before any mpl import pulled in transitively

PROJECT_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(PROJECT_ROOT))

from micromouse_sim.physics.engine import (
    PhysicsEngine, WHEEL_RADIUS, TRACK_WIDTH, PULSES_PER_M,
)
from micromouse_sim.environment.loader import build_model_xml, load_track
from micromouse_sim.actuation.delay_buffer import ActuationDelayBuffer
from micromouse_sim.actuation.motor_model import MotorModel
from micromouse_sim.config.vw_controller_config import (
    VW_DEFAULTS,
    VW_PARAM_NAMES,
    push_vw_params as push_vw_controller_params,
)
from micromouse_sim import localize_core, control_core

# ── Paths ────────────────────────────────────────────────────────────────────
BASE_XML = PROJECT_ROOT / "mujoco_models" / "micromouse" / "base.xml"
_TRACK_DIR = Path(r"C:\Users\chj15\Desktop\RobotRace\路径优化\robotrace-shortcut-path-main\data")
if not _TRACK_DIR.exists():
    _TRACK_DIR = PROJECT_ROOT.parent / "路径优化" / "robotrace-shortcut-path-main" / "data"
TRACK_DATA_DIR = _TRACK_DIR

# ── Multi-rate timing (mirror workbench.py / eval_vw.py) ─────────────────────
PHYSICS_DT = 2e-5          # 50 kHz
IMU_RATE   = 5000          # Hz
IMU_STEPS  = int(1.0 / (IMU_RATE * PHYSICS_DT))    # 10
IMU_DT     = 1.0 / IMU_RATE                         # 200 us
CTRL_RATE  = 1000          # Hz
CTRL_STEPS = int(1.0 / (CTRL_RATE * PHYSICS_DT))   # 50
CTRL_DT    = 1.0 / CTRL_RATE                        # 1 ms


# ═════════════════════════════════════════════════════════════════════════════
# Deployed controller parameters (MUST match workbench.SimParams defaults)
# ═════════════════════════════════════════════════════════════════════════════

@dataclass
class BenchParams:
    track: str = "2019kansai"
    track_width: float = 0.20

    # v-omega controller (== workbench.SimParams)
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
    ideal_motor: bool = False
    act_delay_us: float = 0.0


# ═════════════════════════════════════════════════════════════════════════════
# Simulation harness — built ONCE, reset between runs (avoids recompiling XML)
# ═════════════════════════════════════════════════════════════════════════════

@dataclass
class Trace:
    """Per-control-tick (1 kHz) time series of one run."""
    t:         np.ndarray
    v_ref:     np.ndarray
    v_est:     np.ndarray   # Kalman estimate (what the controller sees)
    v_true:    np.ndarray   # ground-truth body forward velocity
    w_ref:     np.ndarray   # omega reference (what we injected)
    w_meas:    np.ndarray   # omega measured (gyro, what the controller sees)
    w_true:    np.ndarray   # ground-truth yaw rate
    tau_v:     np.ndarray
    tau_w:     np.ndarray
    w_tau_ff:  np.ndarray
    w_tau_fb:  np.ndarray
    u_L:       np.ndarray
    u_R:       np.ndarray


class BenchSim:
    """Reusable v/omega controller test rig with direct reference injection."""

    def __init__(self, params: BenchParams, seed: int = 42):
        self.p = params
        self.seed = seed

        track = load_track(str(TRACK_DATA_DIR / f"{params.track}_points.txt"))
        self.track = track
        model_xml = build_model_xml(str(BASE_XML), track=track,
                                    track_width=params.track_width)
        self.engine = PhysicsEngine(
            model_xml=model_xml,
            downforce=params.downforce,
            skirt_mu=params.skirt_mu,
            skirt_R=params.skirt_R,
        )
        self.motor_L = MotorModel()
        self.motor_R = MotorModel()

        # base drag values (for disturbance injection)
        self._F_skirt_base = self.engine._F_skirt
        self._tau_skirt_base = self.engine._tau_skirt

        # one-time gyro bias calibration at rest
        for _ in range(2000):
            self.engine.step()
        _bias = []
        for _ in range(2000):
            self.engine.step()
            _bias.append(self.engine.get_state().gyro[2])
        self._gyro_bias = float(np.mean(_bias))
        self._init_cores()
        self.reset()

    # ── cores ────────────────────────────────────────────────────────────────
    def _init_cores(self):
        p = self.p
        localize_core.reset(self.seed)
        control_core.reset()
        control_core.vw_reset()
        localize_core.set_calibration(
            pulses_per_m_L=PULSES_PER_M, pulses_per_m_R=PULSES_PER_M,
            accel_noise_std=p.sigma_accel, enc_dist_noise=p.sigma_enc_dist,
            track_width=TRACK_WIDTH, gyro_bias_init=self._gyro_bias,
        )
        localize_core.set_slip_params(p.thresh_lon, p.thresh_lat, p.k_slip)
        push_vw_controller_params(control_core, p)

    def reset(self):
        """Return to spawn pose, re-init cores, settle briefly."""
        self.engine.reset()
        # restore any disturbance to drag
        self.engine._F_skirt = self._F_skirt_base
        self.engine._tau_skirt = self._tau_skirt_base
        for _ in range(1500):
            self.engine.step()
        self._init_cores()

    # ── main run with arbitrary reference + disturbance callbacks ─────────────
    def run(self,
            duration: float,
            v_ref_fn: Callable[[float], float],
            w_ref_fn: Optional[Callable[[float], float]],
            dist_fn: Optional[Callable[[float], None]] = None) -> Trace:
        """
        Run the multi-rate loop for `duration` seconds.

        v_ref_fn(t)  -> linear velocity reference (m/s), passed to vw_control_tick
        w_ref_fn(t)  -> omega reference (rad/s) injected via override; None = let
                        the tracking controller run with lateral_error forced to 0
                        (i.e. omega_ref ~ 0; the car holds straight)
        dist_fn(t)   -> optional hook called each control tick to mutate the plant
                        (e.g. self.engine._tau_skirt = ...) for disturbance tests.
        """
        eng = self.engine
        u_L = u_R = 0.0
        act_delay = ActuationDelayBuffer(self.p.act_delay_us, PHYSICS_DT)
        gyro_acc = np.zeros(3); accel_acc = np.zeros(3); imu_n = 0
        step_count = 0

        rec = {k: [] for k in
               ("t", "v_ref", "v_est", "v_true", "w_ref", "w_meas", "w_true",
                "tau_v", "tau_w", "w_tau_ff", "w_tau_fb", "u_L", "u_R")}

        n_steps = int(duration / PHYSICS_DT)
        for _ in range(n_steps):
            st = eng.get_state()
            u_delayed_L, u_delayed_R = act_delay.apply(u_L, u_R)
            if self.p.ideal_motor:
                eng.set_control(u_delayed_L * 0.05, u_delayed_R * 0.05)
            else:
                eng.set_control(
                    self.motor_L.compute_torque(u_delayed_L, st.wheel_L_vel, PHYSICS_DT),
                    self.motor_R.compute_torque(u_delayed_R, st.wheel_R_vel, PHYSICS_DT),
                )
            eng.step()
            st = eng.get_state()

            gyro_acc += st.gyro; accel_acc += st.accel; imu_n += 1
            step_in_ctrl = step_count % CTRL_STEPS

            # 5 kHz omega loop
            if step_in_ctrl % IMU_STEPS == 0 and imu_n > 0:
                gyro_avg = gyro_acc / imu_n; accel_avg = accel_acc / imu_n
                noisy = localize_core.imu_step(gyro_avg, accel_avg, IMU_DT)
                localize_core.push_imu(float(noisy["gyro"][2]),
                                       float(noisy["accel"][0]),
                                       float(noisy["accel"][1]), IMU_DT)
                control_core.vw_omega_step(float(noisy["gyro"][2]), IMU_DT)
                gyro_acc[:] = 0.0; accel_acc[:] = 0.0; imu_n = 0

            # 1 kHz control tick
            if step_in_ctrl == 0:
                t = st.time
                if dist_fn is not None:
                    dist_fn(t)
                localize_core.push_encoder(st.wheel_L_pos, st.wheel_R_pos, CTRL_DT)
                pose = localize_core.read_pose()

                v_ref = float(v_ref_fn(t))
                if w_ref_fn is not None:
                    control_core.vw_override_omega_ref(float(w_ref_fn(t)))
                else:
                    control_core.vw_override_omega_ref(0.0)

                control_core.vw_set_wheel_omega(st.wheel_L_vel, st.wheel_R_vel)
                cmd = control_core.vw_control_tick(
                    0.0, 0.0, float(pose["v_fwd"]), v_ref, CTRL_DT)
                u_L = float(cmd["u_L"]); u_R = float(cmd["u_R"])
                dbg = control_core.vw_get_debug()

                rec["t"].append(t)
                rec["v_ref"].append(v_ref)
                rec["v_est"].append(float(pose["v_fwd"]))
                rec["v_true"].append(st.forward_velocity)
                rec["w_ref"].append(float(dbg["omega_ref"]))
                rec["w_meas"].append(float(dbg["omega_meas"]))
                rec["w_true"].append(float(st.gyro[2]))
                rec["tau_v"].append(float(dbg["tau_v"]))
                rec["tau_w"].append(float(dbg["tau_omega"]))
                rec["w_tau_ff"].append(float(dbg["omega_tau_ff"]))
                rec["w_tau_fb"].append(float(dbg["omega_tau_fb"]))
                rec["u_L"].append(u_L); rec["u_R"].append(u_R)

            step_count += 1

        control_core.vw_clear_omega_override()
        return Trace(**{k: np.asarray(v) for k, v in rec.items()})


# ═════════════════════════════════════════════════════════════════════════════
# Metric extraction
# ═════════════════════════════════════════════════════════════════════════════

@dataclass
class StepMetrics:
    target: float
    y0: float
    ss: float
    ss_err_pct: float       # (ss - target)/target * 100
    overshoot_pct: float    # (peak - target)/span * 100  (>=0)
    rise_ms: float          # 10% -> 90% of span
    settle_ms: float        # time to enter +/-2% band of span around target and stay
    peak: float


def step_metrics(t: np.ndarray, y: np.ndarray, target: float,
                 t_step: float) -> Optional[StepMetrics]:
    pre = y[t < t_step]
    y0 = float(pre[-1]) if pre.size else 0.0
    span = target - y0
    if abs(span) < 1e-9:
        return None
    m = t >= t_step
    tt = t[m] - t_step
    yy = y[m]
    if yy.size < 3:
        return None
    n_ss = max(1, yy.size // 5)
    ss = float(np.mean(yy[-n_ss:]))
    ss_err_pct = (ss - target) / target * 100.0 if abs(target) > 1e-9 else 0.0

    sgn = 1.0 if span > 0 else -1.0
    peak = float(np.max(yy)) if sgn > 0 else float(np.min(yy))
    overshoot_pct = max(0.0, (peak - target) * sgn / abs(span) * 100.0)

    y10 = y0 + 0.1 * span
    y90 = y0 + 0.9 * span
    def _cross(level):
        idx = np.argmax((yy - y0) * sgn >= (level - y0) * sgn)
        return tt[idx] if idx > 0 or ((yy[0] - y0) * sgn >= (level - y0) * sgn) else tt[-1]
    rise_ms = max(0.0, (_cross(y90) - _cross(y10))) * 1000.0

    band = 0.02 * abs(span)
    outside = np.where(np.abs(yy - target) > band)[0]
    settle_ms = (tt[outside[-1]] * 1000.0) if outside.size else 0.0

    return StepMetrics(target, y0, ss, ss_err_pct, overshoot_pct,
                       rise_ms, settle_ms, peak)


def sine_fit(t: np.ndarray, y: np.ndarray, f: float):
    """Single-bin DFT at frequency f. Returns (amplitude, phase_rad)."""
    w = 2.0 * math.pi * f
    yc = y - np.mean(y)
    c = np.cos(w * t); s = np.sin(w * t)
    a = 2.0 / len(yc) * np.sum(yc * c)
    b = 2.0 / len(yc) * np.sum(yc * s)
    amp = math.hypot(a, b)
    phase = math.atan2(b, a)   # y ~ amp*cos(wt - phase) ... consistent relative use
    return amp, phase


# ═════════════════════════════════════════════════════════════════════════════
# Test modes
# ═════════════════════════════════════════════════════════════════════════════

def _fmt_step(name: str, mag: float, sm: Optional[StepMetrics]) -> str:
    if sm is None:
        return f"  {name} {mag:>7.2f}: <no step detected>"
    return (f"  {name} {mag:>7.2f} -> ss={sm.ss:8.3f} "
            f"err={sm.ss_err_pct:+6.1f}%  over={sm.overshoot_pct:5.1f}%  "
            f"rise={sm.rise_ms:6.1f}ms  settle={sm.settle_ms:6.1f}ms")


def test_omega_step(sim: BenchSim, mags, v_hold=0.0, settle_t=0.4,
                    hold_t=0.8, verbose=True):
    """Inject omega_ref step(s); measure omega-loop step response."""
    out = []
    for mag in mags:
        sim.reset()
        dur = settle_t + hold_t
        tr = sim.run(dur,
                     v_ref_fn=lambda t: v_hold,
                     w_ref_fn=lambda t, m=mag, s=settle_t: (m if t >= s else 0.0))
        sm = step_metrics(tr.t, tr.w_meas, mag, settle_t)
        out.append({"mag": mag, "metrics": asdict(sm) if sm else None})
        if verbose:
            print(_fmt_step("omega", mag, sm))
    return out


def test_v_step(sim: BenchSim, mags, settle_t=0.4, hold_t=1.6, verbose=True):
    """Inject v_ref step(s); measure velocity-loop step response."""
    out = []
    for mag in mags:
        sim.reset()
        dur = settle_t + hold_t
        tr = sim.run(dur,
                     v_ref_fn=lambda t, m=mag, s=settle_t: (m if t >= s else 0.0),
                     w_ref_fn=lambda t: 0.0)
        # measure on ground truth (Kalman lag would distort loop metrics)
        sm = step_metrics(tr.t, tr.v_true, mag, settle_t)
        out.append({"mag": mag, "metrics": asdict(sm) if sm else None})
        if verbose:
            print(_fmt_step("v    ", mag, sm))
    return out


def test_decouple(sim: BenchSim, v_hold=2.0, w_step=8.0,
                  settle_t=0.6, hold_t=1.2, verbose=True):
    """
    Cross-talk quantification — the figure of merit for *decoupling*.

    A) v held constant, omega stepped -> peak |delta v| / v   (steering disturbs speed?)
    B) omega held 0, v stepped        -> peak |omega|         (accel disturbs yaw?)
    """
    # ── A: omega step disturbs v ──
    sim.reset()
    trA = sim.run(settle_t + hold_t,
                  v_ref_fn=lambda t: v_hold,
                  w_ref_fn=lambda t, s=settle_t: (w_step if t >= s else 0.0))
    mA = trA.t >= settle_t
    # let v settle to v_hold first; baseline = mean over [settle_t-0.2, settle_t]
    base_mask = (trA.t >= settle_t - 0.2) & (trA.t < settle_t)
    v_base = float(np.mean(trA.v_true[base_mask])) if base_mask.any() else v_hold
    v_post = trA.v_true[mA]
    v_dip = float(np.max(np.abs(v_post - v_base)))
    v_dip_pct = v_dip / max(abs(v_base), 1e-6) * 100.0
    # residual steady-state v error after omega settles
    n_ss = max(1, v_post.size // 5)
    v_ss_after = float(np.mean(v_post[-n_ss:]))
    v_ss_drop_pct = (v_ss_after - v_base) / max(abs(v_base), 1e-6) * 100.0

    # ── B: v step disturbs omega ──
    sim.reset()
    trB = sim.run(settle_t + hold_t,
                  v_ref_fn=lambda t, s=settle_t: (v_hold if t >= s else 0.0),
                  w_ref_fn=lambda t: 0.0)
    mB = trB.t >= settle_t
    w_spurious = float(np.max(np.abs(trB.w_true[mB])))

    res = {
        "A_omega_disturbs_v": {
            "v_hold": v_hold, "w_step": w_step,
            "v_base": v_base, "peak_dip": v_dip, "peak_dip_pct": v_dip_pct,
            "ss_drop_pct": v_ss_drop_pct,
        },
        "B_v_disturbs_omega": {
            "v_step": v_hold,
            "peak_spurious_w": w_spurious,
        },
    }
    if verbose:
        print(f"  [A] omega {w_step} rad/s step @ v={v_hold} m/s:")
        print(f"        peak |dv| = {v_dip*1000:6.1f} mm/s ({v_dip_pct:.1f}% of v),"
              f"  steady v drop = {v_ss_drop_pct:+.1f}%")
        print(f"  [B] v {v_hold} m/s step @ omega=0:")
        print(f"        peak spurious |omega| = {w_spurious:.3f} rad/s")
    return res


def test_bandwidth(sim: BenchSim, loop="omega",
                   freqs=(1, 2, 4, 6, 10, 16, 24, 40),
                   amp=None, bias=None, v_hold=1.5, verbose=True):
    """
    Stepped-sine closed-loop frequency response.
    loop='omega': omega_ref = amp*sin, v_ref = v_hold
    loop='v'    : v_ref = bias + amp*sin, omega_ref = 0
    Reports gain (dB) & phase (deg) per freq, -3 dB bandwidth, phase at BW.
    """
    if loop == "omega":
        amp = amp if amp is not None else 4.0      # rad/s
        ref_field, meas_field = "w_ref", "w_meas"
    else:
        amp = amp if amp is not None else 0.5      # m/s
        bias = bias if bias is not None else 1.5
        ref_field, meas_field = "v_ref", "v_true"

    rows = []
    for f in freqs:
        sim.reset()
        n_per = 8
        dur_settle = 0.4
        dur_sine = max(0.5, min(2.5, n_per / f))
        dur = dur_settle + dur_sine
        w = 2 * math.pi * f
        if loop == "omega":
            tr = sim.run(dur,
                         v_ref_fn=lambda t: v_hold,
                         w_ref_fn=lambda t, s=dur_settle, ww=w, a=amp:
                             (a * math.sin(ww * (t - s)) if t >= s else 0.0))
        else:
            tr = sim.run(dur,
                         v_ref_fn=lambda t, s=dur_settle, ww=w, a=amp, b=bias:
                             (b + a * math.sin(ww * (t - s)) if t >= s else b),
                         w_ref_fn=lambda t: 0.0)
        # steady portion: skip first period after sine start
        mask = tr.t >= (dur_settle + 1.0 / f)
        tt = tr.t[mask]
        ref = getattr(tr, ref_field)[mask]
        meas = getattr(tr, meas_field)[mask]
        if tt.size < 8:
            continue
        a_ref, p_ref = sine_fit(tt, ref, f)
        a_meas, p_meas = sine_fit(tt, meas, f)
        gain = a_meas / a_ref if a_ref > 1e-9 else 0.0
        gain_db = 20 * math.log10(gain) if gain > 1e-9 else -120.0
        phase_deg = math.degrees(((p_meas - p_ref) + math.pi) % (2 * math.pi) - math.pi)
        rows.append({"f": f, "gain": gain, "gain_db": gain_db,
                     "phase_deg": phase_deg, "a_ref": a_ref, "a_meas": a_meas})
        if verbose:
            print(f"  f={f:5.1f} Hz  gain={gain:5.2f} ({gain_db:+6.1f} dB)  "
                  f"phase={phase_deg:+7.1f} deg")

    # -3 dB bandwidth (first downward crossing)
    bw = None
    for i in range(1, len(rows)):
        if rows[i - 1]["gain_db"] >= -3.0 >= rows[i]["gain_db"]:
            f0, g0 = rows[i - 1]["f"], rows[i - 1]["gain_db"]
            f1, g1 = rows[i]["f"], rows[i]["gain_db"]
            bw = f0 + (-3.0 - g0) * (f1 - f0) / (g1 - g0)
            break
    if bw is None and rows and rows[-1]["gain_db"] > -3.0:
        bw = rows[-1]["f"]  # bandwidth beyond tested range
    if verbose:
        if bw is not None:
            print(f"  -> closed-loop -3 dB bandwidth ~ {bw:.1f} Hz")
        else:
            print(f"  -> bandwidth below lowest tested freq")
    return {"loop": loop, "amp": amp, "rows": rows, "bw_hz": bw}


def test_disturbance(sim: BenchSim, loop="omega", settle_t=0.6, dist_t=1.0,
                     end_t=2.0, verbose=True):
    """
    Load-disturbance rejection. Step the skirt drag up at dist_t, measure peak
    deviation from reference and recovery time back into a 5% band.
    loop='omega': hold omega_ref=W, boost yaw drag (tau_skirt x4)
    loop='v'    : hold v_ref=V,     boost translational drag (F_skirt x4)
    """
    if loop == "omega":
        W = 6.0; V = 0.0
        def dist(t):
            sim.engine._tau_skirt = (sim._tau_skirt_base * 4.0 if t >= dist_t
                                     else sim._tau_skirt_base)
        ref_val = W
        sim.reset()
        tr = sim.run(end_t,
                     v_ref_fn=lambda t: V,
                     w_ref_fn=lambda t, s=settle_t: (W if t >= s else 0.0),
                     dist_fn=dist)
        y = tr.w_meas
    else:
        V = 2.0
        def dist(t):
            sim.engine._F_skirt = (sim._F_skirt_base * 4.0 if t >= dist_t
                                   else sim._F_skirt_base)
        ref_val = V
        sim.reset()
        tr = sim.run(end_t,
                     v_ref_fn=lambda t, s=settle_t: (V if t >= s else 0.0),
                     w_ref_fn=lambda t: 0.0,
                     dist_fn=dist)
        y = tr.v_true

    m = tr.t >= dist_t
    tt = tr.t[m] - dist_t
    yy = y[m]
    base = ref_val
    dev = np.abs(yy - base)
    peak_dev = float(np.max(dev))
    band = 0.05 * abs(base) if abs(base) > 1e-9 else 0.02
    outside = np.where(dev > band)[0]
    recover_ms = (tt[outside[-1]] * 1000.0) if outside.size else 0.0
    peak_dev_pct = peak_dev / max(abs(base), 1e-6) * 100.0
    if verbose:
        print(f"  [{loop}] drag x4 step @ ref={ref_val}: "
              f"peak dev={peak_dev:.4f} ({peak_dev_pct:.1f}%), "
              f"recover={recover_ms:.0f}ms")
    return {"loop": loop, "ref": ref_val, "peak_dev": peak_dev,
            "peak_dev_pct": peak_dev_pct, "recover_ms": recover_ms,
            "dist_clean": bool(sim.engine._tau_skirt == sim._tau_skirt_base
                               and sim.engine._F_skirt == sim._F_skirt_base)}


# ═════════════════════════════════════════════════════════════════════════════
# CLI
# ═════════════════════════════════════════════════════════════════════════════

def _hdr(title):
    print("=" * 64)
    print(f"  {title}")
    print("=" * 64)


def _apply_cli_overrides(params: BenchParams, args) -> BenchParams:
    for key in [*VW_PARAM_NAMES, "act_delay_us"]:
        if hasattr(args, key) and getattr(args, key) is not None:
            setattr(params, key, getattr(args, key))
    return params


def cmd_all(args):
    p = BenchParams(track=args.track, downforce=args.downforce,
                    skirt_mu=args.skirt_mu, ideal_motor=args.ideal)
    _apply_cli_overrides(p, args)
    print(f"Building bench rig (track={p.track}, downforce={p.downforce}, "
          f"skirt_mu={p.skirt_mu}) ...")
    sim = BenchSim(p, seed=args.seed)
    report = {"params": asdict(p), "seed": args.seed}

    _hdr("1) OMEGA-LOOP STEP RESPONSE (inner 5 kHz loop)")
    report["omega_step"] = test_omega_step(sim, [5.0, 10.0, 15.0])

    _hdr("2) VELOCITY-LOOP STEP RESPONSE (1 kHz loop)")
    report["v_step"] = test_v_step(sim, [1.0, 2.0, 3.0])

    _hdr("3) DECOUPLING CROSS-TALK")
    report["decouple"] = test_decouple(sim, v_hold=2.0, w_step=8.0)

    _hdr("4a) OMEGA-LOOP BANDWIDTH (stepped-sine)")
    report["bw_omega"] = test_bandwidth(sim, loop="omega")
    _hdr("4b) VELOCITY-LOOP BANDWIDTH (stepped-sine)")
    report["bw_v"] = test_bandwidth(sim, loop="v")

    _hdr("5) DISTURBANCE REJECTION (drag x4 step)")
    report["disturb_omega"] = test_disturbance(sim, loop="omega")
    report["disturb_v"] = test_disturbance(sim, loop="v")

    _hdr("REPORT CARD")
    _print_card(report)

    if args.json:
        Path(args.json).write_text(json.dumps(report, indent=2), encoding="utf-8")
        print(f"\nFull JSON written to {args.json}")
    return 0


def _print_card(report):
    def g(*keys, default=None):
        d = report
        for k in keys:
            if d is None:
                return default
            d = d.get(k) if isinstance(d, dict) else None
        return d if d is not None else default

    # omega step @ 10
    os10 = next((r["metrics"] for r in report.get("omega_step", [])
                 if abs(r["mag"] - 10.0) < 1e-6 and r["metrics"]), None)
    vs2 = next((r["metrics"] for r in report.get("v_step", [])
                if abs(r["mag"] - 2.0) < 1e-6 and r["metrics"]), None)
    dec = report.get("decouple", {})
    bw_w = g("bw_omega", "bw_hz")
    bw_v = g("bw_v", "bw_hz")
    dw = report.get("disturb_omega", {})
    dv = report.get("disturb_v", {})

    print(f"  omega loop  | step@10: rise {os10['rise_ms']:.0f}ms  "
          f"over {os10['overshoot_pct']:.0f}%  ss_err {os10['ss_err_pct']:+.1f}%  "
          if os10 else "  omega loop  | <n/a>  ")
    print(f"              | bandwidth ~ {bw_w if bw_w else float('nan'):.1f} Hz")
    print(f"  v loop      | step@2 : rise {vs2['rise_ms']:.0f}ms  "
          f"over {vs2['overshoot_pct']:.0f}%  ss_err {vs2['ss_err_pct']:+.1f}%  "
          if vs2 else "  v loop      | <n/a>  ")
    print(f"              | bandwidth ~ {bw_v if bw_v else float('nan'):.1f} Hz")
    if dec:
        print(f"  decoupling  | omega->v  steady drop {dec['A_omega_disturbs_v']['ss_drop_pct']:+.1f}%"
              f"  (peak {dec['A_omega_disturbs_v']['peak_dip_pct']:.1f}%)")
        print(f"              | v->omega  spurious {dec['B_v_disturbs_omega']['peak_spurious_w']:.3f} rad/s")
    if dw:
        print(f"  disturb rej | omega: peak {dw['peak_dev_pct']:.0f}%  recover {dw['recover_ms']:.0f}ms")
    if dv:
        print(f"              | v    : peak {dv['peak_dev_pct']:.0f}%  recover {dv['recover_ms']:.0f}ms")


def main():
    ap = argparse.ArgumentParser(description="v/omega controller performance bench")
    sub = ap.add_subparsers(dest="mode")

    def common(pp):
        pp.add_argument("--track", default="2019kansai")
        pp.add_argument("--seed", type=int, default=42)
        pp.add_argument("--downforce", type=float, default=6.0)
        pp.add_argument("--skirt-mu", type=float, default=0.05)
        pp.add_argument("--ideal", action="store_true")
        pp.add_argument("--vw-w-Kp", dest="vw_w_Kp", type=float)
        pp.add_argument("--vw-w-Ki", dest="vw_w_Ki", type=float)
        pp.add_argument("--vw-w-Kd", dest="vw_w_Kd", type=float)
        pp.add_argument("--vw-w-max", dest="vw_w_max", type=float)
        pp.add_argument("--beta-w", dest="vw_beta_w", type=float)
        pp.add_argument("--kaw-w", dest="vw_kaw_w", type=float)
        pp.add_argument("--w-Cfrict", dest="vw_w_Cfrict", type=float)
        pp.add_argument("--vw-v-Kp", dest="vw_v_Kp", type=float)
        pp.add_argument("--vw-v-Ki", dest="vw_v_Ki", type=float)
        pp.add_argument("--vw-v-max", dest="vw_v_max", type=float)
        pp.add_argument("--beta-v", dest="vw_beta_v", type=float)
        pp.add_argument("--kaw-v", dest="vw_kaw_v", type=float)
        pp.add_argument("--vw-Jz", dest="vw_Jz", type=float)
        pp.add_argument("--vw-Dw", dest="vw_Dw", type=float)
        pp.add_argument("--motor-I-peak", dest="motor_I_peak", type=float)
        pp.add_argument("--act-delay-us", dest="act_delay_us", type=float)

    p_os = sub.add_parser("omega-step"); common(p_os)
    p_os.add_argument("--mag", type=float, nargs="+", default=[5.0, 10.0, 15.0])
    p_os.add_argument("--v-hold", type=float, default=0.0)

    p_vs = sub.add_parser("v-step"); common(p_vs)
    p_vs.add_argument("--mag", type=float, nargs="+", default=[1.0, 2.0, 3.0])

    p_dc = sub.add_parser("decouple"); common(p_dc)
    p_dc.add_argument("--v", type=float, default=2.0)
    p_dc.add_argument("--omega", type=float, default=8.0)

    p_bw = sub.add_parser("bw"); common(p_bw)
    p_bw.add_argument("--loop", choices=["omega", "v"], default="omega")
    p_bw.add_argument("--freqs", type=float, nargs="+",
                      default=[1, 2, 4, 6, 10, 16, 24, 40])

    p_di = sub.add_parser("disturb"); common(p_di)
    p_di.add_argument("--loop", choices=["omega", "v"], default="omega")

    p_all = sub.add_parser("all"); common(p_all)
    p_all.add_argument("--json", type=str, default=None)

    args = ap.parse_args()
    if args.mode is None:
        ap.print_help(); return 0

    if args.mode == "all":
        return cmd_all(args)

    p = BenchParams(track=args.track, downforce=args.downforce,
                    skirt_mu=args.skirt_mu, ideal_motor=args.ideal)
    _apply_cli_overrides(p, args)
    print(f"Building bench rig (track={p.track}) ...")
    sim = BenchSim(p, seed=args.seed)

    if args.mode == "omega-step":
        _hdr("OMEGA-LOOP STEP RESPONSE")
        test_omega_step(sim, args.mag, v_hold=args.v_hold)
    elif args.mode == "v-step":
        _hdr("VELOCITY-LOOP STEP RESPONSE")
        test_v_step(sim, args.mag)
    elif args.mode == "decouple":
        _hdr("DECOUPLING CROSS-TALK")
        test_decouple(sim, v_hold=args.v, w_step=args.omega)
    elif args.mode == "bw":
        _hdr(f"{args.mode.upper()} {args.loop} BANDWIDTH")
        test_bandwidth(sim, loop=args.loop, freqs=tuple(args.freqs))
    elif args.mode == "disturb":
        _hdr(f"DISTURBANCE REJECTION ({args.loop})")
        test_disturbance(sim, loop=args.loop)
    return 0


if __name__ == "__main__":
    sys.exit(main())
