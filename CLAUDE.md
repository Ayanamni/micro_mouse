# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Mandatory Maintenance Protocol

Before any non-trivial code, simulation, controller, or parameter change, Claude must read and follow `项目维护协议.md`.

Key rule: write or update the module contract and verification plan first, then change code. If this file conflicts with `项目维护协议.md`, the shared protocol wins unless the user explicitly says otherwise.

## Project Overview

MuJoCo-based physics simulation for a high-speed line-following micromouse (超高速巡线电子鼠) — Japanese Robotrace negative-pressure differential-drive design. Goal: validate control algorithms in simulation before deploying to TC387 hardware.

**Current phase**: v/ω decoupled controller + realistic photodiode line-following. Planning/perception deferred.
**Controller architecture**: C++ `VWController` (OmegaLoop @5kHz + VelocityLoop @1kHz + TrackingController @1kHz) → torque mix τ_L=τ_v−τ_ω, τ_R=τ_v+τ_ω → motor model → MuJoCo.
**Active control path**: `workbench.py` uses `control_core.vw_omega_step` / `vw_control_tick` (NOT legacy `control_core.step`).

## Quick Start

```bash
# 启动仿真.bat — 三模式选择器（推荐）
启动仿真.bat
#  [1] Workbench — 仪表盘+调参（无3D窗口）
#  [2] Viewer — 3D自动巡线 15s
#  [3] Interactive — 键盘手动驾驶

# 命令行直接启动
python scripts/workbench.py                          # 仪表盘模式
python scripts/workbench.py --viewer-only --duration 15  # 3D巡线
python scripts/workbench.py --no-render --speed 2.0 --duration 10  # 无头测试

# Headless evaluation (v/ω path metrics — USE THIS FOR DEBUG)
python scripts/eval_vw.py single --track 2019kansai --speed 3.5 --duration 12
python scripts/eval_vw.py sweep --track 2019kansai --speed-min 1.0 --speed-max 5.0
python scripts/eval_vw.py omega-step --track 2019kansai --speed 1.0 --omega-step-value 5.0
```

Dependencies: `mujoco>=3.0`, `numpy`, `scipy`, `pyyaml`, `h5py`, `matplotlib`

## Architecture

```
micromouse_sim/
├── physics/engine.py               # MuJoCo wrapper: Coulomb skirt friction + downforce
├── environment/
│   ├── track.py                    # TrackCenterline (arc-length cubic spline)
│   └── loader.py                   # build_model_xml(): injects track capsules into base.xml
├── sensors/
│   └── line_sensor.py              # Finite-width photodiode array (16 LEDs, +/-70mm)
├── actuation/
│   └── motor_model.py              # Maxon ECX SPEED 13 L + 4:1 gearbox model
├── planner/                        # Stub — deferred (after controller)
├── control/                        # Stub — deferred
├── estimator/                      # Stub — deferred
└── config/defaults.yaml            # Legacy config (now superseded by SimParams)

scripts/
├── workbench.py                    # Main entry: SimRunner + Dashboard + line sensor + E-Stop
├── eval_vw.py                      # Headless eval: v/omega path metrics, sweep, omega-step
├── interactive.py                  # Keyboard manual driving
└── run_sim.py                      # Legacy batch runner

cpp/
├── control_core/
│   ├── vw_controller.hpp/cpp       # V/Omega decoupled controller (PRIMARY — active path)
│   ├── control_core.cpp            # pybind11 bindings (legacy + vw APIs)
│   └── lateral/speed controllers   # Legacy PID (A/B comparison only)
├── localize_core/
│   ├── pipeline_localizer.hpp/cpp  # 5kHz pipeline: IMU+encoder → pose
│   ├── velocity_kalman.hpp/cpp     # 2-state Kalman (v_fwd, accel_bias)
│   ├── slip_detector.hpp/cpp       # Longitudinal/lateral slip detection
│   └── imu_processor.hpp/cpp       # ICM-42688-P noise model
└── shared/                         # Shared types, math utils, Butterworth filters
```

### Core Data Flow (v/ω decoupled — active path)

1. MJCF XML assembled at runtime: `base.xml` + track capsule geoms injected at `<!-- WALLS -->`
2. `PhysicsEngine` loads XML, wraps `mjModel`/`mjData`
3. Each `step()` applies external forces (downforce + Coulomb skirt friction), then `mj_step()`
4. Sensors: IMU (gyro+accel), encoders (jointpos), framepos/quat/linvel/angvel
5. **5kHz**: IMU accumulate → average → `localize_core.imu_step` + `push_imu` → `control_core.vw_omega_step(gyro_z, dt)` → OmegaLoop updates τ_ω
6. **1kHz**: `push_encoder` → `read_pose` → line_sensor.read() → `control_core.vw_control_tick(lat_err, curvature, v_fwd, v_ref, dt)` → {u_L, u_R, throttle, steer, tau_v, tau_omega}
7. Control: `set_control(tau_L, tau_R)` through motor model → `data.ctrl`

Legacy `control_core.step()` path is A/B comparison only — NOT the active control path.

## Vehicle Model (`base.xml`)

| 参数 | 值 | 位置 |
|------|-----|------|
| 车身质量 | 0.090 kg | `chassis/inertial mass` |
| 轮子质量(ea) | 0.0035 kg | `wheel_L/inertial mass` |
| 轮径 | 21mm (r=0.0105) | `geom size="0.0105 X"` |
| 轮宽(半) | 8mm | `geom size="R 0.008"` |
| 轮距(半) | ±45mm | `wheel_L pos="0 0.045 …"` |
| 轮胎摩擦(切向) | 1.3 | `geom friction="1.3 …"` (MuJoCo elliptic cone) |
| 滑板尺寸 | 4×4×1.5mm (半) | `skid_rear/front size` |
| 滑板摩擦 | 0 | `skid friction="0 0 0"` (zero — skirt friction is Coulomb in engine.py) |
| 光电管数量 | 16 | LED geoms |
| 光电管跨度 | ±70mm | LED Y positions |
| 光电管前瞻 | 40mm | `pos="0.040 …"` |

### External Forces (applied in `engine.step()` — Coulomb skirt model)

| Force | Mechanism | Key Params |
|-------|-----------|------------|
| Downforce | `xfrc_applied[chassis, 2]` | `downforce=5.0` N |
| Skirt friction (translation) | `xfrc_applied[chassis, 0:2]` | `F=μ*F_N`, opposes velocity |
| Skirt friction (yaw) | `qfrc_applied[5]` | `τ=μ*F_N*R`, opposes yaw |

Params set in `workbench.py` → `PhysicsEngine(downforce, skirt_mu, skirt_R)`.

## Tunable Parameters Quick Reference

### v/ω Controller (PRIMARY — SimParams in workbench.py)

| 参数 | 行号 | 说明 |
|------|------|------|
| 目标线速度 | `workbench.py` L80 | `target_speed` (m/s) |
| ω环 Kp/Ki/Kd | `workbench.py` L87-89 | 角速度PID增益 |
| ω环 Jz/Bw | `workbench.py` L85-86 | 转动惯量/偏航阻尼 |
| ω环力矩上限 | `workbench.py` L90 | `vw_w_max` (Nm) |
| v环 Kp/Ki | `workbench.py` L95-96 | 线速度PI增益 |
| v环前馈 K_acc/K_vel/C_frict | `workbench.py` L92-94 | 加速/速度/摩擦补偿 |
| v环力矩上限 | `workbench.py` L97 | `vw_v_max` (Nm) |
| 巡线 Kp/Ki/Kd (纯光电管) | `workbench.py` L99-101 | lateral_error→ω_ref |

### Legacy PID (A/B comparison only)

| 参数 | 行号 | 说明 |
|------|------|------|
| 侧向PID | `workbench.py` L104-108 | Kp/Kd/Ki/Kff/ema_alpha |
| 速度PI | `workbench.py` L111-112 | Kp/Ki |

### Estimation & Physics

| 参数 | 行号 | 说明 |
|------|------|------|
| Kalman噪声 | `workbench.py` L115-116 | sigma_accel, sigma_enc_dist |
| 滑移检测 | `workbench.py` L119-121 | thresh_lon, thresh_lat, k_slip |
| 下压力 | `workbench.py` L124 | `downforce` (N) |
| 裙边摩擦 | `workbench.py` L125-126 | `skirt_mu`, `skirt_R` |
| 光电管半宽/前瞻 | `workbench.py` L140-143 | half_span, fwd_offset |
| 轮径/轮距 | `engine.py` L24-25 | WHEEL_RADIUS=0.0104, TRACK_WIDTH=0.090 |
| 轮胎摩擦 | `base.xml` L73 | `friction="1.3 0.08 0.01"` |
| 电机参数 | `motor_model.py` L18-38 | R, L, Kt, V_bus (**11.1V = 3S LiPo**), I_peak, tau_max |

## Critical Gotchas

- **`qfrc_applied` uses `=` not `+=`** — MuJoCo persists values across steps.
- **Steer no longer clamped by throttle** — `control_core.cpp` line 49-52: steer can exceed throttle for aggressive turning. Final u_L/u_R clamped to [-1,1] only.
- **Wheel symmetry is critical** — both wheels MUST have same Z offset in base.xml, otherwise car spins.
- **Motor noise disabled** — `cogging_amplitude=0`, `gear_noise_std=0` in motor_model.py. Enable for realism testing.
- **`xfrc_applied[:]` each step** — overwrites previous values; don't += before the =.

## Closed-Loop Debug Workflow (v/ω path)

**After EVERY code change, Claude MUST run this workflow before proposing further changes:**

### Step 1: Run headless eval
```bash
python scripts/eval_vw.py single --track 2019kansai --speed <target> --duration 12 --seed 42
```
- Use current `target_speed` from `SimParams` in `workbench.py`
- Duration must be enough for >=1 full lap or until line lost

### Step 2: Read and analyze the output
The script outputs structured metrics + a JSON one-liner:
- **max_lat_mm**: peak lateral error while line detected
- **mean_lat_mm / rms_lat_mm**: average lateral tracking quality
- **line_lost_t**: time of first line loss (null = never lost)
- **rms_v_err**: RMS(actual_speed - target) — velocity tracking quality
- **rms_w_err**: RMS(omega_ref - omega_meas) — omega tracking quality
- **tau_v_sat% / tau_w_sat%**: fraction of time at torque limit

Diagnosis:
- **Line never lost** → SUCCESS. Propose next incremental improvement.
- **Lost < 1s** → Immediate failure. Check: yaw drift at startup? lat_Kp wrong sign? Sensor reading broken? Jz/Bw misconfigured?
- **Lost 1-3s** → Mid-run failure. Usually turn-entry: check max|lat| growth before loss.
- **Lost > 3s** → Late divergence: check integrator windup, slow drift, tau saturation.

### Step 3: For omega-loop tuning, run step response
```bash
python scripts/eval_vw.py omega-step --track 2019kansai --speed 1.0 --omega-step-value 5.0
```
Outputs: steady-state error (%), overshoot (%), rise time 10-90 (ms).

### Step 4: For speed profiling, run sweep
```bash
python scripts/eval_vw.py sweep --track 2019kansai --speed-min 1.0 --speed-max 4.0 --step 0.5
```
Finds max survivable constant speed.

### Step 5: Diagnose and propose ONE change
Based on the metrics:
1. **Lat error large (|lat| > 30mm)** → increase `vw_lat_Kp`
2. **Lat error oscillating** → increase `vw_lat_Kd` or `skirt_mu`
3. **Speed oscillating** → reduce `vw_v_Kp` or `vw_v_Ki`
4. **Lost in sharp turn** → omega loop can't track: increase `vw_w_Kp` / `vw_w_max` / `skirt_mu`, or reduce `target_speed`
5. **tau_omega saturating > 50%** → increase `vw_w_max` or reduce omega demands
6. **Line lost from startup** → gentler speed ramp, check wheel symmetry

### Step 6: Apply ONE change, goto Step 1
Never change multiple parameters at once. Single-variable iteration is essential for understanding cause-effect.

### Quick Override Syntax
```bash
# Test with parameter override without editing workbench.py
python scripts/eval_vw.py single --speed 3.0 --vw-lat-Kp 400.0 --vw-w-Kp 0.08 --duration 12
```

## Reference

- Track data: `C:\Users\chj15\Desktop\RobotRace\路径优化\robotrace-shortcut-path-main\data\`
- Architecture plan: `C:\Users\chj15\.claude\plans\mujoco-ai-txt-golden-lollipop.md`
- MuJoCo 3 documentation: https://mujoco.readthedocs.io/
