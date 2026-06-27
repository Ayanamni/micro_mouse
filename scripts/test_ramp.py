#!/usr/bin/env python3
"""Test with ramped torque and pitch damping."""
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import numpy as np
from micromouse_sim.physics.engine import PhysicsEngine

BASE_XML = Path(__file__).resolve().parent.parent / "mujoco_models" / "micromouse" / "base.xml"

xml = BASE_XML.read_text(encoding="utf-8").replace("<!-- WALLS -->", "")
engine = PhysicsEngine(xml, downforce=5.0)

# Settle without torque
print("Settling...")
engine.set_control(0.0, 0.0)
for i in range(10000):
    engine.step()
state = engine.get_state()
print(f"After settle: z={state.pos[2]:.4f}")

# Run with ramped torque
max_torque = 0.002  # 2 mNm per wheel
ramp_steps = 20000    # ramp over 0.4s
n_steps = 100000      # 2 seconds total

print(f"Running: max_torque={max_torque*1000:.1f}mNm, ramp over {ramp_steps*2e-5*1000:.0f}ms")

for i in range(n_steps):
    frac = min(1.0, i / ramp_steps)
    tau = max_torque * frac
    engine.set_control(tau, tau)
    engine.step()

    if i % 5000 == 0:
        state = engine.get_state()
        qw, qx, qy, qz = state.quat
        pitch = np.degrees(np.arcsin(np.clip(2*(qw*qy - qz*qx), -1, 1)))
        yaw = np.degrees(state.yaw)
        print(f"t={state.time:.3f}s pos=({state.pos[0]:.4f},{state.pos[1]:.4f},{state.pos[2]:.4f}) "
              f"v=({state.linvel[0]:.3f},{state.linvel[1]:.3f}) "
              f"pitch={pitch:.1f}deg yaw={yaw:.0f}deg tau={tau*1000:.1f}mNm "
              f"wL={state.wheel_L_vel:.1f} wR={state.wheel_R_vel:.1f}")

state = engine.get_state()
print(f"\nFinal: x={state.pos[0]:.4f}m v_fwd={state.forward_velocity:.3f}m/s pitch={np.degrees(np.arcsin(np.clip(2*(state.quat[0]*state.quat[2] - state.quat[3]*state.quat[1]), -1, 1))):.1f}deg")
print("STABLE PITCH" if abs(pitch) < 15 else "Pitch oscillating")
