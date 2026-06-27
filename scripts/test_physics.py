#!/usr/bin/env python3
"""Quick physics test — verify the chassis moves with freejoint."""
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import mujoco
import numpy as np
from micromouse_sim.physics.engine import PhysicsEngine
from micromouse_sim.environment.loader import build_model_xml

BASE_XML = Path(__file__).resolve().parent.parent / "mujoco_models" / "micromouse" / "base.xml"

model_xml = build_model_xml(str(BASE_XML), track=None)
engine = PhysicsEngine(model_xml=model_xml, downforce=5.0)

# Open-loop torque
engine.set_control(0.005, 0.005)

dt = engine.timestep
n_steps = 50000  # 1 second

print(f"Timestep: {dt:.2e}s ({1/dt:.0f} Hz), Steps: {n_steps}")
print(f"Initial ctrl: {engine.data.ctrl}")

for i in range(n_steps):
    engine.step()
    if i % 10000 == 0:
        state = engine.get_state()
        print(f"t={state.time:.3f}s | pos=({state.pos[0]:.4f},{state.pos[1]:.4f},{state.pos[2]:.4f}) "
              f"v=({state.linvel[0]:.4f},{state.linvel[1]:.4f}) "
              f"wL={state.wheel_L_vel:.1f} wR={state.wheel_R_vel:.1f} rad/s")

state = engine.get_state()
print(f"\nFinal: pos=({state.pos[0]:.4f},{state.pos[1]:.4f}) "
      f"yaw={np.degrees(state.yaw):.1f}deg "
      f"v_fwd={state.forward_velocity:.4f} m/s")
print("SUCCESS" if abs(state.forward_velocity) > 0.01 else "FAIL: Car not moving")
