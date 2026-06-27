#!/usr/bin/env python3
"""Diagnose wheel-ground contact and friction."""
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import mujoco
import numpy as np

BASE_XML = Path(__file__).resolve().parent.parent / "mujoco_models" / "micromouse" / "base.xml"
xml = BASE_XML.read_text().replace("<!-- WALLS -->", "")
model = mujoco.MjModel.from_xml_string(xml)
data = mujoco.MjData(model)

chassis_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "chassis")

data.ctrl[0] = 0.01  # More torque
data.ctrl[1] = 0.01

# Let physics settle first (no torque, just gravity + downforce)
print("Settling (no torque)...")
for i in range(5000):
    data.xfrc_applied[chassis_id] = [0, 0, -5.0, 0, 0, 0]
    mujoco.mj_step(model, data)

# Check initial contact forces
print(f"After settle: chassis z={data.qpos[2]:.4f}")
for j in range(data.ncon):
    contact = data.contact[j]
    print(f"  Contact {j}: geom1={model.geom(contact.geom1).name} "
          f"geom2={model.geom(contact.geom2).name} "
          f"dist={contact.dist:.6f}")

# Now apply torque
data.ctrl[0] = 0.01
data.ctrl[1] = 0.01

print("\nRunning with torque...")
for i in range(5000):
    data.xfrc_applied[chassis_id] = [0, 0, -5.0, 0, 0, 0]
    mujoco.mj_step(model, data)
    if i % 1000 == 0:
        # Check contacts
        ncon = data.ncon
        wheel_contact = False
        for j in range(ncon):
            c = data.contact[j]
            g1 = model.geom(c.geom1).name
            g2 = model.geom(c.geom2).name
            if "wheel" in g1 or "wheel" in g2:
                # print(f"  t={data.time:.6f}: {g1}-{g2} dist={c.dist:.6f}")
                wheel_contact = True
        # Read pos
        sid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, "pos")
        adr = model.sensor_adr[sid]
        x, y, z = data.sensordata[adr:adr+3]
        sid_v = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, "linvel")
        adr_v = model.sensor_adr[sid_v]
        vx, vy, vz = data.sensordata[adr_v:adr_v+3]
        sid_wl = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, "enc_L_vel")
        wl = data.sensordata[model.sensor_adr[sid_wl]]
        print(f"  t={i*2e-5:.4f}s pos=({x:.4f},{y:.4f},{z:.4f}) "
              f"v=({vx:.4f},{vy:.4f}) wL={wl:.2f} ncon={ncon} wheels_touch={wheel_contact}")

print(f"\nFinal pos: x={x:.4f} vx={vx:.4f}")
