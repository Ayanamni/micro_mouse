#!/usr/bin/env python3
"""Minimal test: pure MuJoCo contacts, no suspension. Start below equilibrium."""
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import mujoco, numpy as np

BASE_XML = Path(__file__).resolve().parent.parent / "mujoco_models" / "micromouse" / "base.xml"
xml = BASE_XML.read_text(encoding="utf-8").replace("<!-- WALLS -->", "")
model = mujoco.MjModel.from_xml_string(xml)
data = mujoco.MjData(model)

chassis_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "chassis")
pos_sid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, "pos")
pos_adr = model.sensor_adr[pos_sid]
lvel_sid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, "linvel")
lvel_adr = model.sensor_adr[lvel_sid]
enc_sid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, "enc_L_vel")
enc_adr = model.sensor_adr[enc_sid]

def run(tau_Nm, label):
    mujoco.mj_resetData(model, data)
    print(f"\n=== {label}: tau={tau_Nm*1000:.1f}mNm ===")

    # Gentle settle: apply downforce and let physics settle
    for i in range(10000):
        data.xfrc_applied[chassis_id] = [0, 0, -5.0, 0, 0, 0]
        # Pitch damping
        data.qfrc_applied[4] = -0.001 * data.qvel[4]
        mujoco.mj_step(model, data)
    z0 = data.qpos[2]
    x0 = data.sensordata[pos_adr]
    print(f"  After settle: z={z0:.4f} x={x0:.4f} ncon={data.ncon}")

    # Apply torque
    data.ctrl[0] = tau_Nm
    data.ctrl[1] = tau_Nm

    for i in range(10000):
        data.xfrc_applied[chassis_id] = [0, 0, -5.0, 0, 0, 0]
        data.qfrc_applied[4] = -0.001 * data.qvel[4]  # pitch damping
        mujoco.mj_step(model, data)
        if i % 2000 == 0:
            x = data.sensordata[pos_adr]
            vx = data.sensordata[lvel_adr]
            wL = data.sensordata[enc_adr]
            print(f"  t={i*2e-5:.3f}: x={x:.4f} vx={vx:.4f} z={data.qpos[2]:.4f} wL={wL:.1f}")

    xf = data.sensordata[pos_adr]
    vxf = data.sensordata[lvel_adr]
    print(f"  Final: dx={xf-x0:.4f}m vx={vxf:.4f}m/s")

run(0.005, "POS 5mNm")
run(0.010, "POS 10mNm")
run(0.020, "POS 20mNm")
