#!/usr/bin/env python3
"""Debug chassis settling behavior."""
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import mujoco, numpy as np

BASE_XML = Path(__file__).resolve().parent.parent / "mujoco_models" / "micromouse" / "base.xml"
xml = BASE_XML.read_text(encoding="utf-8").replace("<!-- WALLS -->", "")
model = mujoco.MjModel.from_xml_string(xml)
data = mujoco.MjData(model)

chassis_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "chassis")

# Print initial state
print(f"Initial qpos: {data.qpos}")
print(f"nq={model.nq} nv={model.nv}")
print(f"Freejoint qpos[0:7] = {data.qpos[:7]}")

# Gear ratios
for i in range(model.nu):
    print(f"Actuator {i}: gear={model.actuator_gear[i]} ctrlrange={model.actuator_ctrlrange[i]}")

# List all geoms and their positions in world
print("\nAll geoms:")
for i in range(model.ngeom):
    gid = i
    name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, gid) or f"geom_{gid}"
    pos = data.geom_xpos[gid]
    size = model.geom_size[gid]
    friction = model.geom_friction[gid]
    print(f"  {name}: pos=({pos[0]:.4f},{pos[1]:.4f},{pos[2]:.4f}) size={size} friction={friction}")

# Settle slowly
print("\nSettling...")
for i in range(5000):
    data.xfrc_applied[chassis_id] = [0, 0, -5.0, 0, 0, 0]
    mujoco.mj_step(model, data)
    if i % 500 == 0:
        t = data.qpos[2]  # chassis z
        # Get skid and wheel geom positions
        skid_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "skid")
        wl_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "wheel_L_geom")
        skid_z = data.geom_xpos[skid_id][2]
        wl_z = data.geom_xpos[wl_id][2]
        # Penetration = geom bottom - 0 (ground)
        skid_bottom = skid_z - model.geom_size[skid_id][0]  # sphere radius
        wl_bottom = wl_z - model.geom_size[wl_id][0]  # cylinder radius
        print(f"  step {i}: chassis_z={t:.4f} skid_bottom={skid_bottom:.4f} wL_bottom={wl_bottom:.4f} ncon={data.ncon}")

print(f"\nFinal chassis z: {data.qpos[2]:.4f}")
print(f"Final qpos: {data.qpos[:7]}")
