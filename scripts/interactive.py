#!/usr/bin/env python3
"""Interactive micromouse simulation. Arrow keys drive, Q/E gear, F follow-cam."""
import sys
from pathlib import Path
import mujoco, mujoco.viewer, numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from micromouse_sim.physics.engine import PhysicsEngine
from micromouse_sim.environment.loader import build_model_xml, load_track

PROJECT_ROOT = Path(__file__).resolve().parent.parent
BASE_XML = PROJECT_ROOT / "mujoco_models" / "micromouse" / "base.xml"
_tp = Path(r"C:\Users\chj15\Desktop\RobotRace\路径优化\robotrace-shortcut-path-main\data")
if not _tp.exists():
    _tp = PROJECT_ROOT.parent / "路径优化" / "robotrace-shortcut-path-main" / "data"
TRACK_DATA_DIR = _tp

# Key codes
K_UP=265; K_DOWN=264; K_LEFT=263; K_RIGHT=262
K_Q=81; K_E=69; K_R=82; K_F=70; K_SPC=32; K_ESC=256


class State:
    common = 0.0       # -1..1  common-mode torque
    diff = 0.0         # -1..1  differential torque
    gear = 1           # 1-5
    follow = False
    reset = False
    quit = False


_state = State()


def key_cb(keycode: int):
    s = _state
    if keycode == K_UP:       s.common = min(1.0, s.common + 0.25)
    elif keycode == K_DOWN:   s.common = max(-1.0, s.common - 0.25)
    elif keycode == K_LEFT:   s.diff = max(-1.0, s.diff - 0.25)
    elif keycode == K_RIGHT:  s.diff = min(1.0, s.diff + 0.25)
    elif keycode == K_Q:      s.gear = min(5, s.gear + 1)
    elif keycode == K_E:      s.gear = max(1, s.gear - 1)
    elif keycode == K_SPC:    s.common = 0.0; s.diff = 0.0
    elif keycode == K_R:      s.reset = True
    elif keycode == K_F:      s.follow = not s.follow
    elif keycode == K_ESC:    s.quit = True
    if keycode in (K_UP, K_DOWN, K_LEFT, K_RIGHT, K_Q, K_E, K_SPC):
        print(f"  common={s.common:+.2f} diff={s.diff:+.2f} gear={s.gear}" +
              (f"  follow={'ON' if s.follow else 'OFF'}" if keycode == K_F else ""))


def main():
    track = load_track(str(TRACK_DATA_DIR / "robotena_points.txt"))
    print(f"Track: {track.total_length:.2f}m, {track.waypoints.shape[0]} pts")

    model_xml = build_model_xml(str(BASE_XML), track=track, track_width=0.180)
    engine = PhysicsEngine(model_xml=model_xml, downforce=5.0)

    print("Settling...")
    for _ in range(5000):
        engine.step()
    print("Ready!")

    viewer = mujoco.viewer.launch_passive(
        engine.model, engine.data, key_callback=key_cb,
        show_left_ui=False, show_right_ui=False)
    viewer.cam.azimuth = 135; viewer.cam.elevation = -30
    viewer.cam.distance = 1.0; viewer.cam.lookat[:] = [0.3, 0, 0.01]

    base_torque = 0.006   # Nm per gear (common)
    diff_gain = 0.010     # Nm per gear (differential)
    sim_t = 0.0; last_print = 0.0; settle_n = 5000

    print("\n  [Arrows] Drive  [Q/E] Gear  [Space] Stop  [R] Reset  [F] Cam  [Esc] Quit\n")

    while viewer.is_running():
        s = _state
        if s.quit: break
        if s.reset:
            engine.reset()
            for _ in range(settle_n): engine.step()
            sim_t = 0.0; s.common = 0.0; s.diff = 0.0; s.reset = False
            print("  [RESET]")

        cm = s.gear * base_torque * s.common
        dm = s.gear * diff_gain * s.diff
        engine.set_control(cm + dm, cm - dm)
        engine.step()

        if s.follow:
            x, y, yaw = engine.get_chassis_pose_2d()
            viewer.cam.lookat[:] = [x, y, 0.01]
            viewer.cam.azimuth += (np.degrees(yaw) - 90 - viewer.cam.azimuth) * 0.15

        sim_t += engine.timestep
        if sim_t - last_print >= 0.5:
            st = engine.get_state()
            try: _, le, _ = track.project(np.array([st.pos[0], st.pos[1]]))
            except: le = 0.0
            print(f"  t={sim_t:.1f}s spd={st.forward_velocity:.1f}m/s "
                  f"cmd=({cm*1000:+.0f}±{dm*1000:+.0f})mNm lat={le*1000:.0f}mm "
                  f"gear={s.gear} cam={'FOLLOW' if s.follow else 'FREE'}")
            last_print = sim_t

        viewer.sync()

    viewer.close()
    print(f"Done. t={sim_t:.1f}s")


if __name__ == "__main__":
    main()
