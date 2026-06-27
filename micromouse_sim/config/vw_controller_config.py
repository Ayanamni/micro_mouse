"""Shared v/omega controller parameter plumbing.

This module is the single Python-side place that knows the pybind
``vw_set_params`` signature. Entrypoints should pass their parameter object here
instead of calling the positional binding directly.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

from micromouse_sim.actuation.motor_model import MotorParams


@dataclass(frozen=True)
class VWControllerDefaults:
    vw_wheel_r: float = 0.0105
    vw_track_B: float = 0.090
    vw_Jz: float = 7.9e-5
    vw_Dw: float = 2.1e-3
    vw_w_Kp: float = 0.05
    vw_w_Ki: float = 0.03
    vw_w_Kd: float = 0.0
    vw_w_max: float = 0.05
    vw_beta_w: float = 1.0
    vw_kaw_w: float = 0.5
    vw_m_eq: float = 0.11
    vw_D_v: float = 0.2
    vw_C_frict: float = 0.0016
    vw_v_Kp: float = 1.0
    vw_v_Ki: float = 0.3
    vw_v_max: float = 0.05
    vw_beta_v: float = 1.0
    vw_kaw_v: float = 0.5
    vw_lat_Kp: float = 3000.0
    vw_lat_Ki: float = 0.0
    vw_lat_Kd: float = 0.0
    vw_lat_Kff: float = 0.0
    vw_gyro_lpf_fc: float = 80.0
    vw_w_Cfrict: float = 0.006
    motor_I_peak: float = 10.0


VW_DEFAULTS = VWControllerDefaults()
VW_PARAM_NAMES = tuple(VWControllerDefaults.__dataclass_fields__.keys())


def _get(params: Any, name: str) -> float:
    if isinstance(params, dict):
        return float(params.get(name, getattr(VW_DEFAULTS, name)))
    return float(getattr(params, name, getattr(VW_DEFAULTS, name)))


def push_vw_params(control_core: Any, params: Any,
                   motor_params: MotorParams | None = None) -> None:
    """Push v/omega parameters through one keyword-based binding call."""
    mp = motor_params or MotorParams()
    control_core.vw_set_params(
        wheel_r=_get(params, "vw_wheel_r"),
        track_B=_get(params, "vw_track_B"),
        Jz=_get(params, "vw_Jz"),
        Dw=_get(params, "vw_Dw"),
        w_Kp=_get(params, "vw_w_Kp"),
        w_Ki=_get(params, "vw_w_Ki"),
        w_Kd=_get(params, "vw_w_Kd"),
        w_max=_get(params, "vw_w_max"),
        m_eq=_get(params, "vw_m_eq"),
        D_v=_get(params, "vw_D_v"),
        C_frict=_get(params, "vw_C_frict"),
        v_Kp=_get(params, "vw_v_Kp"),
        v_Ki=_get(params, "vw_v_Ki"),
        v_max=_get(params, "vw_v_max"),
        lat_Kp=_get(params, "vw_lat_Kp"),
        lat_Ki=_get(params, "vw_lat_Ki"),
        lat_Kd=_get(params, "vw_lat_Kd"),
        lat_Kff=_get(params, "vw_lat_Kff"),
        gyro_lpf_fc=_get(params, "vw_gyro_lpf_fc"),
        motor_R=mp.R,
        motor_Kt=mp.Kt,
        motor_Ke=mp.Ke,
        motor_G=mp.gear_ratio,
        motor_eta=mp.efficiency,
        motor_V_bus=mp.V_bus,
        beta_w=_get(params, "vw_beta_w"),
        beta_v=_get(params, "vw_beta_v"),
        kaw_w=_get(params, "vw_kaw_w"),
        kaw_v=_get(params, "vw_kaw_v"),
        motor_I_peak=_get(params, "motor_I_peak"),
        w_Cfrict=_get(params, "vw_w_Cfrict"),
    )
