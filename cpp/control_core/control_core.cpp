/// pybind11 entry point for control_core module.
/// Exposes:
///   - line sensor processing (legacy)
///   - lateral PID + speed PI (legacy, kept for A/B comparison)
///   - v-ω decoupled controller (NEW — primary control path)
///
/// v-ω 接口：
///   vw_omega_step(gyro_z, dt) @ 5kHz  → 更新角速度环内部状态
///   vw_control_tick(lat_err, curv, v_fwd, v_ref, dt) @ 1kHz
///     → 巡线+速度环+混控 → {u_L, u_R, throttle, steer, tau_v, tau_omega}
///   vw_set_params(...)  → 在线调参
///   vw_get_debug()      → 所有内部状态，供仪表盘显示

#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include "line_sensor.hpp"
#include "lateral_controller.hpp"
#include "speed_controller.hpp"
#include "vw_controller.hpp"

namespace py = pybind11;
using namespace ms;

// ═══════════════════════════════════════════════════════════════
// Legacy module-level state (backward compatible)
// ═══════════════════════════════════════════════════════════════
static LineSensor         g_line_sensor;
static LateralController  g_lat_ctrl;
static SpeedController    g_spd_ctrl;

// ═══════════════════════════════════════════════════════════════
// NEW: v-ω 解耦控制器
// ═══════════════════════════════════════════════════════════════
static VWController g_vw_ctrl;

// ═══════════════════════════════════════════════════════════════
// Legacy bindings (keep for backward compatibility)
// ═══════════════════════════════════════════════════════════════

py::dict line_sensor_read(py::array_t<double> lateral_dist) {
    auto buf = lateral_dist.unchecked<1>();
    std::array<double, LINE_SENSOR_COUNT> dists{};
    for (int i = 0; i < LINE_SENSOR_COUNT; ++i) {
        dists[i] = buf(i);
    }

    LineSensorResult result = g_line_sensor.process(dists);

    py::dict out;
    out["adc"]           = py::array_t<uint16_t>(LINE_SENSOR_COUNT, result.adc.data());
    out["lateral_error"] = result.lateral_error;
    out["line_visible"]  = result.line_visible;
    return out;
}

py::dict control_step(double lateral_error,
                      double curvature,
                      double current_speed,
                      double target_speed,
                      double dt) {
    double steer = g_lat_ctrl.update(lateral_error, curvature, dt);
    double throttle = g_spd_ctrl.update(current_speed, target_speed, dt);

    double u_L = throttle + steer;
    double u_R = throttle - steer;
    u_L = std::max(-1.0, std::min(1.0, u_L));
    u_R = std::max(-1.0, std::min(1.0, u_R));

    py::dict out;
    out["u_L"]      = u_L;
    out["u_R"]      = u_R;
    out["throttle"] = throttle;
    out["steer"]    = steer;
    return out;
}

void control_reset() {
    g_lat_ctrl.reset();
    g_spd_ctrl.reset();
}

void set_lateral_gains(double Kp, double Kd, double Ki, double Kff) {
    g_lat_ctrl = LateralController({Kp, Kd, Ki, Kff, 1.0});
}

void set_speed_gains(double Kp, double Ki) {
    g_spd_ctrl = SpeedController({Kp, Ki, 1.0});
}

// ═══════════════════════════════════════════════════════════════
// NEW: v-ω controller bindings
// ═══════════════════════════════════════════════════════════════

/// 5kHz 角速度环步进。每次 IMU 采样调用。
void vw_omega_step(double gyro_z_raw, double dt_imu) {
    g_vw_ctrl.omega_step(static_cast<float>(gyro_z_raw), static_cast<float>(dt_imu));
}

/// 1kHz 控制节拍：巡线 → ω_ref → v 环 → 混控 → 输出
py::dict vw_control_tick(double lateral_error, double curvature,
                         double v_fwd, double v_ref, double dt_ctrl) {
    VWControlResult r = g_vw_ctrl.control_tick(
        static_cast<float>(lateral_error),
        static_cast<float>(curvature),
        static_cast<float>(v_fwd),
        static_cast<float>(v_ref),
        static_cast<float>(dt_ctrl));

    py::dict out;
    out["u_L"]       = r.u_L;
    out["u_R"]       = r.u_R;
    out["throttle"]  = r.throttle;
    out["steer"]     = r.steer;
    out["tau_v"]     = r.tau_v;
    out["tau_omega"] = r.tau_omega;
    out["tau_v_raw"]     = r.tau_v_raw;
    out["tau_omega_raw"] = r.tau_omega_raw;
    out["tau_L_cmd"] = r.tau_L_cmd;
    out["tau_R_cmd"] = r.tau_R_cmd;
    out["tau_limit_L_pos"] = r.tau_limit_L_pos;
    out["tau_limit_L_neg"] = r.tau_limit_L_neg;
    out["tau_limit_R_pos"] = r.tau_limit_R_pos;
    out["tau_limit_R_neg"] = r.tau_limit_R_neg;
    out["sat_v"] = r.sat_v;
    out["sat_w"] = r.sat_w;
    return out;
}

/// 在线设置所有 v-ω 参数。
void vw_set_params(
    // 几何
    double wheel_r, double track_B,
    // ω 环（物理前馈：τ_ff = (Jz*r/B)*α + (Dw*r/B)*ω）
    double Jz, double Dw,
    double w_Kp, double w_Ki, double w_Kd, double w_max,
    // v 环（物理前馈：τ_ff = (m_eq * r/2) * a + (D_v * r/2) * v + C_frict）
    double m_eq, double D_v, double C_frict,
    double v_Kp, double v_Ki, double v_max,
    // 巡线
    double lat_Kp, double lat_Ki, double lat_Kd, double lat_Kff,
    // 滤波
    double gyro_lpf_fc,
    // 电机电气（电压前馈）
    double motor_R, double motor_Kt, double motor_Ke,
    double motor_G, double motor_eta, double motor_V_bus,
    // 2DOF / anti-windup / actuator limit / yaw Coulomb feedforward
    double beta_w, double beta_v, double kaw_w, double kaw_v,
    double motor_I_peak, double w_Cfrict)
{
    VWParams p;
    p.wheel_r = static_cast<float>(wheel_r);
    p.track_B = static_cast<float>(track_B);
    p.Jz = static_cast<float>(Jz);
    p.Dw = static_cast<float>(Dw);
    p.w_Kp = static_cast<float>(w_Kp);
    p.w_Ki = static_cast<float>(w_Ki);
    p.w_Kd = static_cast<float>(w_Kd);
    p.w_max = static_cast<float>(w_max);
    p.m_eq = static_cast<float>(m_eq);
    p.D_v  = static_cast<float>(D_v);
    p.C_frict = static_cast<float>(C_frict);
    p.v_Kp = static_cast<float>(v_Kp);
    p.v_Ki = static_cast<float>(v_Ki);
    p.v_max = static_cast<float>(v_max);
    p.lat_Kp = static_cast<float>(lat_Kp);
    p.lat_Ki = static_cast<float>(lat_Ki);
    p.lat_Kd = static_cast<float>(lat_Kd);
    p.lat_Kff = static_cast<float>(lat_Kff);
    p.gyro_lpf_fc = static_cast<float>(gyro_lpf_fc);
    p.motor_R     = static_cast<float>(motor_R);
    p.motor_Kt    = static_cast<float>(motor_Kt);
    p.motor_Ke    = static_cast<float>(motor_Ke);
    p.motor_G     = static_cast<float>(motor_G);
    p.motor_eta   = static_cast<float>(motor_eta);
    p.motor_V_bus = static_cast<float>(motor_V_bus);
    p.beta_w      = static_cast<float>(beta_w);
    p.beta_v      = static_cast<float>(beta_v);
    p.kaw_w       = static_cast<float>(kaw_w);
    p.kaw_v       = static_cast<float>(kaw_v);
    p.motor_I_peak = static_cast<float>(motor_I_peak);
    p.w_Cfrict    = static_cast<float>(w_Cfrict);
    g_vw_ctrl.set_params(p);
}

/// 更新轮速（电压前馈需要，每控制周期调用）
void vw_set_wheel_omega(double wL, double wR) {
    g_vw_ctrl.set_wheel_omega(static_cast<float>(wL), static_cast<float>(wR));
}

void vw_reset() {
    g_vw_ctrl.reset();
}

void vw_override_omega_ref(double omega_ref) {
    g_vw_ctrl.override_omega_ref(static_cast<float>(omega_ref));
}

void vw_clear_omega_override() {
    g_vw_ctrl.clear_omega_override();
}

/// 获取全部调试状态（供仪表盘显示）。
py::dict vw_get_debug() {
    py::dict d;
    // ω 环
    d["omega_meas"]    = g_vw_ctrl.omega().omega_meas();
    d["omega_ref"]     = g_vw_ctrl.omega().omega_ref();
    d["omega_error"]   = g_vw_ctrl.omega().omega_error();
    d["omega_tau_ff"]  = g_vw_ctrl.omega().tau_ff();
    d["omega_tau_fb"]  = g_vw_ctrl.omega().tau_fb();
    d["omega_integ"]   = g_vw_ctrl.omega().integrator();
    d["omega_deriv_f"] = g_vw_ctrl.omega().deriv_filt();
    // v 环
    d["v_ref"]         = g_vw_ctrl.velocity().v_ref();
    d["v_error"]       = g_vw_ctrl.velocity().v_error();
    d["v_tau_ff"]      = g_vw_ctrl.velocity().tau_ff();
    d["v_tau_fb"]      = g_vw_ctrl.velocity().tau_fb();
    d["v_integ"]       = g_vw_ctrl.velocity().integrator();
    // 巡线
    d["track_omega_fb"] = g_vw_ctrl.tracking().omega_fb();
    d["lat_raw"]   = g_vw_ctrl.tracking().lat_error_raw();
    d["lat_filt"]  = g_vw_ctrl.tracking().lat_error_filt();
    // 混控
    const auto& r = g_vw_ctrl.last_result();
    d["tau_v"]     = r.tau_v;
    d["tau_omega"] = r.tau_omega;
    d["tau_v_raw"] = r.tau_v_raw;
    d["tau_omega_raw"] = r.tau_omega_raw;
    d["tau_L_cmd"] = r.tau_L_cmd;
    d["tau_R_cmd"] = r.tau_R_cmd;
    d["tau_limit_L_pos"] = r.tau_limit_L_pos;
    d["tau_limit_L_neg"] = r.tau_limit_L_neg;
    d["tau_limit_R_pos"] = r.tau_limit_R_pos;
    d["tau_limit_R_neg"] = r.tau_limit_R_neg;
    d["sat_v"] = r.sat_v;
    d["sat_w"] = r.sat_w;
    return d;
}

// ═══════════════════════════════════════════════════════════════
// Module definition
// ═══════════════════════════════════════════════════════════════

PYBIND11_MODULE(control_core, m) {
    m.doc() = "Control core: line sensor, lateral PID, speed PI, v-ω decoupled controller";

    // ── Legacy API (backward compatible) ──
    m.def("line_read", &line_sensor_read,
          "Process 16-ch lateral distances → ADC + centroid error",
          py::arg("lateral_dist"));

    m.def("step", &control_step,
          "Legacy: single PID control step (call at 1kHz)",
          py::arg("lateral_error"), py::arg("curvature"),
          py::arg("current_speed"), py::arg("target_speed"),
          py::arg("dt"));

    m.def("reset", &control_reset, "Reset legacy controller state");
    m.def("set_lateral_gains", &set_lateral_gains,
          "Set legacy lateral PID gains", py::arg("Kp"), py::arg("Kd"), py::arg("Ki"), py::arg("Kff"));
    m.def("set_speed_gains", &set_speed_gains,
          "Set legacy speed PI gains", py::arg("Kp"), py::arg("Ki"));

    // ── NEW: v-ω decoupled controller API ──
    m.def("vw_omega_step", &vw_omega_step,
          "Omega-loop step @ 5kHz: filter gyro, compute τ_ω",
          py::arg("gyro_z_raw"), py::arg("dt_imu"));

    m.def("vw_control_tick", &vw_control_tick,
          "v-ω control tick @ 1kHz: track → v-loop → mix → {u_L, u_R, ...}",
          py::arg("lateral_error"), py::arg("curvature"),
          py::arg("v_fwd"), py::arg("v_ref"), py::arg("dt_ctrl"));

    m.def("vw_set_params", &vw_set_params,
          "Set all v-ω controller parameters (online tuning)",
          py::arg("wheel_r")=0.0105, py::arg("track_B")=0.090,
          py::arg("Jz")=7.9e-5, py::arg("Dw")=2.1e-3,
          py::arg("w_Kp")=0.05, py::arg("w_Ki")=0.03, py::arg("w_Kd")=0.0,
          py::arg("w_max")=0.05,
          py::arg("m_eq")=0.11, py::arg("D_v")=0.2, py::arg("C_frict")=0.0016,
          py::arg("v_Kp")=1.0, py::arg("v_Ki")=0.3, py::arg("v_max")=0.05,
          py::arg("lat_Kp")=3000.0, py::arg("lat_Ki")=0.0,
          py::arg("lat_Kd")=0.0, py::arg("lat_Kff")=0.0,
          py::arg("gyro_lpf_fc")=80.0,
          py::arg("motor_R")=0.344, py::arg("motor_Kt")=0.00241,
          py::arg("motor_Ke")=0.00241, py::arg("motor_G")=4.0,
          py::arg("motor_eta")=0.88, py::arg("motor_V_bus")=11.1,
          py::arg("beta_w")=1.0, py::arg("beta_v")=1.0,
          py::arg("kaw_w")=0.5, py::arg("kaw_v")=0.5,
          py::arg("motor_I_peak")=10.0, py::arg("w_Cfrict")=0.006);

    m.def("vw_set_wheel_omega", &vw_set_wheel_omega,
          "Set wheel angular velocities for voltage feedforward (rad/s)",
          py::arg("wL"), py::arg("wR"));

    m.def("vw_reset", &vw_reset, "Reset v-ω controller state");

    m.def("vw_override_omega_ref", &vw_override_omega_ref,
          "Override ω_ref from external source (bypass tracking controller)",
          py::arg("omega_ref"));

    m.def("vw_clear_omega_override", &vw_clear_omega_override,
          "Clear ω_ref override — resume internal tracking controller");

    m.def("vw_get_debug", &vw_get_debug,
          "Get all v-ω internal states for dashboard display");
}
