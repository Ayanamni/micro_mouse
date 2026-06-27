/// pybind11 entry point for localize_core module.
///
/// 5kHz Pipeline Localizer architecture:
///   Stage 0 (external): ImuProcessor — injects sensor noise
///   Stage 1-3 (internal): PipelineLocalizer — yaw integration + velocity Kalman + slip detect
///
/// Interface (pipeline pattern):
///   imu_step(gyro_raw, accel_raw, dt)  → noisy IMU data (external noise injection)
///   push_imu(gyro_z, accel_x, accel_y, dt) → 5kHz: yaw + v_fwd + position extrapolation
///   push_encoder(enc_L, enc_R, dt)     → 1kHz: Kalman update + position correction
///   read_pose()                         → latest pose (shared memory, zero-copy)

#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include "imu_processor.hpp"
#include "pipeline_localizer.hpp"
#include "shared/types.hpp"

namespace py = pybind11;
using namespace ms;

// ---- Module-level state (persistent across calls) ----
static ImuProcessor      g_imu;
static PipelineLocalizer g_pipeline;

// ============================================================================
// Stage 0: External noise injection (unchanged from original)
// ============================================================================
py::dict imu_step(py::array_t<double> gyro_raw,
                  py::array_t<double> accel_raw,
                  double dt) {
    auto gyro_buf  = gyro_raw.unchecked<1>();
    auto accel_buf = accel_raw.unchecked<1>();

    double g[3] = {gyro_buf(0), gyro_buf(1), gyro_buf(2)};
    double a[3] = {accel_buf(0), accel_buf(1), accel_buf(2)};

    ImuData result = g_imu.process(g, a, dt);

    py::dict out;
    out["gyro"]  = py::array_t<double>(3, result.gyro.data());
    out["accel"] = py::array_t<double>(3, result.accel.data());
    out["dt"]    = result.dt;
    return out;
}

// ============================================================================
// Stage 1: 5kHz IMU push
// ============================================================================
void push_imu(double gyro_z, double accel_x, double accel_y, double dt) {
    g_pipeline.push_imu(gyro_z, accel_x, accel_y, dt);
}

// ============================================================================
// Stage 2: 1kHz encoder push → Kalman update
// ============================================================================
void push_encoder(double enc_L, double enc_R, double dt_enc) {
    g_pipeline.push_encoder(enc_L, enc_R, dt_enc);
}

// ============================================================================
// Output: read latest pose (shared-memory style)
// ============================================================================
py::dict read_pose() {
    const auto& o = g_pipeline.read_pose();

    py::dict out;
    out["x"]          = o.x;
    out["y"]          = o.y;
    out["yaw"]        = o.yaw;
    out["v_fwd"]      = o.v_fwd;
    out["v_lat"]      = o.v_lat;
    out["w_z"]        = o.w_z;
    out["cov_v"]      = o.cov_v;
    out["cov_bias"]   = o.cov_bias;
    out["slip_scale"] = o.slip_scale;
    out["timestamp"]  = o.timestamp;
    out["sequence"]   = o.sequence;
    return out;
}

// ============================================================================
// Configuration
// ============================================================================
void set_calibration(double pulses_per_m_L, double pulses_per_m_R,
                     double accel_noise_std, double enc_dist_noise,
                     double track_width, double gyro_bias_init) {
    CalibrationParams cal;
    cal.pulses_per_m_L  = pulses_per_m_L;
    cal.pulses_per_m_R  = pulses_per_m_R;
    cal.track_width     = track_width;
    cal.gyro_bias_init  = gyro_bias_init;
    g_pipeline.set_calibration(cal);

    // Also update Kalman process noise to match
    VelocityKalmanParams kp;
    kp.sigma_accel      = accel_noise_std;
    kp.sigma_enc_dist   = enc_dist_noise;
    g_pipeline.set_kalman_params(kp);
}

void set_slip_params(double thresh_lon, double thresh_lat, double k_slip) {
    SlipDetectorParams sp;
    sp.thresh_lon = thresh_lon;
    sp.thresh_lat = thresh_lat;
    sp.k_slip     = k_slip;
    g_pipeline.set_slip_params(sp);
}

// ============================================================================
// Reset & Debug
// ============================================================================
void localize_reset(int seed = 42) {
    g_imu.reset(seed);
    g_pipeline.reset();
}

py::dict get_imu_bias() {
    const auto& b = g_imu.bias_state();
    py::dict out;
    out["gyro_bias"]  = py::array_t<double>(3, b.gyro_bias.data());
    out["accel_bias"] = py::array_t<double>(3, b.accel_bias.data());
    return out;
}

py::dict get_debug_state() {
    py::dict out;
    out["v_fwd"]       = g_pipeline.kalman().v_fwd();
    out["accel_bias"]  = g_pipeline.kalman().accel_bias_x();
    out["P00"]         = g_pipeline.kalman().P00();
    out["P11"]         = g_pipeline.kalman().P11();
    out["innovation"]  = g_pipeline.innovation();
    out["dist_accum"]  = g_pipeline.dist_accum();
    out["slip_lon"]    = g_pipeline.slip().last_lon_residual();
    out["slip_lat"]    = g_pipeline.slip().last_lat_residual();
    return out;
}

// ============================================================================
// Module definition
// ============================================================================
PYBIND11_MODULE(localize_core, m) {
    m.doc() = "5kHz Pipeline Localizer: IMU noise + gyro yaw + velocity Kalman + slip detection";

    // Stage 0: external noise injection
    m.def("imu_step", &imu_step,
          "Inject noise into raw IMU data (call at 5kHz)",
          py::arg("gyro_raw"), py::arg("accel_raw"), py::arg("dt"));

    // Stage 1: 5kHz IMU push
    m.def("push_imu", &push_imu,
          "Push one IMU sample through the pipeline (5kHz): yaw integration + Kalman predict + position extrapolation",
          py::arg("gyro_z"), py::arg("accel_x"), py::arg("accel_y"), py::arg("dt"));

    // Stage 2: 1kHz encoder push
    m.def("push_encoder", &push_encoder,
          "Push encoder data — triggers Kalman update (1kHz)",
          py::arg("enc_L"), py::arg("enc_R"), py::arg("dt_enc"));

    // Output
    m.def("read_pose", &read_pose,
          "Read latest pose estimate (shared memory, zero-copy)");

    // Configuration
    m.def("set_calibration", &set_calibration,
          "Set calibration parameters from real-world tests",
          py::arg("pulses_per_m_L"), py::arg("pulses_per_m_R"),
          py::arg("accel_noise_std"), py::arg("enc_dist_noise"),
          py::arg("track_width"), py::arg("gyro_bias_init"));

    m.def("set_slip_params", &set_slip_params,
          "Set slip detection thresholds",
          py::arg("thresh_lon"), py::arg("thresh_lat"), py::arg("k_slip"));

    // Lifecycle
    m.def("reset", &localize_reset,
          "Reset all internal state",
          py::arg("seed") = 42);

    // Debug
    m.def("get_bias", &get_imu_bias,
          "Return current IMU bias state (debug)");
    m.def("get_debug_state", &get_debug_state,
          "Return internal Kalman + slip detector state (debug)");
}
