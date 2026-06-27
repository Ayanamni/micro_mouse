#pragma once
/// Shared type definitions for micromouse simulation C++ modules.
/// These mirror the data structures exchanged between the Python physics
/// engine and the C++ control/localization cores.

#include <cstdint>
#include <array>

namespace ms {

// ---- IMU data (body frame) ------------------------------------------------
struct ImuData {
    std::array<double, 3> gyro;     // rad/s, body frame
    std::array<double, 3> accel;    // m/s², body frame
    double dt;                      // seconds since last sample
};

// ---- Encoder data ----------------------------------------------------------
struct EncoderData {
    double pos_L;     // rad
    double vel_L;     // rad/s
    double pos_R;     // rad
    double vel_R;     // rad/s
    double dt;        // seconds
};

// ---- 2D Pose --------------------------------------------------------------
struct Pose2D {
    double x;       // m
    double y;       // m
    double yaw;     // rad
    double v_fwd;   // m/s, forward velocity (body frame)
    double v_lat;   // m/s, lateral velocity (body frame)
    double w_z;     // rad/s, yaw rate
};

// ---- Full pose with covariance --------------------------------------------
struct PoseEstimate {
    double x, y, yaw;
    double v_fwd, v_lat;
    // 3x3 covariance matrix (row-major): xx, xy, xyaw, yx, yy, yyaw, yawx, yawy, yawyaw
    std::array<double, 9> cov;
};

// ---- Line sensor ADC readings ---------------------------------------------
static constexpr int LINE_SENSOR_COUNT = 16;
using LineAdc = std::array<uint16_t, LINE_SENSOR_COUNT>;

// ---- Control command -------------------------------------------------------
struct ControlCmd {
    double u_L;   // normalized [-1, 1], left motor
    double u_R;   // normalized [-1, 1], right motor
};

// ---- Odometry delta --------------------------------------------------------
struct OdometryDelta {
    double dist_L;   // m, left wheel travel distance this step
    double dist_R;   // m, right wheel travel distance this step
};

// ---- Pipeline localizer output (shared memory struct) -----------------------
struct LocalizeOutput {
    double x       = 0.0;   // m, world frame
    double y       = 0.0;   // m
    double yaw     = 0.0;   // rad, [-pi, pi]
    double v_fwd   = 0.0;   // m/s, body forward
    double v_lat   = 0.0;   // m/s, body lateral (always 0 — not directly observable)
    double w_z     = 0.0;   // rad/s, yaw rate
    double cov_v   = 0.0;   // (m/s)^2, v_fwd variance (P[0])
    double cov_bias= 0.0;   // (m/s^2)^2, accel_bias variance (P[3])
    double slip_scale = 1.0;// current slip factor (1.0 = normal)
    double timestamp = 0.0; // s, last update time
    uint32_t sequence = 0;  // incrementing counter for freshness detection
};

// ---- Calibration parameters (from real-world tests) -------------------------
struct CalibrationParams {
    double pulses_per_m_L = 60606.0;  // encoder counts per meter, left wheel
    double pulses_per_m_R = 60606.0;  // encoder counts per meter, right wheel
    double track_width    = 0.050;    // m, effective track width
    double gyro_bias_init = 0.0;      // rad/s, stationary gyro_z mean
};

} // namespace ms
