#pragma once
/// 5kHz Pipeline Localizer — top-level orchestrator.
///
/// Data flow:
///   push_imu(gz, ax, ay, dt_5k) @ 5kHz
///     → yaw += (gz - bias_init) * dt          (pure gyro integration)
///     → v_fwd a_x feedforward + Kalman predict
///     → x, y 5kHz extrapolation
///     → accumulate dist_pred, accel, gyro for slip detection
///
///   push_encoder(enc_L, enc_R, dt_1k) @ 1kHz
///     → dist_enc = (dL + dR) / 2
///     → slip_scale from SlipDetector (velocity layer, no 2nd-deriv!)
///     → Kalman update with R_eff = R_base * slip_scale
///     → position fine-correction
///     → reset accumulators
///
///   read_pose() @ anytime
///     → {x, y, yaw, v_fwd, v_lat, w_z, cov_v, cov_bias, slip_scale, ...}

#include <cstdint>
#include "shared/types.hpp"
#include "velocity_kalman.hpp"
#include "slip_detector.hpp"

namespace ms {

// CalibrationParams is defined in shared/types.hpp

class PipelineLocalizer {
public:
    PipelineLocalizer();

    /// @name Pipeline inputs
    /// @{

    /// 5kHz: push one IMU sample through the pipeline.
    /// @param gyro_z   Noise-injected gyro Z (rad/s, body frame)
    /// @param accel_x  Noise-injected accel X (m/s^2, body forward)
    /// @param accel_y  Noise-injected accel Y (m/s^2, body lateral)
    /// @param dt       Time step (s), typically 200us
    void push_imu(double gyro_z, double accel_x, double accel_y, double dt);

    /// 1kHz: push encoder data — triggers Kalman update.
    /// @param enc_L   Left wheel cumulative angle (rad)
    /// @param enc_R   Right wheel cumulative angle (rad)
    /// @param dt_enc  Time since last encoder update (s), typically 1ms
    void push_encoder(double enc_L, double enc_R, double dt_enc);

    /// @}

    /// @name Output (shared memory)
    /// @{

    /// Read latest pose estimate. Always returns immediately (no computation).
    const LocalizeOutput& read_pose() const { return output_; }

    /// @}

    /// @name Configuration
    /// @{

    /// Set calibration parameters (from real-world tests).
    void set_calibration(const CalibrationParams& cal);

    /// Set Kalman filter noise parameters.
    void set_kalman_params(const VelocityKalmanParams& kp);

    /// Set slip detector thresholds.
    void set_slip_params(const SlipDetectorParams& sp);

    /// Reset all state to origin.
    void reset();

    /// @}

    /// @name Debug accessors
    /// @{

    const VelocityKalman& kalman() const { return kalman_; }
    VelocityKalman& kalman() { return kalman_; }
    const SlipDetector& slip() const { return slip_; }
    const CalibrationParams& calibration() const { return cal_; }
    double dist_accum() const { return dist_accum_; }
    double innovation() const { return last_innovation_; }

    /// @}

private:
    // ---- Configuration ----
    CalibrationParams cal_;
    double meters_per_count_L_ = 1.0 / 60606.0;
    double meters_per_count_R_ = 1.0 / 60606.0;

    // ---- Sub-modules ----
    VelocityKalman kalman_;
    SlipDetector   slip_;

    // ---- State ----
    double x_     = 0.0;
    double y_     = 0.0;
    double yaw_   = 0.0;

    // ---- Accumulators (reset each encoder interval) ----
    double dist_accum_      = 0.0;  // Σ v_fwd * dt (predicted distance)
    double accel_sum_x_     = 0.0;
    double accel_sum_y_     = 0.0;
    double gyro_sum_z_      = 0.0;  // corrected gyro (after bias subtraction)
    int    slip_sample_cnt_ = 0;

    // ---- Encoder tracking ----
    double prev_enc_L_ = 0.0;
    double prev_enc_R_ = 0.0;

    // ---- Output buffer ----
    LocalizeOutput output_;
    uint32_t seq_ = 0;

    // ---- Diagnostics ----
    double last_innovation_ = 0.0;
    double last_slip_scale_ = 1.0;
};

} // namespace ms
