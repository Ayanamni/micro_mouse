#include "pipeline_localizer.hpp"
#include "shared/math_utils.hpp"
#include <cmath>
#include <algorithm>

namespace ms {

PipelineLocalizer::PipelineLocalizer() {
    reset();
}

// ============================================================================
// 5kHz: push_imu
// ============================================================================
void PipelineLocalizer::push_imu(double gyro_z, double accel_x, double accel_y, double dt) {
    // ---- 1. Pure gyro yaw integration ----
    double gyro_corrected = gyro_z - cal_.gyro_bias_init;
    yaw_ = math::wrap_angle(yaw_ + gyro_corrected * dt);

    // ---- 2. Forward-velocity Kalman predict (a_x feedforward) ----
    kalman_.predict(accel_x, dt);

    // ---- 3. 5kHz position extrapolation ----
    double v = kalman_.v_fwd();
    double cos_yaw = std::cos(yaw_);
    double sin_yaw = std::sin(yaw_);
    x_ += v * cos_yaw * dt;
    y_ += v * sin_yaw * dt;

    // ---- 4. Accumulate for encoder-interval comparison ----
    dist_accum_ += v * dt;

    // ---- 5. Accumulate for slip detection ----
    accel_sum_x_ += accel_x;
    accel_sum_y_ += accel_y;
    gyro_sum_z_  += gyro_corrected;
    slip_sample_cnt_++;

    // ---- 6. Update output buffer ----
    output_.x      = x_;
    output_.y      = y_;
    output_.yaw    = yaw_;
    output_.v_fwd  = v;
    output_.v_lat  = 0.0;  // not directly observable
    output_.w_z    = gyro_corrected;
    output_.cov_v    = kalman_.P00();
    output_.cov_bias = kalman_.P11();
    output_.slip_scale = last_slip_scale_;
    // timestamp and sequence updated at encoder interval
}

// ============================================================================
// 1kHz: push_encoder
// ============================================================================
void PipelineLocalizer::push_encoder(double enc_L, double enc_R, double dt_enc) {
    if (dt_enc <= 0.0) return;

    // ---- 1. Encoder deltas → common-mode distance ----
    double dL_rad = enc_L - prev_enc_L_;
    double dR_rad = enc_R - prev_enc_R_;
    prev_enc_L_ = enc_L;
    prev_enc_R_ = enc_R;

    double dL_m = dL_rad * meters_per_count_L_;
    double dR_m = dR_rad * meters_per_count_R_;
    double dist_enc = (dL_m + dR_m) * 0.5;

    // ---- 2. Slip detection (velocity layer — no 2nd derivative!) ----
    double v_enc = dist_enc / dt_enc;
    double v_imu = (slip_sample_cnt_ > 0) ? (dist_accum_ / dt_enc) : 0.0;
    double a_y_avg = (slip_sample_cnt_ > 0) ? (accel_sum_y_ / slip_sample_cnt_) : 0.0;
    double w_z_avg = (slip_sample_cnt_ > 0) ? (gyro_sum_z_ / slip_sample_cnt_) : 0.0;
    double w_z_out = w_z_avg;

    double slip_scale = slip_.compute(v_enc, v_imu, a_y_avg, kalman_.v_fwd(), w_z_avg);
    last_slip_scale_ = slip_scale;

    // ---- 3. Effective measurement noise ----
    double R_base = kalman_.params().sigma_enc_dist;
    R_base *= R_base;  // variance
    double R_eff = R_base * slip_scale;

    // ---- 4. Kalman update ----
    double dist_pred = dist_accum_;
    kalman_.update(dist_enc, dist_pred, dt_enc, R_eff);
    last_innovation_ = dist_enc - dist_pred;

    // ---- 5. Position fine-correction ----
    // dist_enc - dist_accum is the distance error from using pre-correction v_fwd.
    // Apply correction along current heading.
    double dist_correction = dist_enc - dist_pred;
    double cos_yaw = std::cos(yaw_);
    double sin_yaw = std::sin(yaw_);
    x_ += dist_correction * cos_yaw;
    y_ += dist_correction * sin_yaw;

    // ---- 6. Reset accumulators for next interval ----
    dist_accum_      = 0.0;
    accel_sum_x_     = 0.0;
    accel_sum_y_     = 0.0;
    gyro_sum_z_      = 0.0;
    slip_sample_cnt_ = 0;

    // ---- 7. Update output buffer with post-correction values ----
    output_.x      = x_;
    output_.y      = y_;
    output_.yaw    = yaw_;
    output_.v_fwd  = kalman_.v_fwd();
    output_.v_lat  = 0.0;
    output_.w_z    = w_z_out;
    output_.cov_v    = kalman_.P00();
    output_.cov_bias = kalman_.P11();
    output_.slip_scale = slip_scale;
    output_.timestamp += dt_enc;
    output_.sequence  = ++seq_;
}

// ============================================================================
// Configuration
// ============================================================================
void PipelineLocalizer::set_calibration(const CalibrationParams& cal) {
    cal_ = cal;
    // Convert pulses-per-meter → meters-per-count
    meters_per_count_L_ = 1.0 / cal_.pulses_per_m_L;
    meters_per_count_R_ = 1.0 / cal_.pulses_per_m_R;
}

void PipelineLocalizer::set_kalman_params(const VelocityKalmanParams& kp) {
    kalman_ = VelocityKalman(kp);
}

void PipelineLocalizer::set_slip_params(const SlipDetectorParams& sp) {
    slip_ = SlipDetector(sp);
}

// ============================================================================
// Reset
// ============================================================================
void PipelineLocalizer::reset() {
    x_ = 0.0;
    y_ = 0.0;
    yaw_ = 0.0;

    dist_accum_      = 0.0;
    accel_sum_x_     = 0.0;
    accel_sum_y_     = 0.0;
    gyro_sum_z_      = 0.0;
    slip_sample_cnt_ = 0;

    prev_enc_L_ = 0.0;
    prev_enc_R_ = 0.0;

    kalman_.reset();
    output_ = LocalizeOutput{};
    seq_ = 0;
    last_innovation_ = 0.0;
    last_slip_scale_ = 1.0;
}

} // namespace ms
