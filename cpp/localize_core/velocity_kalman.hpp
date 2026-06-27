#pragma once
/// 2-state forward-velocity Kalman filter.
/// State: [v_fwd, accel_bias_x]^T  — nominal state, directly updated.
/// Error-state covariance P is 2x2, propagated at 5kHz, updated at 1kHz.
///
/// Measurement: encoder common-mode distance dist_enc = (dL + dR) / 2
///   H = [dt_enc, -0.5*dt_enc^2]
///   (bias is negative in H because positive bias reduces v_fwd in prediction)
///
/// Key features:
///   - Joseph-form covariance update (guaranteed positive-definite)
///   - Diagonal clamping on P (prevents floating-point runaway)
///   - All 2x2 matrix ops are hand-coded (zero dependencies)

#include <array>
#include <cstddef>

namespace ms {

struct VelocityKalmanParams {
    // Process noise
    double sigma_accel      = 0.01;    // m/s^2 per sqrt(Hz), from static accel test
    double sigma_bias_walk  = 3.0e-4;  // m/s^2 per sqrt(s), accel bias instability

    // Measurement noise (base, before slip scaling)
    double sigma_enc_dist   = 1.0e-5;  // m, encoder distance measurement noise

    // P clamping
    double max_var_v        = 1.0;     // (m/s)^2
    double max_var_bias     = 0.01;    // (m/s^2)^2
    double min_var          = 1e-12;
};

class VelocityKalman {
public:
    explicit VelocityKalman(const VelocityKalmanParams& params = {});

    /// Predict step: called at 5kHz.
    /// @param accel_x  Forward acceleration (body X, m/s^2), noise-injected
    /// @param dt       Time step (s), typically 200us
    void predict(double accel_x, double dt);

    /// Update step: called at 1kHz when encoder data arrives.
    /// @param dist_enc   Encoder common-mode distance this interval (m)
    /// @param dist_pred  Accumulated predicted distance from 5kHz steps (m)
    /// @param dt_enc     Encoder interval (s), typically 1ms
    /// @param R_eff      Effective measurement noise (R_base * slip_scale)
    void update(double dist_enc, double dist_pred, double dt_enc, double R_eff);

    /// Reset state and covariance to initial values.
    void reset();

    // ---- Accessors ----
    double v_fwd()        const { return v_fwd_; }
    double accel_bias_x() const { return accel_bias_x_; }
    double P00()          const { return P_[0]; }   // v_fwd variance
    double P11()          const { return P_[3]; }   // accel_bias variance
    const double* P_data() const { return P_.data(); }

    // For debugging: force-set bias (simulates real bias for testing)
    void set_accel_bias(double b) { accel_bias_x_ = b; }

    VelocityKalmanParams& params() { return params_; }

private:
    VelocityKalmanParams params_;

    // Nominal state
    double v_fwd_        = 0.0;
    double accel_bias_x_ = 0.0;

    // Error-state covariance (2x2, row-major)
    // P = [p00, p01; p10, p11]
    std::array<double, 4> P_;

    double sigma_accel_sq_;      // cached: sigma_accel^2
    double sigma_bias_walk_sq_;  // cached: sigma_bias_walk^2
};

} // namespace ms
