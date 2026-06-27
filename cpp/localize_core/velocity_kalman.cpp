#include "velocity_kalman.hpp"
#include "shared/math_utils.hpp"
#include <cmath>
#include <algorithm>

namespace ms {

VelocityKalman::VelocityKalman(const VelocityKalmanParams& params)
    : params_(params)
{
    sigma_accel_sq_      = params_.sigma_accel * params_.sigma_accel;
    sigma_bias_walk_sq_  = params_.sigma_bias_walk * params_.sigma_bias_walk;
    reset();
}

void VelocityKalman::reset() {
    v_fwd_        = 0.0;
    accel_bias_x_ = 0.0;
    // Initial uncertainty: moderate
    P_[0] = 0.01;  // v_fwd: 0.1 m/s std
    P_[1] = 0.0;
    P_[2] = 0.0;
    P_[3] = 1e-4;  // accel_bias: 0.01 m/s^2 std
}

void VelocityKalman::predict(double accel_x, double dt) {
    // ---- Nominal state: a_x feedforward ----
    v_fwd_ += (accel_x - accel_bias_x_) * dt;
    // accel_bias_x_ is modelled as slowly-varying (no nominal change in predict)

    // ---- Error-state covariance propagation ----
    // F = [[1, -dt], [0, 1]]
    // P = F * P * F^T + Q
    //
    // F*P = [[p00 - dt*p10,  p01 - dt*p11],
    //        [p10,            p11          ]]
    // (F*P)*F^T =
    // [[(p00-dt*p10) - dt*(p10-dt*p11),  p01 - dt*p11],
    //  [p10 - dt*p11,                    p11          ]]
    //
    // Let's compute step by step (2x2, this is cheap):
    double p00 = P_[0], p01 = P_[1], p10 = P_[2], p11 = P_[3];

    // F*P
    double fp00 = p00 - dt * p10;
    double fp01 = p01 - dt * p11;
    double fp10 = p10;
    double fp11 = p11;

    // (F*P) * F^T
    double q00 = sigma_accel_sq_ * dt;
    double q11 = sigma_bias_walk_sq_ * dt;

    P_[0] = fp00 - dt * fp10 + q00;          // fp00 - dt*fp10 = p00 - 2*dt*p10 + dt^2*p11
    P_[1] = fp01 - dt * fp11;                // = p01 - 2*dt*p11  -- wait no.
    // Let me redo this more carefully:

    // F*P:
    // fp00 = P[0] - dt*P[2]    = p00 - dt*p10
    // fp01 = P[1] - dt*P[3]    = p01 - dt*p11
    // fp10 = P[2]              = p10
    // fp11 = P[3]              = p11

    // (F*P)*F^T:
    // result[0][0] = fp00*1 + fp01*(-dt) = fp00 - dt*fp01
    //              = (p00 - dt*p10) - dt*(p01 - dt*p11)
    //              = p00 - dt*p10 - dt*p01 + dt^2*p11
    // result[0][1] = fp00*0 + fp01*1 = fp01 = p01 - dt*p11
    // result[1][0] = fp10*1 + fp11*(-dt) = p10 - dt*p11
    // result[1][1] = fp10*0 + fp11*1 = p11

    double new_p00 = fp00 - dt * fp01 + q00;
    double new_p01 = fp01;
    double new_p10 = fp10 - dt * fp11;
    double new_p11 = fp11 + q11;

    P_[0] = new_p00;
    P_[1] = new_p01;
    P_[2] = new_p10;
    P_[3] = new_p11;

    // ---- Clamp diagonals ----
    P_[0] = math::clamp(P_[0], params_.min_var, params_.max_var_v);
    P_[3] = math::clamp(P_[3], params_.min_var, params_.max_var_bias);
}

void VelocityKalman::update(double dist_enc, double dist_pred, double dt_enc, double R_eff) {
    // ---- Innovation ----
    double y = dist_enc - dist_pred;

    // ---- Observation matrix H = [dt_enc, -0.5*dt_enc^2] (NEGATIVE: bias reduces v_fwd) ----
    double h0 = dt_enc;
    double h1 = -0.5 * dt_enc * dt_enc;

    // ---- Innovation covariance S = H*P*H^T + R (scalar) ----
    // H*P = [h0*p00 + h1*p10,  h0*p01 + h1*p11]
    // S = (h0*p00 + h1*p10)*h0 + (h0*p01 + h1*p11)*h1 + R
    double hp0 = h0 * P_[0] + h1 * P_[2];  // H*P col 0
    double hp1 = h0 * P_[1] + h1 * P_[3];  // H*P col 1
    double S = hp0 * h0 + hp1 * h1 + R_eff;

    if (S < 1e-20) return;  // degenerate (should not happen)

    // ---- Kalman gain K = P*H^T / S (2x1) ----
    // P*H^T = [p00*h0 + p01*h1,  p10*h0 + p11*h1]^T
    double k0 = (P_[0] * h0 + P_[1] * h1) / S;
    double k1 = (P_[2] * h0 + P_[3] * h1) / S;

    // ---- Correct nominal state ----
    v_fwd_        += k0 * y;
    accel_bias_x_ += k1 * y;

    // ---- Joseph-form covariance update ----
    // P = (I - K*H) * P * (I - K*H)^T + K * R * K^T
    //
    // I - KH = [[1 - k0*h0,  -k0*h1],
    //           [  -k1*h0, 1 - k1*h1]]
    double ikh00 = 1.0 - k0 * h0;
    double ikh01 = -k0 * h1;
    double ikh10 = -k1 * h0;
    double ikh11 = 1.0 - k1 * h1;

    // (I-KH) * P
    double tmp00 = ikh00 * P_[0] + ikh01 * P_[2];
    double tmp01 = ikh00 * P_[1] + ikh01 * P_[3];
    double tmp10 = ikh10 * P_[0] + ikh11 * P_[2];
    double tmp11 = ikh10 * P_[1] + ikh11 * P_[3];

    // (I-KH)*P * (I-KH)^T
    double new_p00 = tmp00 * ikh00 + tmp01 * ikh01;
    double new_p01 = tmp00 * ikh10 + tmp01 * ikh11;
    double new_p10 = tmp10 * ikh00 + tmp11 * ikh01;
    double new_p11 = tmp10 * ikh10 + tmp11 * ikh11;

    // + K * R * K^T
    double krk00 = k0 * R_eff * k0;
    double krk01 = k0 * R_eff * k1;
    double krk11 = k1 * R_eff * k1;

    P_[0] = new_p00 + krk00;
    P_[1] = new_p01 + krk01;
    P_[2] = new_p10 + krk01;  // symmetric: krk10 = k1*R*k0 = krk01
    P_[3] = new_p11 + krk11;

    // ---- Clamp diagonals ----
    P_[0] = math::clamp(P_[0], params_.min_var, params_.max_var_v);
    P_[3] = math::clamp(P_[3], params_.min_var, params_.max_var_bias);
}

} // namespace ms
