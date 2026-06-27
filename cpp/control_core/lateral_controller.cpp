#include "lateral_controller.hpp"
#include "shared/math_utils.hpp"

namespace ms {

LateralController::LateralController(const LateralControllerGains& gains)
    : g_(gains) {}

double LateralController::update(double lateral_error, double curvature, double dt) {
    if (dt <= 0) return 0.0;

    // Derivative
    double derivative = (lateral_error - prev_error_) / dt;
    prev_error_ = lateral_error;

    // Integral (with basic anti-windup via clamping)
    integral_ += lateral_error * dt;
    integral_ = math::clamp(integral_, -0.5 / std::max(g_.Ki, 1e-9), 0.5 / std::max(g_.Ki, 1e-9));

    // PID
    double steer = g_.Kp * lateral_error
                 + g_.Kd * derivative
                 + g_.Ki * integral_;

    // Curvature feedforward
    steer += g_.Kff_curvature * curvature;

    // Clamp
    steer = math::clamp(steer, -g_.max_steer, g_.max_steer);

    return steer;
}

void LateralController::reset() {
    prev_error_ = 0.0;
    integral_   = 0.0;
}

} // namespace ms
