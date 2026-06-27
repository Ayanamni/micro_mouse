#include "speed_controller.hpp"
#include "shared/math_utils.hpp"

namespace ms {

SpeedController::SpeedController(const SpeedControllerGains& gains)
    : g_(gains) {}

double SpeedController::update(double current_speed, double target_speed, double dt) {
    target_ = target_speed;
    if (dt <= 0) return 0.0;

    double error = target_speed - current_speed;

    // Integral
    integral_ += error * dt;
    double i_limit = g_.max_throttle / std::max(g_.Ki, 1e-9);
    integral_ = math::clamp(integral_, -i_limit, i_limit);

    double throttle = g_.Kp * error + g_.Ki * integral_;
    throttle = math::clamp(throttle, -g_.max_throttle, g_.max_throttle);

    return throttle;
}

void SpeedController::reset() {
    integral_ = 0.0;
    target_   = 0.0;
}

} // namespace ms
