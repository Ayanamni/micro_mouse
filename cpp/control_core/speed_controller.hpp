#pragma once
/// Speed PI controller.
/// Converts speed error into common-mode throttle command.

namespace ms {

struct SpeedControllerGains {
    double Kp = 0.15;      // proportional (m/s error → normalized command)
    double Ki = 0.05;      // integral
    double max_throttle = 1.0;
};

class SpeedController {
public:
    explicit SpeedController(const SpeedControllerGains& gains = {});

    /// Compute common-mode throttle from speed error.
    /// @param current_speed  m/s, forward velocity
    /// @param target_speed   m/s, desired forward velocity
    /// @param dt             seconds
    /// @returns              normalized throttle [-1, 1]
    double update(double current_speed, double target_speed, double dt);

    void reset();
    void set_target(double target_speed) { target_ = target_speed; }

private:
    SpeedControllerGains g_;
    double integral_ = 0.0;
    double target_   = 0.0;
};

} // namespace ms
