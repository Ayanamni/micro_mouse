#pragma once
/// Lateral (steering) PID controller with curvature feedforward.
/// Converts lateral error + curvature into differential torque command.

namespace ms {

struct LateralControllerGains {
    double Kp = 30.0;       // proportional to lateral error → differential
    double Kd = 1.0;       // derivative (rate of lateral error change)
    double Ki = 0.5;       // integral (steady-state correction)
    double Kff_curvature = 0.06; // feedforward: curvature → differential
    double max_steer = 10.0;      // max differential command (full authority for sharp turns)
};

class LateralController {
public:
    explicit LateralController(const LateralControllerGains& gains = {});

    /// Compute differential command from lateral error.
    /// @param lateral_error  m, positive = left of track center
    /// @param curvature      1/m, track curvature at current position
    /// @param dt             seconds since last call
    /// @returns              differential command [-1, 1] to ADD to left,
    ///                       positive = steer right (left wheel faster)
    double update(double lateral_error, double curvature, double dt);

    void reset();

private:
    LateralControllerGains g_;
    double prev_error_ = 0.0;
    double integral_   = 0.0;
};

} // namespace ms
