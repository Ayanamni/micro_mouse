#pragma once
/// Lightweight math utilities for micromouse simulation.
/// Avoids heavy library dependencies — just what's needed for SO(2) and filtering.

#include <cmath>
#include <algorithm>
#include <numbers>

namespace ms::math {

constexpr double PI = std::numbers::pi;
constexpr double TWO_PI = 2.0 * PI;

/// Wrap angle to [-π, π]
inline double wrap_angle(double a) {
    a = std::fmod(a + PI, TWO_PI);
    if (a < 0) a += TWO_PI;
    return a - PI;
}

/// 2D rotation matrix (cos, sin cached)
inline void rotate_2d(double x, double y, double cos_yaw, double sin_yaw,
                      double& rx, double& ry) {
    rx = x * cos_yaw - y * sin_yaw;
    ry = x * sin_yaw + y * cos_yaw;
}

/// Simple first-order low-pass filter
struct LowPass1 {
    double alpha;   // smoothing factor [0, 1], 0 = no filtering
    double value;

    explicit LowPass1(double cutoff_hz, double dt)
        : alpha(1.0 - std::exp(-TWO_PI * cutoff_hz * dt)), value(0.0) {}

    double update(double raw) {
        value += alpha * (raw - value);
        return value;
    }

    void reset(double v = 0.0) { value = v; }
};

/// Clamp value to [lo, hi]
template <typename T>
inline T clamp(T v, T lo, T hi) {
    return std::min(std::max(v, lo), hi);
}

} // namespace ms::math
