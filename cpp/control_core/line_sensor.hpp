#pragma once
/// 16-channel photoelectric line sensor array.
/// Projects virtual sensor positions onto the track centerline spline
/// and computes lateral error via intensity-weighted centroid.

#include <array>
#include <cstdint>
#include "shared/types.hpp"

namespace ms {

struct LineSensorConfig {
    int    n_sensors       = 16;
    double total_width     = 0.130;   // m
    double lookahead       = 0.050;   // m, forward from axle
    double line_half_width = 0.010;   // m, white line half-width (≈10mm)
    double noise_std       = 0.0;     // normalized [0,1] — user said no noise
};

struct LineSensorResult {
    LineAdc adc;             // raw ADC readings [0, 65535]
    double lateral_error;    // m, positive = left of center
    bool   line_visible;     // true if at least one sensor sees the line
};

class LineSensor {
public:
    explicit LineSensor(const LineSensorConfig& cfg = {});

    /// Process sensor readings. Lateral error is computed externally
    /// (from the Python-side TrackCenterline spline), passed in as
    /// an array of 16 lateral distances.
    /// @param lateral_dist  Lateral distance of each sensor from track center (m)
    /// @returns             ADC readings + weighted-centroid lateral error
    LineSensorResult process(const std::array<double, LINE_SENSOR_COUNT>& lateral_dist);

    /// Direct lateral error from external computation (TrackCenterline).
    /// This is a convenience for when the Python side already has the error.
    static LineSensorResult from_lateral_error(double lateral_error_m);

private:
    LineSensorConfig cfg_;
    double sensor_spacing_; // m, spacing between adjacent sensors
};

} // namespace ms
