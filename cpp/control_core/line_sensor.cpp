#include "line_sensor.hpp"
#include <cmath>
#include <algorithm>

namespace ms {

LineSensor::LineSensor(const LineSensorConfig& cfg)
    : cfg_(cfg)
    , sensor_spacing_(cfg.total_width / (cfg.n_sensors - 1))
{}

LineSensorResult LineSensor::process(
    const std::array<double, LINE_SENSOR_COUNT>& lateral_dist)
{
    LineSensorResult result{};
    result.line_visible = false;

    // Wider sigma for robust detection: line_half_width = 15mm
    double sigma = 0.015;  // m
    double sigma_sq = 2.0 * sigma * sigma;
    double weighted_sum = 0.0;
    double weight_total = 0.0;

    // Sensor lateral positions relative to vehicle center
    double half_width = cfg_.total_width * 0.5;

    // Track min/max sensor Y positions that see the line (for extrapolation)
    double min_y_seen = half_width;
    double max_y_seen = -half_width;

    for (int i = 0; i < cfg_.n_sensors; ++i) {
        double sensor_y = i * sensor_spacing_ - half_width;
        double e = lateral_dist[i];  // lateral distance to centerline at this sensor

        // Gaussian intensity model: I = exp(-e²/(2σ²))
        double intensity = std::exp(-e * e / sigma_sq);

        // Quantize to 16-bit ADC
        uint16_t adc_val = static_cast<uint16_t>(
            std::clamp(intensity * 65535.0, 0.0, 65535.0));

        result.adc[i] = adc_val;

        // Lower threshold for line detection (with wider sigma)
        if (intensity > 0.05) {
            result.line_visible = true;
            weighted_sum += intensity * sensor_y;
            weight_total += intensity;
            if (sensor_y < min_y_seen) min_y_seen = sensor_y;
            if (sensor_y > max_y_seen) max_y_seen = sensor_y;
        }
    }

    // Weighted centroid → lateral error
    // lateral_error > 0 = centerline is to the left of sensor array center
    if (weight_total > 1e-9) {
        result.lateral_error = weighted_sum / weight_total;
    } else if (result.line_visible) {
        // Fallback: should not happen with threshold check, but keep 0
        result.lateral_error = 0.0;
    } else {
        // LINE LOST: extrapolate from which edge had more intensity
        // Find the sensor with maximum intensity as a hint
        double max_intensity = 0.0;
        double max_sensor_y = 0.0;
        for (int i = 0; i < cfg_.n_sensors; ++i) {
            double sensor_y = i * sensor_spacing_ - half_width;
            double e = lateral_dist[i];
            double intensity = std::exp(-e * e / sigma_sq);
            if (intensity > max_intensity) {
                max_intensity = intensity;
                max_sensor_y = sensor_y;
            }
        }
        // Steer toward the brightest sensor (saturate at array edge)
        double sign = (max_sensor_y > 0) ? 1.0 : -1.0;
        result.lateral_error = sign * half_width * 1.2;  // 20% beyond array edge
    }

    return result;
}

LineSensorResult LineSensor::from_lateral_error(double lateral_error_m) {
    LineSensorResult result{};
    result.lateral_error = lateral_error_m;
    result.line_visible = true;
    // Fill ADC with synthetic values centered on the error
    double half_width = 0.130 * 0.5;
    double spacing = 0.130 / 15.0;
    double sigma_sq = 2.0 * 0.010 * 0.010;
    for (int i = 0; i < LINE_SENSOR_COUNT; ++i) {
        double sensor_y = i * spacing - half_width;
        double e = sensor_y - lateral_error_m;
        double intensity = std::exp(-e * e / sigma_sq);
        result.adc[i] = static_cast<uint16_t>(
            std::clamp(intensity * 65535.0, 0.0, 65535.0));
    }
    return result;
}

} // namespace ms
