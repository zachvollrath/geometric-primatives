#pragma once

namespace geo {

// Default absolute tolerance for predicates (parallelism, containment, coincidence).
// Pass a larger tolerance when coordinates are far from the origin.
inline constexpr double default_tolerance = 1e-9;

inline constexpr double pi = 3.14159265358979323846;

[[nodiscard]] constexpr bool nearly_zero(double value, double tolerance = default_tolerance) noexcept {
    const double magnitude = value < 0.0 ? -value : value;
    return magnitude <= tolerance;
}

[[nodiscard]] constexpr bool nearly_equal(double a, double b, double tolerance = default_tolerance) noexcept {
    return nearly_zero(a - b, tolerance);
}

}  // namespace geo
