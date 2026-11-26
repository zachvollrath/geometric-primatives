#pragma once

#include "geo/constants.hpp"

#include <cmath>
#include <stdexcept>

namespace geo {

// Free vector in R^3. Directions and normals are Vec3; positions are Point3.
struct Vec3 {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;

    constexpr Vec3() noexcept = default;
    constexpr Vec3(double x_value, double y_value, double z_value) noexcept
        : x(x_value), y(y_value), z(z_value) {}

    [[nodiscard]] static constexpr Vec3 zero() noexcept { return {}; }
    [[nodiscard]] static constexpr Vec3 unit_x() noexcept { return {1.0, 0.0, 0.0}; }
    [[nodiscard]] static constexpr Vec3 unit_y() noexcept { return {0.0, 1.0, 0.0}; }
    [[nodiscard]] static constexpr Vec3 unit_z() noexcept { return {0.0, 0.0, 1.0}; }

    [[nodiscard]] constexpr double length_squared() const noexcept {
        return x * x + y * y + z * z;
    }

    [[nodiscard]] double length() const noexcept { return std::hypot(x, y, z); }

    [[nodiscard]] constexpr Vec3 operator+(const Vec3& other) const noexcept {
        return {x + other.x, y + other.y, z + other.z};
    }

    [[nodiscard]] constexpr Vec3 operator-(const Vec3& other) const noexcept {
        return {x - other.x, y - other.y, z - other.z};
    }

    [[nodiscard]] constexpr Vec3 operator-() const noexcept { return {-x, -y, -z}; }

    [[nodiscard]] constexpr Vec3 operator*(double scalar) const noexcept {
        return {x * scalar, y * scalar, z * scalar};
    }

    [[nodiscard]] Vec3 operator/(double scalar) const {
        if (!std::isfinite(scalar) || scalar == 0.0) {
            throw std::invalid_argument("Vec3 division requires a finite, non-zero scalar");
        }
        const double inv = 1.0 / scalar;
        return {x * inv, y * inv, z * inv};
    }

    constexpr Vec3& operator+=(const Vec3& other) noexcept {
        x += other.x;
        y += other.y;
        z += other.z;
        return *this;
    }

    constexpr Vec3& operator-=(const Vec3& other) noexcept {
        x -= other.x;
        y -= other.y;
        z -= other.z;
        return *this;
    }

    constexpr Vec3& operator*=(double scalar) noexcept {
        x *= scalar;
        y *= scalar;
        z *= scalar;
        return *this;
    }

    Vec3& operator/=(double scalar) {
        *this = *this / scalar;
        return *this;
    }

    friend constexpr Vec3 operator*(double scalar, const Vec3& v) noexcept { return v * scalar; }

    friend constexpr bool operator==(const Vec3& a, const Vec3& b) noexcept {
        return a.x == b.x && a.y == b.y && a.z == b.z;
    }

    friend constexpr bool operator!=(const Vec3& a, const Vec3& b) noexcept { return !(a == b); }
};

[[nodiscard]] constexpr double dot(const Vec3& a, const Vec3& b) noexcept {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

[[nodiscard]] constexpr Vec3 cross(const Vec3& a, const Vec3& b) noexcept {
    return {
        a.y * b.z - a.z * b.y,
        a.z * b.x - a.x * b.z,
        a.x * b.y - a.y * b.x,
    };
}

[[nodiscard]] constexpr bool nearly_equal(const Vec3& a,
                                           const Vec3& b,
                                           double tolerance = default_tolerance) noexcept {
    return nearly_equal(a.x, b.x, tolerance) && nearly_equal(a.y, b.y, tolerance) &&
           nearly_equal(a.z, b.z, tolerance);
}

[[nodiscard]] constexpr bool nearly_zero(const Vec3& v, double tolerance = default_tolerance) noexcept {
    return v.length_squared() <= tolerance * tolerance;
}

namespace detail {

[[nodiscard]] inline Vec3 checked_unit(const Vec3& v, const char* message) {
    const double length = v.length();
    if (!std::isfinite(length) || !(length > default_tolerance)) {
        throw std::invalid_argument(message);
    }
    const double inv = 1.0 / length;
    return {v.x * inv, v.y * inv, v.z * inv};
}

}  // namespace detail

// Returns v scaled to unit length. Throws std::invalid_argument if v is near zero or non-finite.
[[nodiscard]] inline Vec3 normalized(const Vec3& v) {
    return detail::checked_unit(v, "normalized() requires a non-zero vector");
}

}  // namespace geo
