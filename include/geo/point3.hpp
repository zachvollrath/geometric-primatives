#pragma once

#include "geo/vec3.hpp"

namespace geo {

// Position in R^3. The difference of two points is a Vec3.
struct Point3 {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;

    constexpr Point3() noexcept = default;
    constexpr Point3(double x_value, double y_value, double z_value) noexcept
        : x(x_value), y(y_value), z(z_value) {}

    [[nodiscard]] static constexpr Point3 origin() noexcept { return {}; }

    [[nodiscard]] static constexpr Point3 from_vec3(const Vec3& position) noexcept {
        return {position.x, position.y, position.z};
    }

    // Position vector from the world origin to this point.
    [[nodiscard]] constexpr Vec3 to_vec3() const noexcept { return {x, y, z}; }

    [[nodiscard]] constexpr Point3 operator+(const Vec3& displacement) const noexcept {
        return {x + displacement.x, y + displacement.y, z + displacement.z};
    }

    [[nodiscard]] constexpr Point3 operator-(const Vec3& displacement) const noexcept {
        return {x - displacement.x, y - displacement.y, z - displacement.z};
    }

    [[nodiscard]] constexpr Vec3 operator-(const Point3& other) const noexcept {
        return {x - other.x, y - other.y, z - other.z};
    }

    constexpr Point3& operator+=(const Vec3& displacement) noexcept {
        x += displacement.x;
        y += displacement.y;
        z += displacement.z;
        return *this;
    }

    constexpr Point3& operator-=(const Vec3& displacement) noexcept {
        x -= displacement.x;
        y -= displacement.y;
        z -= displacement.z;
        return *this;
    }

    [[nodiscard]] constexpr double distance_squared_to(const Point3& other) const noexcept {
        return (*this - other).length_squared();
    }

    [[nodiscard]] double distance_to(const Point3& other) const noexcept {
        return (*this - other).length();
    }

    // Unit vector from this point toward `other`.
    // Throws std::invalid_argument if the points coincide.
    [[nodiscard]] Vec3 direction_to(const Point3& other) const {
        return detail::checked_unit(other - *this, "Point3::direction_to requires two distinct points");
    }

    [[nodiscard]] static constexpr Point3 lerp(const Point3& a, const Point3& b, double t) noexcept {
        return a + (b - a) * t;
    }

    [[nodiscard]] static constexpr Point3 midpoint(const Point3& a, const Point3& b) noexcept {
        return lerp(a, b, 0.5);
    }

    friend constexpr bool operator==(const Point3& a, const Point3& b) noexcept {
        return a.x == b.x && a.y == b.y && a.z == b.z;
    }

    friend constexpr bool operator!=(const Point3& a, const Point3& b) noexcept { return !(a == b); }
};

[[nodiscard]] constexpr bool nearly_equal(const Point3& a,
                                           const Point3& b,
                                           double tolerance = default_tolerance) noexcept {
    return nearly_equal(a.to_vec3(), b.to_vec3(), tolerance);
}

}  // namespace geo
