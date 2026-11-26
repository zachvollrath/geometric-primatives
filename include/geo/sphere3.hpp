#pragma once

#include "geo/point3.hpp"

#include <cmath>
#include <stdexcept>

namespace geo {

// Sphere determined by a center and a positive radius.
class Sphere3 {
public:
    // Throws std::invalid_argument if radius is not finite and positive.
    [[nodiscard]] static Sphere3 from_center_and_radius(const Point3& center, double radius);

    // Unique sphere through four non-coplanar points.
    // Throws std::invalid_argument if the points do not determine a sphere.
    [[nodiscard]] static Sphere3 from_points(const Point3& a,
                                              const Point3& b,
                                              const Point3& c,
                                              const Point3& d);

    [[nodiscard]] constexpr Point3 center() const noexcept { return center_; }
    [[nodiscard]] constexpr double radius() const noexcept { return radius_; }

    [[nodiscard]] double volume() const noexcept;
    [[nodiscard]] double surface_area() const noexcept;

private:
    Sphere3(Point3 center, double radius) noexcept;

    Point3 center_;
    double radius_ = 0.0;
};

inline Sphere3::Sphere3(Point3 center, double radius) noexcept : center_(center), radius_(radius) {}

inline Sphere3 Sphere3::from_center_and_radius(const Point3& center, double radius) {
    if (!std::isfinite(radius) || !(radius > default_tolerance)) {
        throw std::invalid_argument("Sphere3 requires a positive finite radius");
    }
    return Sphere3{center, radius};
}

inline Sphere3 Sphere3::from_points(const Point3& a, const Point3& b, const Point3& c, const Point3& d) {
    const Vec3 v1 = b - a;
    const Vec3 v2 = c - a;
    const Vec3 v3 = d - a;
    const double scale = v1.length() + v2.length() + v3.length();
    const Vec3 v2xv3 = cross(v2, v3);
    const double determinant = dot(v1, v2xv3);
    if (!std::isfinite(determinant) || !(std::fabs(determinant) > default_tolerance * scale * scale * scale)) {
        throw std::invalid_argument("Sphere3::from_points requires four non-coplanar points");
    }

    // |O - A| = |O - B| = |O - C| = |O - D| gives O · (P - A) = 0.5 (|P|^2 - |A|^2).
    const double rhs1 = 0.5 * (b.to_vec3().length_squared() - a.to_vec3().length_squared());
    const double rhs2 = 0.5 * (c.to_vec3().length_squared() - a.to_vec3().length_squared());
    const double rhs3 = 0.5 * (d.to_vec3().length_squared() - a.to_vec3().length_squared());
    const Vec3 center_vec = (v2xv3 * rhs1 + cross(v3, v1) * rhs2 + cross(v1, v2) * rhs3) / determinant;
    if (!std::isfinite(center_vec.x) || !std::isfinite(center_vec.y) || !std::isfinite(center_vec.z)) {
        throw std::invalid_argument("Sphere3::from_points requires four non-coplanar points");
    }

    const Point3 center = Point3::from_vec3(center_vec);
    const double radius = center.distance_to(a);
    if (!std::isfinite(radius) || !(radius > default_tolerance)) {
        throw std::invalid_argument("Sphere3::from_points requires four non-coplanar points");
    }
    return Sphere3{center, radius};
}

inline double Sphere3::volume() const noexcept {
    return (4.0 / 3.0) * pi * radius_ * radius_ * radius_;
}

inline double Sphere3::surface_area() const noexcept { return 4.0 * pi * radius_ * radius_; }

}  // namespace geo
