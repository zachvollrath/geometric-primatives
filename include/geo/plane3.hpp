#pragma once

#include "geo/line3.hpp"

#include <cmath>
#include <optional>
#include <stdexcept>

namespace geo {

// Which side of a plane a point lies on. Positive is the side the normal points toward.
enum class PlaneSide { Negative, On, Positive };

enum class LinePlaneRelation { Point, Parallel, Contained };

enum class PlanePlaneRelation { Line, Parallel, Coincident };

struct LinePlaneIntersection {
    LinePlaneRelation relation = LinePlaneRelation::Parallel;
    std::optional<Point3> point;
};

struct PlanePlaneIntersection {
    PlanePlaneRelation relation = PlanePlaneRelation::Parallel;
    std::optional<Line3> line;
};

// Plane in Hessian normal form: normal · x = offset, with a unit normal.
// offset is the signed distance from the world origin to the plane.
class Plane3 {
public:
    // Throws std::invalid_argument if normal is near zero or non-finite.
    [[nodiscard]] static Plane3 from_point_and_normal(const Point3& point, const Vec3& normal);

    // Normal is (b - a) × (c - a), following the right-hand rule.
    // Throws std::invalid_argument if the points are collinear.
    [[nodiscard]] static Plane3 from_points(const Point3& a, const Point3& b, const Point3& c);

    // Plane ax + by + cz + d = 0. The stored normal is (a, b, c) normalized,
    // and offset is the corresponding Hessian constant.
    // Throws std::invalid_argument if (a, b, c) is near zero or any coefficient is non-finite.
    [[nodiscard]] static Plane3 from_coefficients(double a, double b, double c, double d);

    [[nodiscard]] constexpr Vec3 normal() const noexcept { return normal_; }
    [[nodiscard]] constexpr double offset() const noexcept { return offset_; }

    [[nodiscard]] double signed_distance_to(const Point3& point) const noexcept;
    [[nodiscard]] double distance_to(const Point3& point) const noexcept;
    [[nodiscard]] Point3 closest_point_to(const Point3& point) const noexcept;
    [[nodiscard]] Point3 point_closest_to_origin() const noexcept;
    [[nodiscard]] bool contains(const Point3& point, double tolerance = default_tolerance) const noexcept;
    [[nodiscard]] PlaneSide side_of(const Point3& point,
                                     double tolerance = default_tolerance) const noexcept;

    // Same plane with the normal reversed. Signed distances change sign.
    [[nodiscard]] Plane3 flipped() const noexcept;

    // Same plane, ignoring normal sign.
    [[nodiscard]] bool coincides_with(const Plane3& other,
                                       double tolerance = default_tolerance) const noexcept;

    // Acute angle between the planes, in radians, in [0, pi/2].
    [[nodiscard]] double angle_to(const Plane3& other) const noexcept;

    // Coincident planes count as parallel and have distance 0.
    [[nodiscard]] bool is_parallel_to(const Plane3& other,
                                       double tolerance = default_tolerance) const noexcept;

    // Absolute distance when the planes are parallel. Empty when they are not.
    [[nodiscard]] std::optional<double> distance_between(const Plane3& other,
                                                          double tolerance = default_tolerance) const noexcept;

    friend constexpr bool operator==(const Plane3& a, const Plane3& b) noexcept {
        return a.normal_ == b.normal_ && a.offset_ == b.offset_;
    }

    friend constexpr bool operator!=(const Plane3& a, const Plane3& b) noexcept { return !(a == b); }

private:
    struct TrustedUnit {};

    Plane3(Vec3 unit_normal, double offset, TrustedUnit) noexcept;

    Vec3 normal_;
    double offset_ = 0.0;
};

inline Plane3::Plane3(Vec3 unit_normal, double offset, TrustedUnit) noexcept
    : normal_(unit_normal), offset_(offset) {}

inline Plane3 Plane3::from_point_and_normal(const Point3& point, const Vec3& normal) {
    const Vec3 unit = detail::checked_unit(normal, "Plane3 requires a non-zero normal");
    return Plane3(unit, dot(unit, point.to_vec3()), TrustedUnit{});
}

inline Plane3 Plane3::from_points(const Point3& a, const Point3& b, const Point3& c) {
    const Vec3 unit = detail::checked_unit(cross(b - a, c - a),
                                            "Plane3::from_points requires non-collinear points");
    return Plane3(unit, dot(unit, a.to_vec3()), TrustedUnit{});
}

inline Plane3 Plane3::from_coefficients(double a, double b, double c, double d) {
    const Vec3 raw{a, b, c};
    const double length = raw.length();
    if (!std::isfinite(length) || !(length > default_tolerance) || !std::isfinite(d)) {
        throw std::invalid_argument(
            "Plane3::from_coefficients requires a non-zero normal and finite coefficients");
    }
    const double inv = 1.0 / length;
    const Vec3 unit{raw.x * inv, raw.y * inv, raw.z * inv};
    return Plane3(unit, -d * inv, TrustedUnit{});
}

inline double Plane3::signed_distance_to(const Point3& point) const noexcept {
    return dot(normal_, point.to_vec3()) - offset_;
}

inline double Plane3::distance_to(const Point3& point) const noexcept {
    const double signed_distance = signed_distance_to(point);
    return signed_distance < 0.0 ? -signed_distance : signed_distance;
}

inline Point3 Plane3::closest_point_to(const Point3& point) const noexcept {
    return point - normal_ * signed_distance_to(point);
}

inline Point3 Plane3::point_closest_to_origin() const noexcept {
    return Point3::from_vec3(normal_ * offset_);
}

inline bool Plane3::contains(const Point3& point, double tolerance) const noexcept {
    return nearly_zero(signed_distance_to(point), tolerance);
}

inline PlaneSide Plane3::side_of(const Point3& point, double tolerance) const noexcept {
    const double signed_distance = signed_distance_to(point);
    if (nearly_zero(signed_distance, tolerance)) {
        return PlaneSide::On;
    }
    return signed_distance > 0.0 ? PlaneSide::Positive : PlaneSide::Negative;
}

inline Plane3 Plane3::flipped() const noexcept { return Plane3(-normal_, -offset_, TrustedUnit{}); }

inline double Plane3::angle_to(const Plane3& other) const noexcept {
    double cosine = std::fabs(dot(normal_, other.normal_));
    if (cosine > 1.0) {
        cosine = 1.0;
    }
    return std::acos(cosine);
}

inline bool Plane3::is_parallel_to(const Plane3& other, double tolerance) const noexcept {
    return !(cross(normal_, other.normal_).length() > tolerance);
}

inline std::optional<double> Plane3::distance_between(const Plane3& other, double tolerance) const noexcept {
    if (!is_parallel_to(other, tolerance)) {
        return std::nullopt;
    }
    const double alignment = dot(normal_, other.normal_);
    const double other_offset = alignment >= 0.0 ? other.offset_ : -other.offset_;
    const double delta = offset_ - other_offset;
    return delta < 0.0 ? -delta : delta;
}

inline bool Plane3::coincides_with(const Plane3& other, double tolerance) const noexcept {
    if (cross(normal_, other.normal_).length() > tolerance) {
        return false;
    }
    const double alignment = dot(normal_, other.normal_);
    const double other_offset = alignment >= 0.0 ? other.offset_ : -other.offset_;
    return nearly_equal(offset_, other_offset, tolerance);
}

// Intersection of an infinite line and a plane.
// Parallel: no point. Contained: the line lies in the plane. Point: a single intersection.
[[nodiscard]] inline LinePlaneIntersection intersect(const Line3& line,
                                                      const Plane3& plane,
                                                      double tolerance = default_tolerance) {
    const double denom = dot(plane.normal(), line.direction());
    const double signed_distance = plane.signed_distance_to(line.origin());
    if (nearly_zero(denom, tolerance)) {
        if (nearly_zero(signed_distance, tolerance)) {
            return {LinePlaneRelation::Contained, std::nullopt};
        }
        return {LinePlaneRelation::Parallel, std::nullopt};
    }
    const double t = -signed_distance / denom;
    return {LinePlaneRelation::Point, line.point_at(t)};
}

[[nodiscard]] inline LinePlaneIntersection intersect(const Plane3& plane,
                                                      const Line3& line,
                                                      double tolerance = default_tolerance) {
    return intersect(line, plane, tolerance);
}

// Intersection of two planes. The line direction is a.normal × b.normal,
// and the line origin is the point on the intersection closest to the world origin.
[[nodiscard]] inline PlanePlaneIntersection intersect(const Plane3& a,
                                                       const Plane3& b,
                                                       double tolerance = default_tolerance) {
    const Vec3 n1 = a.normal();
    const Vec3 n2 = b.normal();
    const Vec3 direction = cross(n1, n2);
    if (!(direction.length() > tolerance)) {
        const double alignment = dot(n1, n2);
        const double offset_b = alignment >= 0.0 ? b.offset() : -b.offset();
        if (nearly_equal(a.offset(), offset_b, tolerance)) {
            return {PlanePlaneRelation::Coincident, std::nullopt};
        }
        return {PlanePlaneRelation::Parallel, std::nullopt};
    }

    // Point on both planes closest to the origin. With direction = n1 × n2:
    // p = (d1 (n2 × direction) + d2 (direction × n1)) / |direction|^2.
    const Vec3 point_vec = (cross(n2, direction) * a.offset() + cross(direction, n1) * b.offset()) /
                           direction.length_squared();
    return {PlanePlaneRelation::Line, Line3{Point3::from_vec3(point_vec), direction}};
}

// Unique point where three planes meet. Empty when the planes do not determine one point.
[[nodiscard]] inline std::optional<Point3> intersect(const Plane3& a,
                                                      const Plane3& b,
                                                      const Plane3& c,
                                                      double tolerance = default_tolerance) {
    const Vec3 n1 = a.normal();
    const Vec3 n2 = b.normal();
    const Vec3 n3 = c.normal();
    const Vec3 n2xn3 = cross(n2, n3);
    const double determinant = dot(n1, n2xn3);
    if (nearly_zero(determinant, tolerance)) {
        return std::nullopt;
    }

    // n1·x = d1, n2·x = d2, n3·x = d3
    // x = (d1 (n2 × n3) + d2 (n3 × n1) + d3 (n1 × n2)) / (n1 · (n2 × n3))
    const Vec3 point_vec =
        (n2xn3 * a.offset() + cross(n3, n1) * b.offset() + cross(n1, n2) * c.offset()) / determinant;
    const Point3 point = Point3::from_vec3(point_vec);
    if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
        return std::nullopt;
    }
    if (a.distance_to(point) > tolerance || b.distance_to(point) > tolerance ||
        c.distance_to(point) > tolerance) {
        return std::nullopt;
    }
    return point;
}

}  // namespace geo
