#pragma once

#include "geo/point3.hpp"

#include <optional>

namespace geo {

// Infinite line in parametric form: point(t) = origin + t * direction.
// direction is a unit vector, so t is signed distance from origin along the line.
class Line3 {
public:
    // Normalizes direction. Throws std::invalid_argument if it is near zero or non-finite.
    Line3(Point3 origin, Vec3 direction);

    // Line through two distinct points, directed from a toward b.
    // Throws std::invalid_argument if the points coincide.
    [[nodiscard]] static Line3 through(const Point3& a, const Point3& b);

    [[nodiscard]] constexpr Point3 origin() const noexcept { return origin_; }
    [[nodiscard]] constexpr Vec3 direction() const noexcept { return direction_; }

    [[nodiscard]] Point3 point_at(double t) const noexcept;
    // Parameter t of the closest point on the line to `point`.
    [[nodiscard]] double parameter_of(const Point3& point) const noexcept;
    [[nodiscard]] Point3 closest_point_to(const Point3& point) const noexcept;
    [[nodiscard]] double distance_to(const Point3& point) const noexcept;
    [[nodiscard]] bool contains(const Point3& point, double tolerance = default_tolerance) const noexcept;

    [[nodiscard]] Line3 reversed() const noexcept;

    // Same infinite line, ignoring origin placement and direction sign.
    [[nodiscard]] bool coincides_with(const Line3& other,
                                       double tolerance = default_tolerance) const noexcept;

    friend constexpr bool operator==(const Line3& a, const Line3& b) noexcept {
        return a.origin_ == b.origin_ && a.direction_ == b.direction_;
    }

    friend constexpr bool operator!=(const Line3& a, const Line3& b) noexcept { return !(a == b); }

private:
    struct TrustedUnit {};

    Line3(Point3 origin, Vec3 unit_direction, TrustedUnit) noexcept;

    Point3 origin_;
    Vec3 direction_;
};

inline Line3::Line3(Point3 origin, Vec3 unit_direction, TrustedUnit) noexcept
    : origin_(origin), direction_(unit_direction) {}

inline Line3::Line3(Point3 origin, Vec3 direction)
    : Line3(origin,
            detail::checked_unit(direction, "Line3 requires a non-zero direction"),
            TrustedUnit{}) {}

inline Line3 Line3::through(const Point3& a, const Point3& b) {
    return Line3(a,
                 detail::checked_unit(b - a, "Line3::through requires two distinct points"),
                 TrustedUnit{});
}

inline Point3 Line3::point_at(double t) const noexcept { return origin_ + direction_ * t; }

inline double Line3::parameter_of(const Point3& point) const noexcept {
    return dot(point - origin_, direction_);
}

inline Point3 Line3::closest_point_to(const Point3& point) const noexcept {
    return point_at(parameter_of(point));
}

inline double Line3::distance_to(const Point3& point) const noexcept {
    return cross(point - origin_, direction_).length();
}

inline bool Line3::contains(const Point3& point, double tolerance) const noexcept {
    return distance_to(point) <= tolerance;
}

inline Line3 Line3::reversed() const noexcept { return Line3(origin_, -direction_, TrustedUnit{}); }

inline bool Line3::coincides_with(const Line3& other, double tolerance) const noexcept {
    if (cross(direction_, other.direction_).length() > tolerance) {
        return false;
    }
    return distance_to(other.origin_) <= tolerance;
}

enum class LineLineRelation { Point, Parallel, Coincident, Skew };

struct LineLineIntersection {
    LineLineRelation relation = LineLineRelation::Skew;
    std::optional<Point3> point;
};

// Intersection of two infinite lines. Skew lines in 3D do not meet.
[[nodiscard]] inline LineLineIntersection intersect(const Line3& a,
                                                      const Line3& b,
                                                      double tolerance = default_tolerance) {
    const Vec3 direction_a = a.direction();
    const Vec3 direction_b = b.direction();
    const double cosine = dot(direction_a, direction_b);
    const double sine_squared = 1.0 - cosine * cosine;
    const Vec3 origin_delta = a.origin() - b.origin();
    if (!(sine_squared > tolerance * tolerance)) {
        if (cross(origin_delta, direction_b).length() <= tolerance) {
            return {LineLineRelation::Coincident, std::nullopt};
        }
        return {LineLineRelation::Parallel, std::nullopt};
    }

    // a.origin + s * direction_a = b.origin + t * direction_b.
    const double along_a = dot(origin_delta, direction_a);
    const double along_b = dot(origin_delta, direction_b);
    const double s = (cosine * along_b - along_a) / sine_squared;
    const double t = cosine * s + along_b;
    const Point3 on_a = a.point_at(s);
    const Point3 on_b = b.point_at(t);
    if (on_a.distance_to(on_b) <= tolerance) {
        return {LineLineRelation::Point, Point3::midpoint(on_a, on_b)};
    }
    return {LineLineRelation::Skew, std::nullopt};
}

}  // namespace geo
