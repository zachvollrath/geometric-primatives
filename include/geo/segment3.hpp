#pragma once

#include "geo/point3.hpp"

#include <optional>
#include <utility>

namespace geo {

enum class SegmentIntersectionKind { None, Point, Overlap };

struct SegmentPointProximity {
    Point3 point;
    double distance = 0.0;
    double parameter = 0.0;
};

struct SegmentSegmentProximity {
    Point3 point_on_first;
    Point3 point_on_second;
    double distance = 0.0;
};

struct SegmentIntersection {
    SegmentIntersectionKind kind = SegmentIntersectionKind::None;
    std::optional<Point3> point;
    // Endpoints of the shared portion when kind is Overlap.
    Point3 overlap_start;
    Point3 overlap_end;
};

// Finite segment from start to end. Unlike Line3, a segment has a length.
class Segment3 {
public:
    Segment3(Point3 start, Point3 end) noexcept;

    [[nodiscard]] constexpr Point3 start() const noexcept { return start_; }
    [[nodiscard]] constexpr Point3 end() const noexcept { return end_; }

    [[nodiscard]] double length() const noexcept;
    // Unit direction from start toward end. Throws std::invalid_argument if the segment has no length.
    [[nodiscard]] Vec3 direction() const;

    // Point at parameter t, where 0 is start and 1 is end.
    [[nodiscard]] Point3 point_at(double t) const noexcept;

    [[nodiscard]] SegmentPointProximity closest_to(const Point3& point) const noexcept;
    [[nodiscard]] SegmentSegmentProximity closest_points_to(const Segment3& other) const noexcept;

    [[nodiscard]] SegmentIntersection intersect(const Segment3& other,
                                                 double tolerance = default_tolerance) const;

    // Direction tests. A zero-length segment is neither parallel nor perpendicular to anything.
    [[nodiscard]] bool is_parallel_to(const Segment3& other,
                                       double tolerance = default_tolerance) const noexcept;
    [[nodiscard]] bool is_perpendicular_to(const Segment3& other,
                                            double tolerance = default_tolerance) const noexcept;

private:
    Point3 start_;
    Point3 end_;
};

inline Segment3::Segment3(Point3 start, Point3 end) noexcept : start_(start), end_(end) {}

inline double Segment3::length() const noexcept { return start_.distance_to(end_); }

inline Vec3 Segment3::direction() const {
    return detail::checked_unit(end_ - start_, "Segment3::direction requires a segment with length");
}

inline Point3 Segment3::point_at(double t) const noexcept { return start_ + (end_ - start_) * t; }

inline SegmentPointProximity Segment3::closest_to(const Point3& point) const noexcept {
    const Vec3 span = end_ - start_;
    const double span_length_squared = span.length_squared();
    if (!(span_length_squared > default_tolerance * default_tolerance)) {
        return {start_, point.distance_to(start_), 0.0};
    }
    double parameter = dot(point - start_, span) / span_length_squared;
    if (parameter < 0.0) {
        parameter = 0.0;
    } else if (parameter > 1.0) {
        parameter = 1.0;
    }
    const Point3 closest = point_at(parameter);
    return {closest, point.distance_to(closest), parameter};
}

inline SegmentSegmentProximity Segment3::closest_points_to(const Segment3& other) const noexcept {
    const Vec3 u = end_ - start_;
    const Vec3 v = other.end_ - other.start_;
    const Vec3 origin_delta = start_ - other.start_;
    const double a = dot(u, u);
    const double e = dot(v, v);
    const double c = dot(u, v);
    const double b = dot(origin_delta, u);
    const double f = dot(origin_delta, v);
    const double degeneracy = default_tolerance * default_tolerance;

    auto at = [&](double s, double t) {
        const Point3 on_first = start_ + u * s;
        const Point3 on_second = other.start_ + v * t;
        return SegmentSegmentProximity{on_first, on_second, on_first.distance_to(on_second)};
    };
    auto clamp01 = [](double value) {
        if (value < 0.0) {
            return 0.0;
        }
        if (value > 1.0) {
            return 1.0;
        }
        return value;
    };

    if (!(a > degeneracy) && !(e > degeneracy)) {
        return at(0.0, 0.0);
    }
    if (!(a > degeneracy)) {
        return at(0.0, clamp01(f / e));
    }
    if (!(e > degeneracy)) {
        return at(clamp01(-b / a), 0.0);
    }

    const double denominator = a * e - c * c;
    if (denominator > degeneracy * a * e) {
        const double s = (c * f - b * e) / denominator;
        const double t = (c * s + f) / e;
        if (s >= 0.0 && s <= 1.0 && t >= 0.0 && t <= 1.0) {
            return at(s, t);
        }
    }

    const SegmentSegmentProximity edges[] = {
        at(0.0, clamp01(f / e)),
        at(1.0, clamp01((c + f) / e)),
        at(clamp01(-b / a), 0.0),
        at(clamp01((c - b) / a), 1.0),
    };
    const SegmentSegmentProximity* best = &edges[0];
    for (const SegmentSegmentProximity& candidate : edges) {
        if (candidate.distance < best->distance) {
            best = &candidate;
        }
    }
    return *best;
}

inline bool Segment3::is_parallel_to(const Segment3& other, double tolerance) const noexcept {
    const Vec3 u = end_ - start_;
    const Vec3 v = other.end_ - other.start_;
    const double scale = u.length() * v.length();
    if (!(scale > tolerance)) {
        return false;
    }
    return cross(u, v).length() <= tolerance * scale;
}

inline bool Segment3::is_perpendicular_to(const Segment3& other, double tolerance) const noexcept {
    const Vec3 u = end_ - start_;
    const Vec3 v = other.end_ - other.start_;
    const double scale = u.length() * v.length();
    if (!(scale > tolerance)) {
        return false;
    }
    return nearly_zero(dot(u, v), tolerance * scale);
}

inline SegmentIntersection Segment3::intersect(const Segment3& other, double tolerance) const {
    const SegmentSegmentProximity closest = closest_points_to(other);
    if (closest.distance > tolerance) {
        return {};
    }

    const bool first_has_length = length() > tolerance;
    const bool second_has_length = other.length() > tolerance;
    if (!first_has_length || !second_has_length || !is_parallel_to(other, tolerance)) {
        return {SegmentIntersectionKind::Point,
                Point3::midpoint(closest.point_on_first, closest.point_on_second),
                {},
                {}};
    }

    const Vec3 span = end_ - start_;
    const double span_length_squared = span.length_squared();
    const auto project = [&](const Point3& point) {
        return dot(point - start_, span) / span_length_squared;
    };
    double t0 = project(other.start_);
    double t1 = project(other.end_);
    if (t0 > t1) {
        std::swap(t0, t1);
    }
    const double lo = t0 > 0.0 ? t0 : 0.0;
    const double hi = t1 < 1.0 ? t1 : 1.0;
    const double parameter_tolerance = tolerance / length();
    if (lo > hi + parameter_tolerance) {
        return {};
    }
    if (hi - lo <= parameter_tolerance) {
        double parameter = 0.5 * (lo + hi);
        if (parameter < 0.0) {
            parameter = 0.0;
        } else if (parameter > 1.0) {
            parameter = 1.0;
        }
        return {SegmentIntersectionKind::Point, point_at(parameter), {}, {}};
    }
    return {SegmentIntersectionKind::Overlap, std::nullopt, point_at(lo), point_at(hi)};
}

}  // namespace geo
