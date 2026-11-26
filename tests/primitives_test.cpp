#include "geo/geo.hpp"

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <stdexcept>
#include <utility>

namespace {

int g_failed = 0;

void check(bool condition, const char* expression, const char* file, int line) {
    if (!condition) {
        std::cerr << file << ":" << line << " failed: " << expression << "\n";
        ++g_failed;
    }
}

void check_near(double actual, double expected, const char* file, int line) {
    if (!geo::nearly_equal(actual, expected)) {
        std::cerr << file << ":" << line << " failed: " << actual << " != " << expected << "\n";
        ++g_failed;
    }
}

template <class Fn>
void expect_invalid_argument(Fn&& fn, const char* file, int line) {
    try {
        std::forward<Fn>(fn)();
        std::cerr << file << ":" << line << " failed: expected std::invalid_argument\n";
        ++g_failed;
    } catch (const std::invalid_argument&) {
        // expected
    } catch (const std::exception& ex) {
        std::cerr << file << ":" << line << " failed: wrong exception: " << ex.what() << "\n";
        ++g_failed;
    }
}

void expect_point(const geo::Point3& point, double x, double y, double z, const char* file, int line) {
    check_near(point.x, x, file, line);
    check_near(point.y, y, file, line);
    check_near(point.z, z, file, line);
}

#define CHECK(...) ::check(static_cast<bool>(__VA_ARGS__), #__VA_ARGS__, __FILE__, __LINE__)
#define CHECK_NEAR(actual, expected) ::check_near((actual), (expected), __FILE__, __LINE__)
#define EXPECT_POINT(point, x, y, z) ::expect_point((point), (x), (y), (z), __FILE__, __LINE__)
#define EXPECT_INVALID(...) \
    ::expect_invalid_argument([&] { (void)(__VA_ARGS__); }, __FILE__, __LINE__)

void test_vec3() {
    using geo::Vec3;

    const Vec3 a{1.0, 2.0, 3.0};
    const Vec3 b{4.0, 5.0, 6.0};
    CHECK((a + b) == Vec3{5.0, 7.0, 9.0});
    CHECK((b - a) == Vec3{3.0, 3.0, 3.0});
    CHECK((-a) == Vec3{-1.0, -2.0, -3.0});
    CHECK((a * 2.0) == Vec3{2.0, 4.0, 6.0});
    CHECK((2.0 * a) == Vec3{2.0, 4.0, 6.0});
    CHECK((a / 2.0) == Vec3{0.5, 1.0, 1.5});
    CHECK(a != b);
    CHECK(Vec3::zero() == Vec3{});
    CHECK(Vec3::unit_x() == Vec3{1.0, 0.0, 0.0});

    Vec3 v{1.0, 2.0, 3.0};
    v += Vec3{1.0, 1.0, 1.0};
    CHECK(v == Vec3{2.0, 3.0, 4.0});
    v -= Vec3{2.0, 3.0, 4.0};
    CHECK(v == Vec3{});
    v = Vec3{2.0, 4.0, 6.0};
    v *= 0.5;
    CHECK(v == Vec3{1.0, 2.0, 3.0});
    v /= 0.5;
    CHECK(v == Vec3{2.0, 4.0, 6.0});

    CHECK(geo::dot(Vec3::unit_x(), Vec3::unit_y()) == 0.0);
    CHECK(geo::dot(Vec3{1.0, 2.0, 3.0}, Vec3{4.0, -5.0, 6.0}) == 12.0);
    CHECK(geo::cross(Vec3::unit_x(), Vec3::unit_y()) == Vec3::unit_z());
    CHECK(Vec3{3.0, 4.0, 0.0}.length_squared() == 25.0);
    CHECK_NEAR((Vec3{3.0, 4.0, 0.0}.length()), 5.0);
    CHECK_NEAR((Vec3{1.0, 2.0, 2.0}.length()), 3.0);

    CHECK(geo::normalized(Vec3{0.0, 3.0, 0.0}) == Vec3::unit_y());
    CHECK(geo::normalized(Vec3{-2.0, 0.0, 0.0}) == -Vec3::unit_x());
    CHECK_NEAR(geo::normalized(Vec3{1.0, 2.0, 2.0}).length(), 1.0);
    CHECK(geo::nearly_equal(Vec3{1.0, 0.0, 0.0}, Vec3{1.0, 1e-12, 0.0}));
    CHECK(geo::nearly_zero(Vec3{1e-12, -1e-12, 0.0}));

    EXPECT_INVALID(geo::normalized(Vec3::zero()));
    EXPECT_INVALID(Vec3{1.0, 2.0, 3.0} / 0.0);
}

void test_point3() {
    using geo::Point3;
    using geo::Vec3;

    const Point3 p{1.0, 2.0, 3.0};
    CHECK((p + Vec3{1.0, 0.0, -1.0}) == Point3{2.0, 2.0, 2.0});
    CHECK((p - Vec3{1.0, 2.0, 3.0}) == Point3::origin());
    CHECK((p - Point3{1.0, 1.0, 1.0}) == Vec3{0.0, 1.0, 2.0});
    CHECK(p.to_vec3() == Vec3{1.0, 2.0, 3.0});
    CHECK(Point3::from_vec3(Vec3{1.0, 2.0, 3.0}) == p);
    CHECK(p != Point3::origin());
    CHECK_NEAR(p.distance_to(p), 0.0);
    CHECK(p.distance_squared_to(Point3{4.0, 6.0, 3.0}) == 25.0);
    CHECK_NEAR(p.distance_to(Point3{4.0, 6.0, 3.0}), 5.0);
    CHECK(Point3::lerp(Point3::origin(), Point3{2.0, 0.0, 0.0}, 0.25) == Point3{0.5, 0.0, 0.0});
    CHECK(Point3::midpoint(Point3::origin(), Point3{2.0, 4.0, 6.0}) == Point3{1.0, 2.0, 3.0});
    CHECK(geo::nearly_equal(p, Point3{1.0, 2.0, 3.0 + 1e-12}));

    Point3 q{1.0, 1.0, 1.0};
    q += Vec3{1.0, 2.0, 3.0};
    CHECK(q == Point3{2.0, 3.0, 4.0});
    q -= Vec3{2.0, 3.0, 4.0};
    CHECK(q == Point3::origin());
}

void test_line3() {
    using geo::Line3;
    using geo::Point3;
    using geo::Vec3;

    const Point3 start{0.0, 0.0, 0.0};
    const Point3 end{0.0, 0.0, 5.0};
    const Line3 line = Line3::through(start, end);
    CHECK(line.origin() == start);
    CHECK(geo::nearly_equal(line.direction(), Vec3::unit_z()));
    EXPECT_POINT(line.point_at(5.0), 0.0, 0.0, 5.0);
    EXPECT_POINT(line.point_at(-2.0), 0.0, 0.0, -2.0);
    CHECK_NEAR(line.point_at(3.5).distance_to(line.origin()), 3.5);
    CHECK_NEAR(line.point_at(-2.0).distance_to(line.origin()), 2.0);
    CHECK(geo::nearly_equal(line.point_at(2.0), line.reversed().point_at(-2.0)));
    CHECK(line.reversed().direction() == -line.direction());
    CHECK(line != line.reversed());
    CHECK(line.coincides_with(line.reversed()));

    const Line3 x_axis = Line3::through(Point3::origin(), Point3{1.0, 0.0, 0.0});
    const Point3 off{10.0, 3.0, 4.0};
    EXPECT_POINT(x_axis.closest_point_to(off), 10.0, 0.0, 0.0);
    CHECK_NEAR(x_axis.parameter_of(off), 10.0);
    CHECK_NEAR(x_axis.distance_to(off), 5.0);
    CHECK_NEAR(x_axis.distance_to(Point3{0.0, 3.0, 4.0}), 5.0);
    CHECK(x_axis.contains(Point3{4.0, 0.0, 0.0}));
    CHECK(!x_axis.contains(off));

    const Line3 shifted = Line3::through(Point3{5.0, 0.0, 0.0}, Point3{8.0, 0.0, 0.0});
    CHECK(x_axis.coincides_with(shifted));
    CHECK(x_axis != shifted);
    const Line3 opposite = Line3::through(Point3::origin(), Point3{-1.0, 0.0, 0.0});
    CHECK(x_axis.coincides_with(opposite));
    const Line3 parallel = Line3::through(Point3{0.0, 1.0, 0.0}, Point3{1.0, 1.0, 0.0});
    CHECK(!x_axis.coincides_with(parallel));

    const Line3 constructed{Point3{0.0, 0.0, -2.0}, Vec3{0.0, 0.0, 4.0}};
    CHECK(geo::nearly_equal(constructed.direction(), Vec3::unit_z()));
    EXPECT_POINT(constructed.point_at(2.0), 0.0, 0.0, 0.0);

    EXPECT_INVALID(Line3(Point3::origin(), Vec3::zero()));
    EXPECT_INVALID(Line3::through(Point3{1.0, 2.0, 3.0}, Point3{1.0, 2.0, 3.0}));
}

void test_plane3() {
    using geo::Plane3;
    using geo::PlaneSide;
    using geo::Point3;
    using geo::Vec3;

    const Plane3 xy = Plane3::from_point_and_normal(Point3::origin(), Vec3::unit_z());
    CHECK(xy.normal() == Vec3::unit_z());
    CHECK(xy.offset() == 0.0);
    CHECK_NEAR(xy.signed_distance_to(Point3{1.0, 2.0, 3.0}), 3.0);
    CHECK_NEAR(xy.signed_distance_to(Point3{1.0, 2.0, -4.0}), -4.0);
    CHECK_NEAR(xy.distance_to(Point3{1.0, 2.0, -4.0}), 4.0);
    EXPECT_POINT(xy.closest_point_to(Point3{1.0, 2.0, 3.0}), 1.0, 2.0, 0.0);
    EXPECT_POINT(xy.point_closest_to_origin(), 0.0, 0.0, 0.0);
    CHECK(xy.contains(Point3{5.0, -2.0, 0.0}));
    CHECK(xy.side_of(Point3{0.0, 0.0, 1.0}) == PlaneSide::Positive);
    CHECK(xy.side_of(Point3{0.0, 0.0, -1.0}) == PlaneSide::Negative);
    CHECK(xy.side_of(Point3{2.0, 2.0, 0.0}) == PlaneSide::On);

    const Plane3 flipped = xy.flipped();
    CHECK(flipped.normal() == -xy.normal());
    CHECK(flipped != xy);
    CHECK(flipped.coincides_with(xy));
    CHECK_NEAR(flipped.signed_distance_to(Point3{0.0, 0.0, 3.0}), -3.0);

    const Point3 a{0.0, 0.0, 0.0};
    const Point3 b{1.0, 0.0, 0.0};
    const Point3 c{0.0, 1.0, 0.0};
    const Plane3 from_points = Plane3::from_points(a, b, c);
    CHECK(geo::nearly_equal(from_points.normal(), Vec3::unit_z()));
    CHECK(from_points.coincides_with(xy));
    const Plane3 reversed_winding = Plane3::from_points(a, c, b);
    CHECK(geo::nearly_equal(reversed_winding.normal(), -from_points.normal()));
    CHECK(from_points.coincides_with(reversed_winding));
    CHECK(from_points != reversed_winding);

    const Plane3 z_equals_5 = Plane3::from_coefficients(0.0, 0.0, 1.0, -5.0);
    CHECK(geo::nearly_equal(z_equals_5.normal(), Vec3::unit_z()));
    CHECK_NEAR(z_equals_5.offset(), 5.0);
    CHECK(z_equals_5.contains(Point3{4.0, -1.0, 5.0}));
    EXPECT_POINT(z_equals_5.point_closest_to_origin(), 0.0, 0.0, 5.0);

    const Plane3 scaled = Plane3::from_coefficients(2.0, 2.0, 2.0, -2.0);
    const double inv_sqrt3 = 1.0 / std::sqrt(3.0);
    CHECK(geo::nearly_equal(scaled.normal(), Vec3{inv_sqrt3, inv_sqrt3, inv_sqrt3}));
    CHECK_NEAR(scaled.offset(), inv_sqrt3);
    CHECK(scaled.contains(Point3{1.0, 0.0, 0.0}));
    CHECK_NEAR(scaled.signed_distance_to(Point3::origin()), -inv_sqrt3);
    EXPECT_POINT(scaled.point_closest_to_origin(), 1.0 / 3.0, 1.0 / 3.0, 1.0 / 3.0);
    EXPECT_POINT(scaled.closest_point_to(Point3{1.0, 1.0, 1.0}), 1.0 / 3.0, 1.0 / 3.0, 1.0 / 3.0);

    const Plane3 z_equals_1 = Plane3::from_point_and_normal(Point3{0.0, 0.0, 1.0}, Vec3::unit_z());
    const Plane3 z_equals_1_down =
        Plane3::from_point_and_normal(Point3{0.0, 0.0, 1.0}, -Vec3::unit_z());
    CHECK(z_equals_1.coincides_with(z_equals_1_down));
    CHECK(!z_equals_1.coincides_with(xy));
    CHECK_NEAR(z_equals_1.offset(), 1.0);
    CHECK_NEAR(z_equals_1_down.offset(), -1.0);

    const Plane3 yz = Plane3::from_coefficients(2.0, 0.0, 0.0, 0.0);
    CHECK(geo::nearly_equal(yz.normal(), Vec3::unit_x()));
    CHECK_NEAR(yz.offset(), 0.0);

    EXPECT_INVALID(Plane3::from_point_and_normal(Point3::origin(), Vec3::zero()));
    EXPECT_INVALID(Plane3::from_points(a, b, Point3{2.0, 0.0, 0.0}));
    EXPECT_INVALID(Plane3::from_points(a, a, b));
    EXPECT_INVALID(Plane3::from_coefficients(0.0, 0.0, 0.0, 1.0));
}

void test_intersections() {
    using geo::Line3;
    using geo::LinePlaneRelation;
    using geo::Plane3;
    using geo::PlanePlaneRelation;
    using geo::Point3;
    using geo::Vec3;

    const Plane3 xy = Plane3::from_point_and_normal(Point3::origin(), Vec3::unit_z());
    const Line3 vertical{Point3{1.0, 2.0, -4.0}, Vec3{0.0, 0.0, 8.0}};
    const auto hit = geo::intersect(vertical, xy);
    CHECK(hit.relation == LinePlaneRelation::Point);
    CHECK(hit.point.has_value());
    if (hit.point) {
        EXPECT_POINT(*hit.point, 1.0, 2.0, 0.0);
    }
    const auto hit_swapped = geo::intersect(xy, vertical);
    CHECK(hit_swapped.relation == hit.relation);
    CHECK(hit_swapped.point.has_value());

    const Line3 crossing_at_origin{Point3::origin(), Vec3::unit_z()};
    const auto through_origin = geo::intersect(crossing_at_origin, xy);
    CHECK(through_origin.relation == LinePlaneRelation::Point);
    if (through_origin.point) {
        EXPECT_POINT(*through_origin.point, 0.0, 0.0, 0.0);
    }

    const Line3 in_xy{Point3{2.0, -3.0, 0.0}, Vec3::unit_x()};
    const auto contained = geo::intersect(in_xy, xy);
    CHECK(contained.relation == LinePlaneRelation::Contained);
    CHECK(!contained.point.has_value());

    const Line3 above{Point3{0.0, 0.0, 1.0}, Vec3::unit_x()};
    const auto parallel = geo::intersect(above, xy);
    CHECK(parallel.relation == LinePlaneRelation::Parallel);
    CHECK(!parallel.point.has_value());

    const double inv_sqrt3 = 1.0 / std::sqrt(3.0);
    const Plane3 diagonal =
        Plane3::from_point_and_normal(Point3{1.0, 0.0, 0.0}, Vec3{1.0, 1.0, 1.0});
    const Line3 toward_up{Point3::origin(), Vec3::unit_z()};
    const auto diagonal_hit = geo::intersect(toward_up, diagonal);
    CHECK(diagonal_hit.relation == LinePlaneRelation::Point);
    if (diagonal_hit.point) {
        EXPECT_POINT(*diagonal_hit.point, 0.0, 0.0, 1.0);
    }
    CHECK_NEAR(diagonal.offset(), inv_sqrt3);

    const Line3 to_corner{Point3::origin(), Vec3{1.0, 1.0, 1.0}};
    const Plane3 x_equals_1 = Plane3::from_point_and_normal(Point3{1.0, 0.0, 0.0}, Vec3::unit_x());
    const auto corner = geo::intersect(to_corner, x_equals_1);
    CHECK(corner.relation == LinePlaneRelation::Point);
    if (corner.point) {
        EXPECT_POINT(*corner.point, 1.0, 1.0, 1.0);
    }

    const Line3 in_diagonal{Point3{1.0 / 3.0, 1.0 / 3.0, 1.0 / 3.0}, Vec3{1.0, -1.0, 0.0}};
    CHECK(geo::intersect(in_diagonal, diagonal).relation == LinePlaneRelation::Contained);
    const Line3 parallel_diagonal{Point3::origin(), Vec3{1.0, -1.0, 0.0}};
    CHECK(geo::intersect(parallel_diagonal, diagonal).relation == LinePlaneRelation::Parallel);

    const Plane3 y0 = Plane3::from_point_and_normal(Point3::origin(), Vec3::unit_y());
    const auto axes = geo::intersect(xy, y0);
    CHECK(axes.relation == PlanePlaneRelation::Line);
    CHECK(axes.line.has_value());
    if (axes.line) {
        CHECK(axes.line->coincides_with(Line3{Point3::origin(), Vec3::unit_x()}));
        CHECK(xy.contains(axes.line->point_at(-2.0)));
        CHECK(xy.contains(axes.line->point_at(3.0)));
        CHECK(y0.contains(axes.line->point_at(-2.0)));
        CHECK(y0.contains(axes.line->point_at(3.0)));
        CHECK(geo::nearly_zero(geo::dot(axes.line->direction(), xy.normal())));
        CHECK(geo::nearly_zero(geo::dot(axes.line->direction(), y0.normal())));
    }
    const auto axes_flipped = geo::intersect(y0, xy);
    CHECK(axes_flipped.relation == PlanePlaneRelation::Line);
    if (axes.line && axes_flipped.line) {
        CHECK(axes.line->coincides_with(*axes_flipped.line));
        CHECK(geo::nearly_equal(axes.line->direction(), -axes_flipped.line->direction()));
    }

    const Plane3 x1 = Plane3::from_point_and_normal(Point3{1.0, 0.0, 0.0}, Vec3::unit_x());
    const Plane3 y2 = Plane3::from_point_and_normal(Point3{0.0, 2.0, 0.0}, Vec3::unit_y());
    const auto offset_line = geo::intersect(x1, y2);
    CHECK(offset_line.relation == PlanePlaneRelation::Line);
    if (offset_line.line) {
        CHECK(offset_line.line->coincides_with(Line3{Point3{1.0, 2.0, 0.0}, Vec3::unit_z()}));
        CHECK(x1.contains(offset_line.line->origin()));
        CHECK(y2.contains(offset_line.line->point_at(4.0)));
    }

    const Plane3 z1 = Plane3::from_point_and_normal(Point3{0.0, 0.0, 1.0}, Vec3::unit_z());
    const auto parallel_planes = geo::intersect(xy, z1);
    CHECK(parallel_planes.relation == PlanePlaneRelation::Parallel);
    CHECK(!parallel_planes.line.has_value());

    const Plane3 z1_down = Plane3::from_point_and_normal(Point3{4.0, 5.0, 1.0}, -Vec3::unit_z());
    const auto coincident = geo::intersect(z1, z1_down);
    CHECK(coincident.relation == PlanePlaneRelation::Coincident);
    CHECK(!coincident.line.has_value());
    CHECK(z1.coincides_with(Plane3::from_coefficients(0.0, 0.0, 2.0, -2.0)));
}

}  // namespace

static_assert(geo::dot(geo::Vec3{1.0, 2.0, 3.0}, geo::Vec3{4.0, -5.0, 6.0}) == 12.0, "");
static_assert(geo::cross(geo::Vec3::unit_x(), geo::Vec3::unit_y()) == geo::Vec3::unit_z(), "");
static_assert(geo::Point3::lerp(geo::Point3{0.0, 0.0, 0.0}, geo::Point3{10.0, 0.0, 0.0}, 0.25) ==
                  geo::Point3{2.5, 0.0, 0.0},
              "");

int main() {
    std::cerr.setf(std::ios::scientific);
    std::cerr.precision(17);

    test_vec3();
    test_point3();
    test_line3();
    test_plane3();
    test_intersections();

    if (g_failed != 0) {
        std::cerr << g_failed << " check(s) failed\n";
        return EXIT_FAILURE;
    }
    std::cout << "all checks passed\n";
    return EXIT_SUCCESS;
}
