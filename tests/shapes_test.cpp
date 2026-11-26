#include "geo/geo.hpp"

#include <array>
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

bool has_vertex(const geo::Tetrahedron3& tetrahedron, const geo::Point3& point) {
    for (std::size_t index = 0; index < 4; ++index) {
        if (geo::nearly_equal(tetrahedron.vertex(index), point)) {
            return true;
        }
    }
    return false;
}

#define CHECK(...) ::check(static_cast<bool>(__VA_ARGS__), #__VA_ARGS__, __FILE__, __LINE__)
#define CHECK_NEAR(actual, expected) ::check_near((actual), (expected), __FILE__, __LINE__)
#define EXPECT_POINT(point, x, y, z) ::expect_point((point), (x), (y), (z), __FILE__, __LINE__)
#define EXPECT_INVALID(...) \
    ::expect_invalid_argument([&] { (void)(__VA_ARGS__); }, __FILE__, __LINE__)

void test_direction_and_lines() {
    using geo::Line3;
    using geo::LineLineRelation;
    using geo::Point3;
    using geo::Vec3;

    const Point3 origin = Point3::origin();
    const Point3 tip{3.0, 4.0, 0.0};
    CHECK_NEAR(origin.distance_to(tip), 5.0);
    CHECK(geo::nearly_equal(origin.direction_to(tip), Vec3{0.6, 0.8, 0.0}));
    EXPECT_INVALID(origin.direction_to(origin));

    const auto crossing = geo::intersect(Line3{origin, Vec3::unit_x()},
                                          Line3{Point3{0.0, -1.0, 0.0}, Vec3::unit_y()});
    CHECK(crossing.relation == LineLineRelation::Point);
    if (crossing.point) {
        EXPECT_POINT(*crossing.point, 0.0, 0.0, 0.0);
    }

    const auto parallel = geo::intersect(Line3{origin, Vec3::unit_x()},
                                          Line3{Point3{0.0, 1.0, 0.0}, Vec3::unit_x()});
    CHECK(parallel.relation == LineLineRelation::Parallel);
    CHECK(!parallel.point.has_value());

    const auto same = geo::intersect(Line3{origin, Vec3::unit_x()},
                                      Line3{Point3{2.0, 0.0, 0.0}, -Vec3::unit_x()});
    CHECK(same.relation == LineLineRelation::Coincident);

    const auto skew = geo::intersect(Line3{origin, Vec3::unit_x()},
                                      Line3{Point3{0.0, 1.0, 0.0}, Vec3::unit_z()});
    CHECK(skew.relation == LineLineRelation::Skew);
}

void test_segments() {
    using geo::Point3;
    using geo::Segment3;
    using geo::SegmentIntersectionKind;
    using geo::Vec3;

    const Segment3 along_z{Point3::origin(), Point3{0.0, 0.0, 4.0}};
    CHECK_NEAR(along_z.length(), 4.0);
    CHECK(geo::nearly_equal(along_z.direction(), Vec3::unit_z()));
    EXPECT_POINT(along_z.point_at(0.25), 0.0, 0.0, 1.0);

    const Segment3 x_axis{Point3::origin(), Point3{4.0, 0.0, 0.0}};
    const auto interior = x_axis.closest_to(Point3{2.0, 3.0, 0.0});
    EXPECT_POINT(interior.point, 2.0, 0.0, 0.0);
    CHECK_NEAR(interior.distance, 3.0);
    CHECK_NEAR(interior.parameter, 0.5);

    const auto before = x_axis.closest_to(Point3{-1.0, 0.0, 0.0});
    EXPECT_POINT(before.point, 0.0, 0.0, 0.0);
    CHECK_NEAR(before.distance, 1.0);
    const auto after = x_axis.closest_to(Point3{10.0, 0.0, 0.0});
    EXPECT_POINT(after.point, 4.0, 0.0, 0.0);
    CHECK_NEAR(after.distance, 6.0);

    const Segment3 point_segment{Point3{1.0, 1.0, 0.0}, Point3{1.0, 1.0, 0.0}};
    CHECK_NEAR(point_segment.length(), 0.0);
    EXPECT_INVALID(point_segment.direction());
    const auto from_point = point_segment.closest_to(Point3{1.0, 4.0, 0.0});
    EXPECT_POINT(from_point.point, 1.0, 1.0, 0.0);
    CHECK_NEAR(from_point.distance, 3.0);

    const Segment3 cross_a{Point3::origin(), Point3{2.0, 0.0, 0.0}};
    const Segment3 cross_b{Point3{1.0, -1.0, 0.0}, Point3{1.0, 1.0, 0.0}};
    const auto hit = cross_a.intersect(cross_b);
    CHECK(hit.kind == SegmentIntersectionKind::Point);
    if (hit.point) {
        EXPECT_POINT(*hit.point, 1.0, 0.0, 0.0);
    }
    CHECK(cross_a.is_perpendicular_to(cross_b));
    CHECK(!cross_a.is_parallel_to(cross_b));

    const Segment3 skew_a{Point3::origin(), Point3{2.0, 0.0, 0.0}};
    const Segment3 skew_b{Point3{1.0, 2.0, -1.0}, Point3{1.0, 2.0, 1.0}};
    const auto skew = skew_a.closest_points_to(skew_b);
    EXPECT_POINT(skew.point_on_first, 1.0, 0.0, 0.0);
    EXPECT_POINT(skew.point_on_second, 1.0, 2.0, 0.0);
    CHECK_NEAR(skew.distance, 2.0);
    CHECK(skew_a.intersect(skew_b).kind == SegmentIntersectionKind::None);
    CHECK(skew_a.is_perpendicular_to(skew_b));

    const Segment3 parallel{Point3{0.0, 1.0, 0.0}, Point3{3.0, 1.0, 0.0}};
    const Segment3 anti{Point3{2.0, 0.0, 0.0}, Point3::origin()};
    CHECK(cross_a.is_parallel_to(parallel));
    CHECK(cross_a.is_parallel_to(anti));
    CHECK(!cross_a.is_perpendicular_to(parallel));
    CHECK(cross_a.intersect(parallel).kind == SegmentIntersectionKind::None);
    const auto parallel_gap = cross_a.closest_points_to(parallel);
    CHECK_NEAR(parallel_gap.distance, 1.0);

    const Segment3 overlap_b{Point3{1.0, 0.0, 0.0}, Point3{3.0, 0.0, 0.0}};
    const auto overlap = cross_a.intersect(overlap_b);
    CHECK(overlap.kind == SegmentIntersectionKind::Overlap);
    EXPECT_POINT(overlap.overlap_start, 1.0, 0.0, 0.0);
    EXPECT_POINT(overlap.overlap_end, 2.0, 0.0, 0.0);

    const Segment3 touch{Point3{2.0, 0.0, 0.0}, Point3{4.0, 0.0, 0.0}};
    const auto touched = cross_a.intersect(touch);
    CHECK(touched.kind == SegmentIntersectionKind::Point);
    if (touched.point) {
        EXPECT_POINT(*touched.point, 2.0, 0.0, 0.0);
    }

    const Segment3 separated{Point3{3.0, 0.0, 0.0}, Point3{4.0, 0.0, 0.0}};
    CHECK(cross_a.intersect(separated).kind == SegmentIntersectionKind::None);

    const Segment3 degenerate{Point3::origin(), Point3::origin()};
    CHECK(!degenerate.is_parallel_to(cross_a));
    CHECK(!degenerate.is_perpendicular_to(cross_a));
}

void test_planes() {
    using geo::Plane3;
    using geo::Point3;
    using geo::Vec3;

    const Plane3 xy = Plane3::from_points(Point3::origin(), Point3{1.0, 0.0, 0.0}, Point3{0.0, 1.0, 0.0});
    CHECK(geo::nearly_equal(xy.normal(), Vec3::unit_z()));
    CHECK_NEAR(xy.offset(), 0.0);

    const Plane3 x1 = Plane3::from_point_and_normal(Point3{1.0, 0.0, 0.0}, Vec3::unit_x());
    const Plane3 y2 = Plane3::from_point_and_normal(Point3{0.0, 2.0, 0.0}, Vec3::unit_y());
    const Plane3 z3 = Plane3::from_point_and_normal(Point3{0.0, 0.0, 3.0}, Vec3::unit_z());
    const auto triple = geo::intersect(x1, y2, z3);
    CHECK(triple.has_value());
    if (triple) {
        EXPECT_POINT(*triple, 1.0, 2.0, 3.0);
    }
    const Plane3 z1 = Plane3::from_point_and_normal(Point3{0.0, 0.0, 1.0}, Vec3::unit_z());
    CHECK(!geo::intersect(xy, z1, x1).has_value());

    const Point3 off{1.0, 2.0, 5.0};
    EXPECT_POINT(xy.closest_point_to(off), 1.0, 2.0, 0.0);
    CHECK_NEAR(xy.distance_to(off), 5.0);

    const Plane3 xz = Plane3::from_point_and_normal(Point3::origin(), Vec3::unit_y());
    const auto line = geo::intersect(xy, xz);
    CHECK(line.relation == geo::PlanePlaneRelation::Line);
    if (line.line) {
        CHECK(line.line->coincides_with(geo::Line3{Point3::origin(), Vec3::unit_x()}));
    }

    CHECK_NEAR(xy.angle_to(xz), geo::pi / 2.0);
    const Plane3 tilted = Plane3::from_point_and_normal(Point3::origin(), Vec3{0.0, 1.0, 1.0});
    CHECK_NEAR(xy.angle_to(tilted), geo::pi / 4.0);
    CHECK_NEAR(xy.angle_to(xy), 0.0);

    const Plane3 z5 = Plane3::from_point_and_normal(Point3{0.0, 0.0, 5.0}, Vec3::unit_z());
    const Plane3 z5_down = Plane3::from_point_and_normal(Point3{1.0, 2.0, 5.0}, -Vec3::unit_z());
    CHECK(xy.is_parallel_to(z5));
    CHECK(xy.is_parallel_to(z5_down));
    CHECK(xy.distance_between(z5).has_value());
    CHECK(xy.distance_between(z5_down).has_value());
    if (xy.distance_between(z5) && xy.distance_between(z5_down)) {
        CHECK_NEAR(*xy.distance_between(z5), 5.0);
        CHECK_NEAR(*xy.distance_between(z5_down), 5.0);
    }
    CHECK_NEAR(*xy.distance_between(xy), 0.0);
    CHECK(!xy.is_parallel_to(xz));
    CHECK(!xy.distance_between(xz).has_value());
}

void test_sphere_and_tetrahedron() {
    using geo::Line3;
    using geo::Plane3;
    using geo::Point3;
    using geo::Sphere3;
    using geo::Tetrahedron3;
    using geo::Vec3;

    const Point3 a{0.0, 0.0, 0.0};
    const Point3 b{1.0, 0.0, 0.0};
    const Point3 c{0.0, 1.0, 0.0};
    const Point3 d{0.0, 0.0, 1.0};

    const Sphere3 sphere = Sphere3::from_points(a, b, c, d);
    EXPECT_POINT(sphere.center(), 0.5, 0.5, 0.5);
    CHECK_NEAR(sphere.radius(), std::sqrt(0.75));
    CHECK_NEAR(sphere.volume(), geo::pi * std::sqrt(3.0) / 2.0);
    CHECK_NEAR(sphere.surface_area(), 3.0 * geo::pi);
    CHECK_NEAR(sphere.center().distance_to(b), sphere.radius());
    CHECK_NEAR(sphere.center().distance_to(c), sphere.radius());
    CHECK_NEAR(sphere.center().distance_to(d), sphere.radius());

    const Point3 center{1.0, 2.0, 3.0};
    const Sphere3 known = Sphere3::from_center_and_radius(center, 2.0);
    CHECK_NEAR(known.volume(), (4.0 / 3.0) * geo::pi * 8.0);
    CHECK_NEAR(known.surface_area(), 4.0 * geo::pi * 4.0);
    const Sphere3 recovered = Sphere3::from_points(center + Vec3{2.0, 0.0, 0.0},
                                                    center + Vec3{0.0, 2.0, 0.0},
                                                    center + Vec3{0.0, 0.0, 2.0},
                                                    center + Vec3{-2.0, 0.0, 0.0});
    EXPECT_POINT(recovered.center(), 1.0, 2.0, 3.0);
    CHECK_NEAR(recovered.radius(), 2.0);

    EXPECT_INVALID(Sphere3::from_center_and_radius(center, 0.0));
    EXPECT_INVALID(Sphere3::from_center_and_radius(center, -1.0));
    EXPECT_INVALID(Sphere3::from_points(a, b, c, Point3{1.0, 1.0, 0.0}));

    const Tetrahedron3 tetrahedron{a, b, c, d};
    EXPECT_POINT(tetrahedron.centroid(), 0.25, 0.25, 0.25);
    CHECK_NEAR(tetrahedron.volume(), 1.0 / 6.0);
    CHECK_NEAR(tetrahedron.surface_area(), 1.5 + std::sqrt(3.0) / 2.0);
    CHECK(Tetrahedron3::from_points(a, b, c, d).volume() == tetrahedron.volume());
    EXPECT_INVALID(Tetrahedron3(a, b, c, Point3{1.0, 1.0, 0.0}));

    const Plane3 x0 = Plane3::from_point_and_normal(a, Vec3::unit_x());
    const Plane3 y0 = Plane3::from_point_and_normal(a, Vec3::unit_y());
    const Plane3 z0 = Plane3::from_point_and_normal(a, Vec3::unit_z());
    const Plane3 diagonal = Plane3::from_point_and_normal(b, Vec3{1.0, 1.0, 1.0});
    const Tetrahedron3 from_planes = Tetrahedron3::from_planes(z0, y0, x0, diagonal);
    CHECK_NEAR(from_planes.volume(), tetrahedron.volume());
    EXPECT_POINT(from_planes.centroid(), 0.25, 0.25, 0.25);
    CHECK(has_vertex(from_planes, a));
    CHECK(has_vertex(from_planes, b));
    CHECK(has_vertex(from_planes, c));
    CHECK(has_vertex(from_planes, d));
    EXPECT_INVALID(Tetrahedron3::from_planes(z0, z0, x0, y0));

    const std::array<Line3, 6> edges{
        Line3::through(a, b),
        Line3::through(a, c),
        Line3::through(a, d),
        Line3::through(b, c),
        Line3::through(b, d),
        Line3::through(c, d),
    };
    const Tetrahedron3 from_lines = Tetrahedron3::from_lines(edges);
    CHECK_NEAR(from_lines.volume(), tetrahedron.volume());
    EXPECT_POINT(from_lines.centroid(), 0.25, 0.25, 0.25);
    CHECK(has_vertex(from_lines, a));
    CHECK(has_vertex(from_lines, b));
    CHECK(has_vertex(from_lines, c));
    CHECK(has_vertex(from_lines, d));

    const std::array<Line3, 6> reordered{
        edges[5], edges[1], edges[4], edges[0], edges[2], edges[3],
    };
    const Tetrahedron3 reordered_tet = Tetrahedron3::from_lines(reordered);
    EXPECT_POINT(reordered_tet.centroid(), 0.25, 0.25, 0.25);

    const std::array<Line3, 6> invalid{
        Line3{a, Vec3::unit_x()},
        Line3{a, Vec3::unit_x()},
        Line3{a, Vec3::unit_x()},
        Line3{a, Vec3::unit_x()},
        Line3{a, Vec3::unit_x()},
        Line3{a, Vec3::unit_x()},
    };
    EXPECT_INVALID(Tetrahedron3::from_lines(invalid));
}

}  // namespace

int main() {
    std::cerr.setf(std::ios::scientific);
    std::cerr.precision(17);

    test_direction_and_lines();
    test_segments();
    test_planes();
    test_sphere_and_tetrahedron();

    if (g_failed != 0) {
        std::cerr << g_failed << " check(s) failed\n";
        return EXIT_FAILURE;
    }
    std::cout << "all checks passed\n";
    return EXIT_SUCCESS;
}
