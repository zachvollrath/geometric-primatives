#include "geo/geo.hpp"

#include <array>
#include <iostream>

namespace {

void print_point(const geo::Point3& point) {
    std::cout << "(" << point.x << ", " << point.y << ", " << point.z << ")";
}

void print_vector(const geo::Vec3& vector) {
    std::cout << "(" << vector.x << ", " << vector.y << ", " << vector.z << ")";
}

void heading(const char* title) { std::cout << "\n" << title << "\n"; }

}  // namespace

int main() {
    using geo::Line3;
    using geo::Plane3;
    using geo::PlanePlaneRelation;
    using geo::Point3;
    using geo::Segment3;
    using geo::SegmentIntersectionKind;
    using geo::Sphere3;
    using geo::Tetrahedron3;
    using geo::Vec3;

    std::cout.setf(std::ios::fixed);
    std::cout.precision(6);
    std::cout << std::boolalpha;
    std::cout << "Geometric primitives\n";

    heading("Points");
    const Point3 origin = Point3::origin();
    const Point3 tip{3.0, 4.0, 0.0};
    std::cout << "Distance between ";
    print_point(origin);
    std::cout << " and ";
    print_point(tip);
    std::cout << ": " << origin.distance_to(tip) << "\n";
    std::cout << "Direction from ";
    print_point(origin);
    std::cout << " to ";
    print_point(tip);
    std::cout << ": ";
    print_vector(origin.direction_to(tip));
    std::cout << "\n";

    heading("Line segment");
    const Segment3 segment{origin, Point3{0.0, 0.0, 4.0}};
    std::cout << "Segment from ";
    print_point(segment.start());
    std::cout << " to ";
    print_point(segment.end());
    std::cout << "\n";
    std::cout << "Length: " << segment.length() << "\n";
    std::cout << "Direction: ";
    print_vector(segment.direction());
    std::cout << "\n";

    const Segment3 horizontal{origin, Point3{2.0, 0.0, 0.0}};
    const Segment3 vertical{Point3{1.0, -1.0, 0.0}, Point3{1.0, 1.0, 0.0}};
    const auto crossing = horizontal.intersect(vertical);
    std::cout << "Intersection of ";
    print_point(horizontal.start());
    std::cout << "--";
    print_point(horizontal.end());
    std::cout << " and ";
    print_point(vertical.start());
    std::cout << "--";
    print_point(vertical.end());
    std::cout << ": ";
    if (crossing.kind == SegmentIntersectionKind::Point && crossing.point) {
        print_point(*crossing.point);
    } else {
        std::cout << "none";
    }
    std::cout << "\n";

    const Segment3 x_axis{origin, Point3{4.0, 0.0, 0.0}};
    const Point3 off_axis{2.0, 3.0, 0.0};
    const auto nearest = x_axis.closest_to(off_axis);
    std::cout << "Closest point on ";
    print_point(x_axis.start());
    std::cout << "--";
    print_point(x_axis.end());
    std::cout << " to ";
    print_point(off_axis);
    std::cout << ": ";
    print_point(nearest.point);
    std::cout << ", distance " << nearest.distance << "\n";

    const Segment3 skew{Point3{1.0, 2.0, -1.0}, Point3{1.0, 2.0, 1.0}};
    const auto pair = x_axis.closest_points_to(skew);
    std::cout << "Closest points between ";
    print_point(x_axis.start());
    std::cout << "--";
    print_point(x_axis.end());
    std::cout << " and ";
    print_point(skew.start());
    std::cout << "--";
    print_point(skew.end());
    std::cout << ": ";
    print_point(pair.point_on_first);
    std::cout << " and ";
    print_point(pair.point_on_second);
    std::cout << ", distance " << pair.distance << "\n";

    const Segment3 shifted{Point3{0.0, 1.0, 0.0}, Point3{3.0, 1.0, 0.0}};
    const Segment3 upright{origin, Point3{0.0, 2.0, 0.0}};
    std::cout << "Segments along x and along x+1 in y are parallel: " << x_axis.is_parallel_to(shifted)
              << ", perpendicular: " << x_axis.is_perpendicular_to(shifted) << "\n";
    std::cout << "Segments along x and along y are parallel: " << x_axis.is_parallel_to(upright)
              << ", perpendicular: " << x_axis.is_perpendicular_to(upright) << "\n";

    heading("Planes");
    const Plane3 xy = Plane3::from_points(origin, Point3{1.0, 0.0, 0.0}, Point3{0.0, 1.0, 0.0});
    std::cout << "Plane through (0, 0, 0), (1, 0, 0), (0, 1, 0): normal ";
    print_vector(xy.normal());
    std::cout << ", offset " << xy.offset() << "\n";

    const Plane3 x1 = Plane3::from_point_and_normal(Point3{1.0, 0.0, 0.0}, Vec3::unit_x());
    const Plane3 y2 = Plane3::from_point_and_normal(Point3{0.0, 2.0, 0.0}, Vec3::unit_y());
    const Plane3 z3 = Plane3::from_point_and_normal(Point3{0.0, 0.0, 3.0}, Vec3::unit_z());
    const auto triple_point = geo::intersect(x1, y2, z3);
    std::cout << "Intersection of x = 1, y = 2, and z = 3: ";
    if (triple_point) {
        print_point(*triple_point);
    } else {
        std::cout << "none";
    }
    std::cout << "\n";

    const Point3 above{1.0, 2.0, 5.0};
    std::cout << "Closest point on the xy plane to ";
    print_point(above);
    std::cout << ": ";
    print_point(xy.closest_point_to(above));
    std::cout << ", distance " << xy.distance_to(above) << "\n";

    const Plane3 xz = Plane3::from_point_and_normal(origin, Vec3::unit_y());
    const auto plane_line = geo::intersect(xy, xz);
    std::cout << "Intersection of the xy and xz planes: ";
    if (plane_line.relation == PlanePlaneRelation::Line && plane_line.line) {
        std::cout << "line through ";
        print_point(plane_line.line->origin());
        std::cout << " direction ";
        print_vector(plane_line.line->direction());
    } else {
        std::cout << "none";
    }
    std::cout << "\n";

    const Plane3 tilted = Plane3::from_point_and_normal(origin, Vec3{0.0, 1.0, 1.0});
    std::cout << "Angle between xy and xz: " << xy.angle_to(xz) << " rad ("
              << xy.angle_to(xz) * 180.0 / geo::pi << " deg)\n";
    std::cout << "Angle between xy and the plane with normal (0, 1, 1): " << xy.angle_to(tilted) << " rad ("
              << xy.angle_to(tilted) * 180.0 / geo::pi << " deg)\n";

    const Plane3 z4 = Plane3::from_point_and_normal(Point3{0.0, 0.0, 4.0}, Vec3::unit_z());
    std::cout << "xy and z = 4 are parallel: " << xy.is_parallel_to(z4);
    if (const auto gap = xy.distance_between(z4)) {
        std::cout << ", distance " << *gap;
    }
    std::cout << "\n";
    std::cout << "xy and xz are parallel: " << xy.is_parallel_to(xz) << "\n";

    heading("Sphere");
    const Point3 a = origin;
    const Point3 b{1.0, 0.0, 0.0};
    const Point3 c{0.0, 1.0, 0.0};
    const Point3 d{0.0, 0.0, 1.0};
    const Sphere3 sphere = Sphere3::from_points(a, b, c, d);
    std::cout << "Sphere through (0, 0, 0), (1, 0, 0), (0, 1, 0), (0, 0, 1)\n";
    std::cout << "Center: ";
    print_point(sphere.center());
    std::cout << "\nRadius: " << sphere.radius() << "\n";
    std::cout << "Volume: " << sphere.volume() << "\n";
    std::cout << "Surface area: " << sphere.surface_area() << "\n";

    heading("Tetrahedron");
    const Tetrahedron3 tetrahedron = Tetrahedron3::from_points(a, b, c, d);
    std::cout << "From points (0, 0, 0), (1, 0, 0), (0, 1, 0), (0, 0, 1)\n";
    std::cout << "Centroid: ";
    print_point(tetrahedron.centroid());
    std::cout << "\nVolume: " << tetrahedron.volume() << "\n";
    std::cout << "Surface area: " << tetrahedron.surface_area() << "\n";

    const Plane3 face_x = Plane3::from_point_and_normal(a, Vec3::unit_x());
    const Plane3 face_y = Plane3::from_point_and_normal(a, Vec3::unit_y());
    const Plane3 face_z = Plane3::from_point_and_normal(a, Vec3::unit_z());
    const Plane3 face_diag = Plane3::from_point_and_normal(b, Vec3{1.0, 1.0, 1.0});
    const Tetrahedron3 from_planes = Tetrahedron3::from_planes(face_z, face_y, face_x, face_diag);
    std::cout << "From planes x = 0, y = 0, z = 0, and x + y + z = 1\n";
    std::cout << "Centroid: ";
    print_point(from_planes.centroid());
    std::cout << "\nVolume: " << from_planes.volume() << "\n";
    std::cout << "Surface area: " << from_planes.surface_area() << "\n";

    const std::array<Line3, 6> edges{
        Line3::through(a, b), Line3::through(a, c), Line3::through(a, d),
        Line3::through(b, c), Line3::through(b, d), Line3::through(c, d),
    };
    const Tetrahedron3 from_lines = Tetrahedron3::from_lines(edges);
    std::cout << "From the six edge lines\n";
    std::cout << "Centroid: ";
    print_point(from_lines.centroid());
    std::cout << "\nVolume: " << from_lines.volume() << "\n";
    std::cout << "Surface area: " << from_lines.surface_area() << "\n";

    return 0;
}
