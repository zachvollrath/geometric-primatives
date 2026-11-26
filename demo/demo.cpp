#include "geo/geo.hpp"
#include "scene_view.hpp"

#include <array>
#include <iostream>
#include <string>

namespace {

void print_point(const geo::Point3& point) {
    std::cout << "(" << point.x << ", " << point.y << ", " << point.z << ")";
}

void print_vector(const geo::Vec3& vector) {
    std::cout << "(" << vector.x << ", " << vector.y << ", " << vector.z << ")";
}

void heading(const char* title) { std::cout << "\n" << title << "\n"; }

const demo_view::Rgb kRed{0.86, 0.28, 0.24};
const demo_view::Rgb kBlue{0.25, 0.48, 0.88};
const demo_view::Rgb kGreen{0.22, 0.72, 0.42};
const demo_view::Rgb kOrange{0.95, 0.58, 0.16};
const demo_view::Rgb kYellow{0.95, 0.82, 0.22};
const demo_view::Rgb kWhite{0.94, 0.95, 0.97};
const demo_view::Rgb kTeal{0.16, 0.70, 0.72};

void add_normal(demo_view::Scene& scene, const geo::Plane3& plane, const geo::Point3& origin, double length) {
    const geo::Point3 start = plane.closest_point_to(origin);
    scene.add_arrow(start, plane.normal(), length, kWhite, 0.035);
}

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

    demo_view::Gallery gallery("visualizations");
    bool images_ok = true;
    const auto show = [&](const demo_view::Scene& scene, const char* filename, const char* title, const std::string& caption) {
        if (!gallery.add(scene, filename, title, caption)) {
            images_ok = false;
        }
    };

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
    {
        demo_view::Scene scene;
        scene.add_point(origin, kBlue, 0.12);
        scene.add_point(tip, kOrange, 0.12);
        scene.add_arrow(origin, origin.direction_to(tip), origin.distance_to(tip), kYellow, 0.07);
        show(scene, "points_direction", "Points",
             "Distance 5. Direction from the blue point to the orange point.");
    }

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
    {
        demo_view::Scene scene;
        scene.add_segment(segment.start(), segment.end(), kBlue, 0.06);
        const Vec3 beside = geo::cross(segment.direction(), Vec3::unit_x()) * 0.35;
        scene.add_arrow(segment.start() + beside, segment.direction(), segment.length(), kYellow, 0.04);
        show(scene, "segment_length", "Line segment", "Length 4, direction along +z.");
    }

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
    {
        demo_view::Scene scene;
        scene.add_segment(horizontal.start(), horizontal.end(), kRed, 0.05);
        scene.add_segment(vertical.start(), vertical.end(), kBlue, 0.05);
        if (crossing.point) {
            scene.add_point(*crossing.point, kYellow, 0.11);
        }
        show(scene, "segment_intersection", "Segment intersection", "The segments meet at (1, 0, 0).");
    }

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
    {
        demo_view::Scene scene;
        scene.add_segment(x_axis.start(), x_axis.end(), kBlue, 0.05);
        scene.add_point(off_axis, kOrange, 0.12);
        scene.add_point(nearest.point, kYellow, 0.12);
        const Vec3 drop = nearest.point - off_axis;
        const double drop_length = drop.length();
        const Vec3 drop_dir = drop / drop_length;
        scene.add_segment(off_axis + drop_dir * 0.16, nearest.point - drop_dir * 0.16, kGreen, 0.028);
        show(scene, "segment_closest_point", "Closest point on a segment",
             "Yellow is the closest point. The green connector has length 3.");
    }

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
    {
        demo_view::Scene scene;
        scene.add_segment(x_axis.start(), x_axis.end(), kBlue, 0.05);
        scene.add_segment(skew.start(), skew.end(), kRed, 0.05);
        scene.add_point(pair.point_on_first, kYellow, 0.1);
        scene.add_point(pair.point_on_second, kYellow, 0.1);
        scene.add_segment(pair.point_on_first, pair.point_on_second, kGreen, 0.025);
        show(scene, "segments_closest_pair", "Closest points between segments",
             "The segments are skew. The green connector has length 2.");
    }

    const Segment3 shifted{Point3{0.0, 1.0, 0.0}, Point3{3.0, 1.0, 0.0}};
    const Segment3 upright{origin, Point3{0.0, 2.0, 0.0}};
    std::cout << "Segments along x and along x+1 in y are parallel: " << x_axis.is_parallel_to(shifted)
              << ", perpendicular: " << x_axis.is_perpendicular_to(shifted) << "\n";
    std::cout << "Segments along x and along y are parallel: " << x_axis.is_parallel_to(upright)
              << ", perpendicular: " << x_axis.is_perpendicular_to(upright) << "\n";
    {
        demo_view::Scene scene;
        scene.add_segment(x_axis.start(), x_axis.end(), kBlue, 0.05);
        scene.add_segment(shifted.start(), shifted.end(), kRed, 0.05);
        scene.add_segment(Point3{0.0, 0.0, 0.0}, Point3{0.0, 1.0, 0.0}, kGreen, 0.025);
        show(scene, "segments_parallel", "Parallel segments", "Both run along x. The green connector is the separation.");
    }
    {
        demo_view::Scene scene;
        scene.add_segment(Point3::origin(), Point3{2.0, 0.0, 0.0}, kBlue, 0.05);
        scene.add_segment(upright.start(), upright.end(), kRed, 0.05);
        show(scene, "segments_perpendicular", "Perpendicular segments", "One runs along x and the other along y.");
    }

    heading("Planes");
    const Plane3 xy = Plane3::from_points(origin, Point3{1.0, 0.0, 0.0}, Point3{0.0, 1.0, 0.0});
    std::cout << "Plane through (0, 0, 0), (1, 0, 0), (0, 1, 0): normal ";
    print_vector(xy.normal());
    std::cout << ", offset " << xy.offset() << "\n";
    {
        demo_view::Scene scene;
        scene.add_plane(xy, origin, 1.6, kBlue, 0.45);
        scene.add_point(origin, kWhite, 0.08);
        scene.add_point(Point3{1.0, 0.0, 0.0}, kRed, 0.08);
        scene.add_point(Point3{0.0, 1.0, 0.0}, kGreen, 0.08);
        const Point3 patch_center{1.0 / 3.0, 1.0 / 3.0, 0.0};
        add_normal(scene, xy, patch_center, 1.2);
        show(scene, "plane_from_points", "Plane from three points",
             "The xy plane. The arrow is the right-hand normal.");
    }

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
    if (triple_point) {
        demo_view::Scene scene;
        scene.add_plane(x1, *triple_point, 1.35, kRed, 0.38);
        scene.add_plane(y2, *triple_point, 1.35, kGreen, 0.38);
        scene.add_plane(z3, *triple_point, 1.35, kBlue, 0.38);
        scene.add_point(*triple_point, kYellow, 0.12);
        show(scene, "three_planes", "Intersection of three planes", "x = 1, y = 2, and z = 3 meet at (1, 2, 3).");
    }

    const Point3 above{1.0, 2.0, 5.0};
    std::cout << "Closest point on the xy plane to ";
    print_point(above);
    std::cout << ": ";
    print_point(xy.closest_point_to(above));
    std::cout << ", distance " << xy.distance_to(above) << "\n";
    {
        const Point3 projected = xy.closest_point_to(above);
        demo_view::Scene scene;
        scene.add_plane(xy, projected, 2.4, kBlue, 0.45);
        scene.add_point(above, kOrange, 0.12);
        scene.add_point(projected, kYellow, 0.12);
        const Vec3 drop = projected - above;
        const Vec3 drop_dir = drop / drop.length();
        scene.add_segment(above + drop_dir * 0.16, projected - drop_dir * 0.16, kGreen, 0.03);
        show(scene, "plane_closest_point", "Closest point on a plane",
             "The foot of the perpendicular is (1, 2, 0). Distance 5.");
    }

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
    if (plane_line.relation == PlanePlaneRelation::Line && plane_line.line) {
        demo_view::Scene scene;
        scene.add_plane(xy, origin, 1.6, kBlue, 0.4);
        scene.add_plane(xz, origin, 1.6, kGreen, 0.4);
        scene.add_segment(plane_line.line->point_at(-1.7), plane_line.line->point_at(1.7), kYellow, 0.045);
        show(scene, "planes_intersection_line", "Intersection of two planes",
             "The xy and xz planes meet in the yellow line.");
    }

    const Plane3 tilted = Plane3::from_point_and_normal(origin, Vec3{0.0, 1.0, 1.0});
    std::cout << "Angle between xy and xz: " << xy.angle_to(xz) << " rad ("
              << xy.angle_to(xz) * 180.0 / geo::pi << " deg)\n";
    std::cout << "Angle between xy and the plane with normal (0, 1, 1): " << xy.angle_to(tilted) << " rad ("
              << xy.angle_to(tilted) * 180.0 / geo::pi << " deg)\n";
    {
        demo_view::Scene scene;
        scene.add_plane(xy, origin, 1.5, kBlue, 0.42);
        scene.add_plane(xz, origin, 1.5, kGreen, 0.42);
        add_normal(scene, xy, origin, 1.1);
        add_normal(scene, xz, Point3{0.0, 0.0, 0.35}, 1.1);
        show(scene, "plane_angle_right", "Right angle between planes", "The xy and xz planes meet at 90 degrees.");
    }
    {
        demo_view::Scene scene;
        scene.add_plane(xy, origin, 1.5, kBlue, 0.42);
        scene.add_plane(tilted, origin, 1.5, kOrange, 0.42);
        add_normal(scene, xy, origin, 1.1);
        add_normal(scene, tilted, Point3{0.0, 0.35, 0.0}, 1.1);
        show(scene, "plane_angle_oblique", "Acute angle between planes",
             "The orange plane's normal is (0, 1, 1). The dihedral angle is 45 degrees.");
    }

    const Plane3 z4 = Plane3::from_point_and_normal(Point3{0.0, 0.0, 4.0}, Vec3::unit_z());
    std::cout << "xy and z = 4 are parallel: " << xy.is_parallel_to(z4);
    if (const auto gap = xy.distance_between(z4)) {
        std::cout << ", distance " << *gap;
    }
    std::cout << "\n";
    std::cout << "xy and xz are parallel: " << xy.is_parallel_to(xz) << "\n";
    if (xy.distance_between(z4)) {
        const Point3 foot{0.0, 0.0, 0.0};
        const Point3 raised = z4.closest_point_to(foot);
        demo_view::Scene scene;
        scene.add_plane(xy, foot, 1.7, kBlue, 0.5);
        scene.add_plane(z4, raised, 1.7, kOrange, 0.5);
        const Vec3 rise = raised - foot;
        const Vec3 rise_dir = rise / rise.length();
        scene.add_segment(foot + rise_dir * 0.14, raised - rise_dir * 0.14, kGreen, 0.035);
        scene.add_point(foot, kYellow, 0.1);
        scene.add_point(raised, kYellow, 0.1);
        show(scene, "planes_parallel", "Parallel planes",
             "xy and z = 4 are parallel. The green connector is the distance between them.");
    }

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
    {
        demo_view::Scene scene;
        scene.add_sphere(sphere, kBlue, 0.55);
        scene.add_point(sphere.center(), kYellow, 0.045);
        const auto marker = [&](const Point3& point, demo_view::Rgb color) {
            const Vec3 outward = geo::normalized(point - sphere.center());
            scene.add_point(sphere.center() + outward * (sphere.radius() * 1.04), color, 0.07);
        };
        marker(a, kWhite);
        marker(b, kRed);
        marker(c, kGreen);
        marker(d, kOrange);
        show(scene, "sphere", "Sphere through four points",
             "Center (0.5, 0.5, 0.5), radius sqrt(3)/2. The yellow point is the center.");
    }

    heading("Tetrahedron");
    const Tetrahedron3 tetrahedron = Tetrahedron3::from_points(a, b, c, d);
    std::cout << "From points (0, 0, 0), (1, 0, 0), (0, 1, 0), (0, 0, 1)\n";
    std::cout << "Centroid: ";
    print_point(tetrahedron.centroid());
    std::cout << "\nVolume: " << tetrahedron.volume() << "\n";
    std::cout << "Surface area: " << tetrahedron.surface_area() << "\n";
    {
        demo_view::Scene scene;
        scene.add_tetrahedron(tetrahedron, 0.48);
        scene.add_point(tetrahedron.centroid(), kYellow, 0.07);
        show(scene, "tetrahedron_points", "Tetrahedron from four points",
             "Centroid (0.25, 0.25, 0.25), volume 1/6. The yellow point is the centroid.");
    }

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
    {
        demo_view::Scene scene;
        const Point3 centroid = from_planes.centroid();
        scene.add_plane(face_x, face_x.closest_point_to(centroid), 0.95, kRed, 0.34);
        scene.add_plane(face_y, face_y.closest_point_to(centroid), 0.95, kGreen, 0.34);
        scene.add_plane(face_z, face_z.closest_point_to(centroid), 0.95, kBlue, 0.34);
        scene.add_plane(face_diag, face_diag.closest_point_to(centroid), 0.95, kOrange, 0.34);
        const Point3 vertex[4] = {from_planes.vertex(0), from_planes.vertex(1), from_planes.vertex(2),
                                   from_planes.vertex(3)};
        const int corner[6][2] = {{0, 1}, {0, 2}, {0, 3}, {1, 2}, {1, 3}, {2, 3}};
        for (const auto& edge : corner) {
            scene.add_segment(vertex[edge[0]], vertex[edge[1]], kWhite, 0.02);
        }
        for (const Point3& point : vertex) {
            scene.add_point(point, kYellow, 0.055);
        }
        show(scene, "tetrahedron_planes", "Tetrahedron from four planes",
             "Each vertex is the intersection of the other three face planes.");
    }

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
    {
        demo_view::Scene scene;
        const Point3 centroid = from_lines.centroid();
        for (const Line3& edge : edges) {
            const double middle = edge.parameter_of(centroid);
            scene.add_segment(edge.point_at(middle - 0.9), edge.point_at(middle + 0.9), kTeal, 0.028);
        }
        for (std::size_t index = 0; index < 4; ++index) {
            scene.add_point(from_lines.vertex(index), kYellow, 0.06);
        }
        show(scene, "tetrahedron_lines", "Tetrahedron from six lines",
             "The six edge lines. Opposite edges are skew, and the vertices are their intersections.");
    }

    if (!gallery.write_index()) {
        images_ok = false;
    }
    return images_ok ? 0 : 1;
}
