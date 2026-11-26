#pragma once

#include "geo/plane3.hpp"

#include <array>
#include <cmath>
#include <optional>
#include <stdexcept>
#include <vector>

namespace geo {

// Tetrahedron with four vertices. Volume is the absolute volume.
class Tetrahedron3 {
public:
    // Throws std::invalid_argument if the points are coplanar.
    Tetrahedron3(Point3 a, Point3 b, Point3 c, Point3 d);

    [[nodiscard]] static Tetrahedron3 from_points(const Point3& a,
                                                   const Point3& b,
                                                   const Point3& c,
                                                   const Point3& d);

    // Face planes. Each vertex is the intersection of the three planes opposite that vertex's slot.
    // Throws std::invalid_argument if the planes do not bound a tetrahedron.
    [[nodiscard]] static Tetrahedron3 from_planes(const Plane3& face0,
                                                   const Plane3& face1,
                                                   const Plane3& face2,
                                                   const Plane3& face3);

    // Six lines that contain the edges. Opposite edges may be skew.
    // Throws std::invalid_argument if those lines are not the edges of one tetrahedron.
    [[nodiscard]] static Tetrahedron3 from_lines(const std::array<Line3, 6>& edges,
                                                  double tolerance = default_tolerance);

    [[nodiscard]] Point3 vertex(std::size_t index) const;
    [[nodiscard]] Point3 centroid() const noexcept;
    [[nodiscard]] double volume() const noexcept;
    [[nodiscard]] double surface_area() const noexcept;

private:
    [[nodiscard]] double scalar_triple() const noexcept;
    [[nodiscard]] static double triangle_area(const Point3& a, const Point3& b, const Point3& c) noexcept;

    std::array<Point3, 4> vertices_;
};

inline Tetrahedron3::Tetrahedron3(Point3 a, Point3 b, Point3 c, Point3 d) : vertices_{a, b, c, d} {
    const double scale = (b - a).length() + (c - a).length() + (d - a).length();
    const double triple = scalar_triple();
    if (!std::isfinite(triple) || !(std::fabs(triple) > default_tolerance * scale * scale * scale)) {
        throw std::invalid_argument("Tetrahedron3 requires four non-coplanar points");
    }
}

inline Tetrahedron3 Tetrahedron3::from_points(const Point3& a,
                                               const Point3& b,
                                               const Point3& c,
                                               const Point3& d) {
    return Tetrahedron3{a, b, c, d};
}

inline Tetrahedron3 Tetrahedron3::from_planes(const Plane3& face0,
                                               const Plane3& face1,
                                               const Plane3& face2,
                                               const Plane3& face3) {
    const std::optional<Point3> v0 = intersect(face1, face2, face3);
    const std::optional<Point3> v1 = intersect(face0, face2, face3);
    const std::optional<Point3> v2 = intersect(face0, face1, face3);
    const std::optional<Point3> v3 = intersect(face0, face1, face2);
    if (!v0 || !v1 || !v2 || !v3) {
        throw std::invalid_argument("Tetrahedron3::from_planes requires four planes that meet at four vertices");
    }
    try {
        return Tetrahedron3{*v0, *v1, *v2, *v3};
    } catch (const std::invalid_argument&) {
        throw std::invalid_argument("Tetrahedron3::from_planes requires four planes that meet at four vertices");
    }
}

inline Tetrahedron3 Tetrahedron3::from_lines(const std::array<Line3, 6>& edges, double tolerance) {
    struct Cluster {
        Point3 point;
    };
    std::vector<Cluster> clusters;
    for (std::size_t i = 0; i < edges.size(); ++i) {
        for (std::size_t j = i + 1; j < edges.size(); ++j) {
            const LineLineIntersection hit = intersect(edges[i], edges[j], tolerance);
            if (hit.relation != LineLineRelation::Point || !hit.point) {
                continue;
            }
            bool merged = false;
            for (const Cluster& cluster : clusters) {
                if (cluster.point.distance_to(*hit.point) <= tolerance) {
                    merged = true;
                    break;
                }
            }
            if (!merged) {
                clusters.push_back(Cluster{*hit.point});
            }
        }
    }

    std::array<Point3, 4> vertices{};
    std::size_t vertex_count = 0;
    for (const Cluster& cluster : clusters) {
        int lines_through_point = 0;
        for (const Line3& edge : edges) {
            if (edge.distance_to(cluster.point) <= tolerance) {
                ++lines_through_point;
            }
        }
        if (lines_through_point < 3) {
            continue;
        }
        if (vertex_count >= vertices.size()) {
            throw std::invalid_argument("Tetrahedron3::from_lines requires the six edges of one tetrahedron");
        }
        vertices[vertex_count] = cluster.point;
        ++vertex_count;
    }
    if (vertex_count != vertices.size()) {
        throw std::invalid_argument("Tetrahedron3::from_lines requires the six edges of one tetrahedron");
    }

    for (const Line3& edge : edges) {
        int vertices_on_line = 0;
        for (const Point3& vertex : vertices) {
            if (edge.distance_to(vertex) <= tolerance) {
                ++vertices_on_line;
            }
        }
        if (vertices_on_line != 2) {
            throw std::invalid_argument("Tetrahedron3::from_lines requires the six edges of one tetrahedron");
        }
    }

    try {
        return Tetrahedron3{vertices[0], vertices[1], vertices[2], vertices[3]};
    } catch (const std::invalid_argument&) {
        throw std::invalid_argument("Tetrahedron3::from_lines requires the six edges of one tetrahedron");
    }
}

inline Point3 Tetrahedron3::vertex(std::size_t index) const {
    if (index >= vertices_.size()) {
        throw std::out_of_range("Tetrahedron3 vertex index");
    }
    return vertices_[index];
}

inline Point3 Tetrahedron3::centroid() const noexcept {
    const Vec3 sum = vertices_[0].to_vec3() + vertices_[1].to_vec3() + vertices_[2].to_vec3() +
                     vertices_[3].to_vec3();
    return Point3::from_vec3(sum * 0.25);
}

inline double Tetrahedron3::scalar_triple() const noexcept {
    return dot(vertices_[1] - vertices_[0], cross(vertices_[2] - vertices_[0], vertices_[3] - vertices_[0]));
}

inline double Tetrahedron3::volume() const noexcept { return std::fabs(scalar_triple()) / 6.0; }

inline double Tetrahedron3::triangle_area(const Point3& a, const Point3& b, const Point3& c) noexcept {
    return 0.5 * cross(b - a, c - a).length();
}

inline double Tetrahedron3::surface_area() const noexcept {
    return triangle_area(vertices_[0], vertices_[1], vertices_[2]) +
           triangle_area(vertices_[0], vertices_[1], vertices_[3]) +
           triangle_area(vertices_[0], vertices_[2], vertices_[3]) +
           triangle_area(vertices_[1], vertices_[2], vertices_[3]);
}

}  // namespace geo
