#pragma once

#include "geo/geo.hpp"

#include <string>
#include <vector>

namespace demo_view {

struct Rgb {
    double r = 1.0;
    double g = 1.0;
    double b = 1.0;
};

// Ray-traced preview of the primitives used by geo_demo.
class Scene {
public:
    struct Primitive {
        enum class Shape { Sphere, Triangle, Capsule };

        Shape shape = Shape::Sphere;
        geo::Point3 a;
        geo::Point3 b;
        geo::Point3 c;
        double radius = 0.0;
        Rgb color;
        double alpha = 1.0;
    };

    struct Image {
        int width = 0;
        int height = 0;
        std::vector<unsigned char> rgb;
    };

    void add_sphere(const geo::Sphere3& sphere, Rgb color, double alpha = 1.0);
    void add_point(const geo::Point3& point, Rgb color, double radius = 0.08);
    void add_segment(const geo::Point3& start, const geo::Point3& end, Rgb color, double radius = 0.045);
    void add_arrow(const geo::Point3& from,
                    const geo::Vec3& direction,
                    double length,
                    Rgb color,
                    double radius = 0.05);
    void add_triangle(const geo::Point3& a,
                      const geo::Point3& b,
                      const geo::Point3& c,
                      Rgb color,
                      double alpha = 1.0);
    void add_plane(const geo::Plane3& plane,
                    const geo::Point3& center,
                    double half_extent,
                    Rgb color,
                    double alpha = 0.42);
    void add_tetrahedron(const geo::Tetrahedron3& tetrahedron, double alpha = 0.9);

    [[nodiscard]] Image render(int width, int height) const;

private:
    void include_point(const geo::Point3& point);
    void include_sphere(const geo::Point3& center, double radius);

    std::vector<Primitive> primitives_;
    geo::Point3 bounds_min_{};
    geo::Point3 bounds_max_{};
    bool has_bounds_ = false;
};

class Gallery {
public:
    explicit Gallery(std::string directory);

    // Renders immediately and writes directory/filename.png.
    [[nodiscard]] bool add(const Scene& scene,
                            const std::string& filename,
                            const std::string& title,
                            const std::string& caption);

    [[nodiscard]] bool write_index() const;

private:
    struct Shot {
        std::string filename;
        std::string title;
        std::string caption;
    };

    std::string directory_;
    std::vector<Shot> shots_;
};

[[nodiscard]] bool write_png(const Scene::Image& image, const std::string& path);

}  // namespace demo_view
