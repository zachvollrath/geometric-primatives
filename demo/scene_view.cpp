#include "scene_view.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <vector>

namespace demo_view {
namespace {

constexpr double kHitEpsilon = 1.0e-4;
constexpr double kShadowBias = 1.5e-3;

struct Ray {
    geo::Point3 origin;
    geo::Vec3 direction;
};

struct Hit {
    double t = 0.0;
    geo::Point3 position;
    geo::Vec3 normal;
    Rgb color;
    double alpha = 1.0;
};

geo::Vec3 toward(const geo::Point3& from, const geo::Point3& to) { return to - from; }

Rgb scale(Rgb color, double factor) {
    return {color.r * factor, color.g * factor, color.b * factor};
}

Rgb add(Rgb a, Rgb b) { return {a.r + b.r, a.g + b.g, a.b + b.b}; }

double clamp01(double value) {
    if (value < 0.0) {
        return 0.0;
    }
    if (value > 1.0) {
        return 1.0;
    }
    return value;
}

bool solve_quadratic(double a, double b, double c, double& t0, double& t1) {
    if (std::abs(a) < 1.0e-12) {
        return false;
    }
    const double discriminant = b * b - 4.0 * a * c;
    if (discriminant < 0.0) {
        return false;
    }
    const double root = std::sqrt(discriminant);
    const double q = b < 0.0 ? -0.5 * (b - root) : -0.5 * (b + root);
    if (std::abs(q) < 1.0e-15) {
        t0 = (-b - root) / (2.0 * a);
        t1 = (-b + root) / (2.0 * a);
    } else {
        t0 = q / a;
        t1 = c / q;
    }
    if (t0 > t1) {
        std::swap(t0, t1);
    }
    return true;
}

bool hit_sphere(const Ray& ray, const geo::Point3& center, double radius, double& t, geo::Vec3& normal) {
    const geo::Vec3 offset = ray.origin - center;
    const double b = 2.0 * geo::dot(offset, ray.direction);
    const double c = geo::dot(offset, offset) - radius * radius;
    double t0 = 0.0;
    double t1 = 0.0;
    if (!solve_quadratic(1.0, b, c, t0, t1)) {
        return false;
    }
    double chosen = t0;
    if (chosen <= kHitEpsilon) {
        chosen = t1;
    }
    if (chosen <= kHitEpsilon) {
        return false;
    }
    t = chosen;
    const geo::Point3 position = ray.origin + ray.direction * t;
    normal = geo::normalized(position - center);
    return true;
}

bool hit_triangle(const Ray& ray,
                  const geo::Point3& a,
                  const geo::Point3& b,
                  const geo::Point3& c,
                  double& t,
                  geo::Vec3& normal) {
    const geo::Vec3 edge1 = b - a;
    const geo::Vec3 edge2 = c - a;
    const geo::Vec3 p = geo::cross(ray.direction, edge2);
    const double determinant = geo::dot(edge1, p);
    if (std::abs(determinant) < 1.0e-10) {
        return false;
    }
    const double inverse = 1.0 / determinant;
    const geo::Vec3 offset = ray.origin - a;
    const double u = geo::dot(offset, p) * inverse;
    if (u < 0.0 || u > 1.0) {
        return false;
    }
    const geo::Vec3 q = geo::cross(offset, edge1);
    const double v = geo::dot(ray.direction, q) * inverse;
    if (v < 0.0 || u + v > 1.0) {
        return false;
    }
    const double distance = geo::dot(edge2, q) * inverse;
    if (distance <= kHitEpsilon) {
        return false;
    }
    t = distance;
    normal = geo::normalized(geo::cross(edge1, edge2));
    return true;
}

bool hit_capsule(const Ray& ray,
                 const geo::Point3& start,
                 const geo::Point3& end,
                 double radius,
                 double& t,
                 geo::Vec3& normal) {
    bool found = false;
    double best = std::numeric_limits<double>::infinity();
    geo::Vec3 best_normal;

    const geo::Vec3 axis = end - start;
    const double axis_length_squared = geo::dot(axis, axis);
    if (axis_length_squared > 1.0e-12) {
        const geo::Vec3 offset = ray.origin - start;
        const double axis_dot_dir = geo::dot(axis, ray.direction);
        const double axis_dot_offset = geo::dot(axis, offset);
        const double a = axis_length_squared - axis_dot_dir * axis_dot_dir;
        const double b = axis_length_squared * geo::dot(offset, ray.direction) - axis_dot_offset * axis_dot_dir;
        const double c = axis_length_squared * geo::dot(offset, offset) - axis_dot_offset * axis_dot_offset -
                         radius * radius * axis_length_squared;
        const double discriminant = b * b - a * c;
        if (a > 1.0e-10 && discriminant >= 0.0) {
            const double root = std::sqrt(discriminant);
            const double candidates[2] = {(-b - root) / a, (-b + root) / a};
            for (double candidate : candidates) {
                if (candidate <= kHitEpsilon) {
                    continue;
                }
                const double along = axis_dot_offset + candidate * axis_dot_dir;
                if (along < 0.0 || along > axis_length_squared) {
                    continue;
                }
                if (candidate < best) {
                    best = candidate;
                    const geo::Point3 position = ray.origin + ray.direction * candidate;
                    const geo::Vec3 radial = toward(start, position) - axis * (along / axis_length_squared);
                    if (radial.length_squared() <= 1.0e-16) {
                        continue;
                    }
                    best_normal = geo::normalized(radial);
                    found = true;
                }
            }
        }
    }

    double sphere_t = 0.0;
    geo::Vec3 sphere_normal;
    if (hit_sphere(ray, start, radius, sphere_t, sphere_normal) && sphere_t < best) {
        best = sphere_t;
        best_normal = sphere_normal;
        found = true;
    }
    if (hit_sphere(ray, end, radius, sphere_t, sphere_normal) && sphere_t < best) {
        best = sphere_t;
        best_normal = sphere_normal;
        found = true;
    }
    if (!found) {
        return false;
    }
    t = best;
    normal = best_normal;
    return true;
}

bool closest_hit(const Ray& ray, const Scene::Primitive& primitive, Hit& hit) {
    double t = 0.0;
    geo::Vec3 normal;
    bool found = false;
    switch (primitive.shape) {
        case Scene::Primitive::Shape::Sphere:
            found = hit_sphere(ray, primitive.a, primitive.radius, t, normal);
            break;
        case Scene::Primitive::Shape::Triangle:
            found = hit_triangle(ray, primitive.a, primitive.b, primitive.c, t, normal);
            break;
        case Scene::Primitive::Shape::Capsule:
            found = hit_capsule(ray, primitive.a, primitive.b, primitive.radius, t, normal);
            break;
    }
    if (!found) {
        return false;
    }
    hit.t = t;
    hit.position = ray.origin + ray.direction * t;
    hit.normal = normal;
    hit.color = primitive.color;
    hit.alpha = primitive.alpha;
    return true;
}

Rgb background(const geo::Vec3& direction) {
    const double height = clamp01(0.5 * (direction.z + 1.0));
    const Rgb zenith{0.23, 0.30, 0.40};
    const Rgb ground{0.07, 0.08, 0.10};
    return {ground.r + (zenith.r - ground.r) * height, ground.g + (zenith.g - ground.g) * height,
            ground.b + (zenith.b - ground.b) * height};
}

Rgb shade(const Hit& hit, const geo::Vec3& ray_direction, const std::vector<Scene::Primitive>& primitives) {
    geo::Vec3 normal = hit.normal;
    if (geo::dot(normal, ray_direction) > 0.0) {
        normal = -normal;
    }
    const geo::Vec3 light = geo::normalized(geo::Vec3{0.45, 0.25, 0.86});
    double diffuse = std::max(0.0, geo::dot(normal, light));
    bool in_shadow = false;
    if (diffuse > 0.0 && hit.alpha > 0.85) {
        const Ray shadow{hit.position + normal * kShadowBias, light};
        for (const Scene::Primitive& primitive : primitives) {
            if (primitive.alpha < 0.85) {
                continue;
            }
            Hit blocked;
            if (closest_hit(shadow, primitive, blocked)) {
                in_shadow = true;
                break;
            }
        }
    }
    if (in_shadow) {
        diffuse = 0.0;
    }
    const double ambient = 0.22;
    Rgb lit = scale(hit.color, ambient + 0.78 * diffuse);
    if (!in_shadow) {
        const geo::Vec3 half_vector = geo::normalized(light - ray_direction);
        const double specular = std::pow(std::max(0.0, geo::dot(normal, half_vector)), 48.0) * 0.28;
        lit = add(lit, Rgb{specular, specular, specular});
    }
    return lit;
}

Rgb trace(const Ray& ray, const std::vector<Scene::Primitive>& primitives) {
    std::vector<Hit> hits;
    hits.reserve(primitives.size());
    for (const Scene::Primitive& primitive : primitives) {
        Hit hit;
        if (closest_hit(ray, primitive, hit)) {
            hits.push_back(hit);
        }
    }
    std::sort(hits.begin(), hits.end(), [](const Hit& lhs, const Hit& rhs) { return lhs.t < rhs.t; });

    Rgb accumulated{0.0, 0.0, 0.0};
    double remaining = 1.0;
    for (const Hit& hit : hits) {
        const Rgb lit = shade(hit, ray.direction, primitives);
        const double coverage = remaining * hit.alpha;
        accumulated = add(accumulated, scale(lit, coverage));
        remaining *= 1.0 - hit.alpha;
        if (remaining < 0.02) {
            break;
        }
    }
    return add(accumulated, scale(background(ray.direction), remaining));
}

unsigned char to_byte(double channel) {
    const double encoded = std::pow(clamp01(channel), 1.0 / 2.2);
    const int value = static_cast<int>(std::lround(encoded * 255.0));
    if (value <= 0) {
        return 0;
    }
    if (value >= 255) {
        return 255;
    }
    return static_cast<unsigned char>(value);
}

uint32_t crc_table[256];
bool crc_ready = false;

void ensure_crc() {
    if (crc_ready) {
        return;
    }
    for (uint32_t index = 0; index < 256; ++index) {
        uint32_t value = index;
        for (int bit = 0; bit < 8; ++bit) {
            if ((value & 1U) != 0U) {
                value = 0xEDB88320U ^ (value >> 1U);
            } else {
                value >>= 1U;
            }
        }
        crc_table[index] = value;
    }
    crc_ready = true;
}

uint32_t crc32(const unsigned char* data, std::size_t size, uint32_t seed = 0xFFFFFFFFU) {
    ensure_crc();
    uint32_t crc = seed;
    for (std::size_t index = 0; index < size; ++index) {
        const uint32_t byte = data[index];
        crc = crc_table[(crc ^ byte) & 0xFFU] ^ (crc >> 8U);
    }
    return crc;
}

void write_be32(std::ostream& out, uint32_t value) {
    const unsigned char bytes[4] = {static_cast<unsigned char>((value >> 24U) & 0xFFU),
                                     static_cast<unsigned char>((value >> 16U) & 0xFFU),
                                     static_cast<unsigned char>((value >> 8U) & 0xFFU),
                                     static_cast<unsigned char>(value & 0xFFU)};
    out.write(reinterpret_cast<const char*>(bytes), 4);
}

void write_chunk(std::ostream& out, const char* type, const std::vector<unsigned char>& data) {
    write_be32(out, static_cast<uint32_t>(data.size()));
    out.write(type, 4);
    if (!data.empty()) {
        out.write(reinterpret_cast<const char*>(data.data()), static_cast<std::streamsize>(data.size()));
    }
    uint32_t crc = crc32(reinterpret_cast<const unsigned char*>(type), 4);
    if (!data.empty()) {
        crc = crc32(data.data(), data.size(), crc);
    }
    write_be32(out, crc ^ 0xFFFFFFFFU);
}

std::string xml_escape(const std::string& text) {
    std::string escaped;
    escaped.reserve(text.size());
    for (char character : text) {
        switch (character) {
            case '&':
                escaped += "&amp;";
                break;
            case '<':
                escaped += "&lt;";
                break;
            case '>':
                escaped += "&gt;";
                break;
            default:
                escaped.push_back(character);
                break;
        }
    }
    return escaped;
}

}  // namespace

void Scene::include_point(const geo::Point3& point) {
    if (!has_bounds_) {
        bounds_min_ = point;
        bounds_max_ = point;
        has_bounds_ = true;
        return;
    }
    bounds_min_.x = std::min(bounds_min_.x, point.x);
    bounds_min_.y = std::min(bounds_min_.y, point.y);
    bounds_min_.z = std::min(bounds_min_.z, point.z);
    bounds_max_.x = std::max(bounds_max_.x, point.x);
    bounds_max_.y = std::max(bounds_max_.y, point.y);
    bounds_max_.z = std::max(bounds_max_.z, point.z);
}

void Scene::include_sphere(const geo::Point3& center, double radius) {
    include_point(center + geo::Vec3{radius, 0.0, 0.0});
    include_point(center + geo::Vec3{-radius, 0.0, 0.0});
    include_point(center + geo::Vec3{0.0, radius, 0.0});
    include_point(center + geo::Vec3{0.0, -radius, 0.0});
    include_point(center + geo::Vec3{0.0, 0.0, radius});
    include_point(center + geo::Vec3{0.0, 0.0, -radius});
}

void Scene::add_sphere(const geo::Sphere3& sphere, Rgb color, double alpha) {
    Primitive primitive;
    primitive.shape = Primitive::Shape::Sphere;
    primitive.a = sphere.center();
    primitive.radius = sphere.radius();
    primitive.color = color;
    primitive.alpha = alpha;
    primitives_.push_back(primitive);
    include_sphere(sphere.center(), sphere.radius());
}

void Scene::add_point(const geo::Point3& point, Rgb color, double radius) {
    Primitive primitive;
    primitive.shape = Primitive::Shape::Sphere;
    primitive.a = point;
    primitive.radius = radius;
    primitive.color = color;
    primitive.alpha = 1.0;
    primitives_.push_back(primitive);
    include_sphere(point, radius);
}

void Scene::add_segment(const geo::Point3& start, const geo::Point3& end, Rgb color, double radius) {
    Primitive primitive;
    primitive.shape = Primitive::Shape::Capsule;
    primitive.a = start;
    primitive.b = end;
    primitive.radius = radius;
    primitive.color = color;
    primitive.alpha = 1.0;
    primitives_.push_back(primitive);
    include_sphere(start, radius);
    include_sphere(end, radius);
}

void Scene::add_triangle(const geo::Point3& a,
                          const geo::Point3& b,
                          const geo::Point3& c,
                          Rgb color,
                          double alpha) {
    Primitive primitive;
    primitive.shape = Primitive::Shape::Triangle;
    primitive.a = a;
    primitive.b = b;
    primitive.c = c;
    primitive.color = color;
    primitive.alpha = alpha;
    primitives_.push_back(primitive);
    include_point(a);
    include_point(b);
    include_point(c);
}

void Scene::add_arrow(const geo::Point3& from,
                       const geo::Vec3& direction,
                       double length,
                       Rgb color,
                       double radius) {
    const double direction_length = direction.length();
    if (!(direction_length > 1.0e-8) || !(length > 1.0e-8)) {
        return;
    }
    const geo::Vec3 dir = direction / direction_length;
    const geo::Point3 tip = from + dir * length;
    const geo::Point3 neck = from + dir * (length * 0.74);
    add_segment(from, neck, color, radius);

    geo::Vec3 side = std::abs(dir.z) < 0.9 ? geo::cross(dir, geo::Vec3::unit_z())
                                            : geo::cross(dir, geo::Vec3::unit_x());
    side = geo::normalized(side);
    const geo::Vec3 side2 = geo::cross(dir, side);
    constexpr int kRing = 10;
    std::vector<geo::Point3> ring;
    ring.reserve(static_cast<std::size_t>(kRing));
    const double head_radius = radius * 2.7;
    for (int index = 0; index < kRing; ++index) {
        const double angle = 2.0 * geo::pi * static_cast<double>(index) / static_cast<double>(kRing);
        ring.push_back(neck + (side * std::cos(angle) + side2 * std::sin(angle)) * head_radius);
    }
    for (int index = 0; index < kRing; ++index) {
        add_triangle(tip, ring[static_cast<std::size_t>(index)],
                     ring[static_cast<std::size_t>((index + 1) % kRing)], color, 1.0);
    }
}

void Scene::add_plane(const geo::Plane3& plane,
                       const geo::Point3& center,
                       double half_extent,
                       Rgb color,
                       double alpha) {
    const geo::Vec3 normal = plane.normal();
    geo::Vec3 tangent = std::abs(normal.z) < 0.9 ? geo::cross(normal, geo::Vec3::unit_z())
                                                  : geo::cross(normal, geo::Vec3::unit_x());
    tangent = geo::normalized(tangent);
    const geo::Vec3 bitangent = geo::cross(normal, tangent);
    const geo::Point3 origin = plane.closest_point_to(center);
    const geo::Point3 p0 = origin - tangent * half_extent - bitangent * half_extent;
    const geo::Point3 p1 = origin + tangent * half_extent - bitangent * half_extent;
    const geo::Point3 p2 = origin + tangent * half_extent + bitangent * half_extent;
    const geo::Point3 p3 = origin - tangent * half_extent + bitangent * half_extent;
    add_triangle(p0, p1, p2, color, alpha);
    add_triangle(p0, p2, p3, color, alpha);
}

void Scene::add_tetrahedron(const geo::Tetrahedron3& tetrahedron, double alpha) {
    const geo::Point3 v0 = tetrahedron.vertex(0);
    const geo::Point3 v1 = tetrahedron.vertex(1);
    const geo::Point3 v2 = tetrahedron.vertex(2);
    const geo::Point3 v3 = tetrahedron.vertex(3);
    const Rgb faces[4] = {{0.86, 0.33, 0.28}, {0.26, 0.52, 0.86}, {0.24, 0.72, 0.42}, {0.93, 0.62, 0.18}};
    add_triangle(v0, v1, v2, faces[0], alpha);
    add_triangle(v0, v1, v3, faces[1], alpha);
    add_triangle(v0, v2, v3, faces[2], alpha);
    add_triangle(v1, v2, v3, faces[3], alpha);
    const Rgb edge{0.94, 0.95, 0.97};
    constexpr double kEdgeRadius = 0.028;
    add_segment(v0, v1, edge, kEdgeRadius);
    add_segment(v0, v2, edge, kEdgeRadius);
    add_segment(v0, v3, edge, kEdgeRadius);
    add_segment(v1, v2, edge, kEdgeRadius);
    add_segment(v1, v3, edge, kEdgeRadius);
    add_segment(v2, v3, edge, kEdgeRadius);
}

Scene::Image Scene::render(int width, int height) const {
    Image image;
    image.width = width;
    image.height = height;
    image.rgb.assign(static_cast<std::size_t>(width * height * 3), 0);
    if (!has_bounds_ || width <= 0 || height <= 0) {
        return image;
    }

    const geo::Point3 center{(bounds_min_.x + bounds_max_.x) * 0.5, (bounds_min_.y + bounds_max_.y) * 0.5,
                              (bounds_min_.z + bounds_max_.z) * 0.5};
    const double radius = std::max(0.5, 0.5 * (bounds_max_ - bounds_min_).length());
    // Look down from a three-quarter view so angles in the xy plane stay readable.
    const geo::Vec3 forward = geo::normalized(geo::Vec3{0.45, -0.40, -1.55});
    const double fov = 34.0 * geo::pi / 180.0;
    const double distance = radius / std::sin(fov * 0.5) * 1.28;
    const geo::Point3 eye = center - forward * distance;
    const geo::Vec3 world_up = std::abs(geo::dot(forward, geo::Vec3::unit_z())) > 0.95 ? geo::Vec3::unit_y()
                                                                                         : geo::Vec3::unit_z();
    const geo::Vec3 right = geo::normalized(geo::cross(forward, world_up));
    const geo::Vec3 up = geo::cross(right, forward);
    const double focal = std::tan(fov * 0.5);
    const double aspect = static_cast<double>(width) / static_cast<double>(height);
    const double sample_offsets[4][2] = {{-0.25, -0.25}, {0.25, -0.25}, {-0.25, 0.25}, {0.25, 0.25}};

    for (int row = 0; row < height; ++row) {
        for (int col = 0; col < width; ++col) {
            Rgb pixel{0.0, 0.0, 0.0};
            for (const auto& offset : sample_offsets) {
                const double px = (2.0 * (static_cast<double>(col) + 0.5 + offset[0]) / static_cast<double>(width) - 1.0) *
                                  aspect * focal;
                const double py = (1.0 - 2.0 * (static_cast<double>(row) + 0.5 + offset[1]) / static_cast<double>(height)) *
                                  focal;
                const Ray ray{eye, geo::normalized(forward + right * px + up * py)};
                pixel = add(pixel, trace(ray, primitives_));
            }
            pixel = scale(pixel, 0.25);
            const std::size_t index = static_cast<std::size_t>((row * width + col) * 3);
            image.rgb[index] = to_byte(pixel.r);
            image.rgb[index + 1] = to_byte(pixel.g);
            image.rgb[index + 2] = to_byte(pixel.b);
        }
    }
    return image;
}

bool write_png(const Scene::Image& image, const std::string& path) {
    if (image.width <= 0 || image.height <= 0 ||
        image.rgb.size() != static_cast<std::size_t>(image.width * image.height * 3)) {
        return false;
    }

    std::vector<unsigned char> raw;
    raw.reserve(static_cast<std::size_t>(image.height) * static_cast<std::size_t>(image.width * 3 + 1));
    for (int row = 0; row < image.height; ++row) {
        raw.push_back(0);
        const std::size_t begin = static_cast<std::size_t>(row * image.width * 3);
        raw.insert(raw.end(), image.rgb.begin() + static_cast<std::ptrdiff_t>(begin),
                   image.rgb.begin() + static_cast<std::ptrdiff_t>(begin + static_cast<std::size_t>(image.width * 3)));
    }

    std::vector<unsigned char> zlib;
    zlib.push_back(0x78);
    zlib.push_back(0x01);
    std::size_t cursor = 0;
    while (cursor < raw.size()) {
        const std::size_t remaining = raw.size() - cursor;
        const std::size_t block = std::min(remaining, static_cast<std::size_t>(65535));
        const bool last = cursor + block == raw.size();
        zlib.push_back(last ? 0x01 : 0x00);
        const auto length = static_cast<uint16_t>(block);
        const uint16_t negated = static_cast<uint16_t>(~length);
        zlib.push_back(static_cast<unsigned char>(length & 0xFFU));
        zlib.push_back(static_cast<unsigned char>((length >> 8U) & 0xFFU));
        zlib.push_back(static_cast<unsigned char>(negated & 0xFFU));
        zlib.push_back(static_cast<unsigned char>((negated >> 8U) & 0xFFU));
        zlib.insert(zlib.end(), raw.begin() + static_cast<std::ptrdiff_t>(cursor),
                    raw.begin() + static_cast<std::ptrdiff_t>(cursor + block));
        cursor += block;
    }
    uint32_t s1 = 1;
    uint32_t s2 = 0;
    for (unsigned char byte : raw) {
        s1 = (s1 + byte) % 65521U;
        s2 = (s2 + s1) % 65521U;
    }
    const uint32_t adler = (s2 << 16U) | s1;
    zlib.push_back(static_cast<unsigned char>((adler >> 24U) & 0xFFU));
    zlib.push_back(static_cast<unsigned char>((adler >> 16U) & 0xFFU));
    zlib.push_back(static_cast<unsigned char>((adler >> 8U) & 0xFFU));
    zlib.push_back(static_cast<unsigned char>(adler & 0xFFU));

    std::ofstream out(path, std::ios::binary);
    if (!out) {
        return false;
    }
    const unsigned char signature[8] = {137, 80, 78, 71, 13, 10, 26, 10};
    out.write(reinterpret_cast<const char*>(signature), 8);

    std::vector<unsigned char> ihdr(13);
    const auto width = static_cast<uint32_t>(image.width);
    const auto height = static_cast<uint32_t>(image.height);
    ihdr[0] = static_cast<unsigned char>((width >> 24U) & 0xFFU);
    ihdr[1] = static_cast<unsigned char>((width >> 16U) & 0xFFU);
    ihdr[2] = static_cast<unsigned char>((width >> 8U) & 0xFFU);
    ihdr[3] = static_cast<unsigned char>(width & 0xFFU);
    ihdr[4] = static_cast<unsigned char>((height >> 24U) & 0xFFU);
    ihdr[5] = static_cast<unsigned char>((height >> 16U) & 0xFFU);
    ihdr[6] = static_cast<unsigned char>((height >> 8U) & 0xFFU);
    ihdr[7] = static_cast<unsigned char>(height & 0xFFU);
    ihdr[8] = 8;
    ihdr[9] = 2;
    write_chunk(out, "IHDR", ihdr);
    write_chunk(out, "IDAT", zlib);
    write_chunk(out, "IEND", {});
    return static_cast<bool>(out);
}

Gallery::Gallery(std::string directory) : directory_(std::move(directory)) {}

bool Gallery::add(const Scene& scene,
                   const std::string& filename,
                   const std::string& title,
                   const std::string& caption) {
    std::error_code error;
    std::filesystem::create_directories(directory_, error);
    if (error) {
        std::cerr << "Could not create " << directory_ << ": " << error.message() << "\n";
        return false;
    }
    const std::filesystem::path path = std::filesystem::path(directory_) / (filename + ".png");
    std::cout << "Rendering " << path.string() << "...\n" << std::flush;
    const Scene::Image image = scene.render(640, 480);
    if (!write_png(image, path.string())) {
        std::cerr << "Could not write " << path.string() << "\n";
        return false;
    }
    shots_.push_back(Shot{filename + ".png", title, caption});
    return true;
}

bool Gallery::write_index() const {
    const std::filesystem::path path = std::filesystem::path(directory_) / "index.html";
    std::ofstream out(path);
    if (!out) {
        std::cerr << "Could not write " << path.string() << "\n";
        return false;
    }
    out << "<!DOCTYPE html>\n<html lang=\"en\">\n<head>\n<meta charset=\"utf-8\">\n"
        << "<title>Geometric primitives</title>\n<style>\n"
        << "body{font-family:sans-serif;background:#12141a;color:#e8e8ea;margin:2rem auto;max-width:720px;}\n"
        << "figure{margin:0 0 2.2rem;} img{width:100%;background:#000;} h1{font-weight:600;}\n"
        << "h2{font-size:1.05rem;margin:0 0 0.45rem;} figcaption{color:#c5c7ce;}\n"
        << "</style>\n</head>\n<body>\n<h1>Geometric primitives</h1>\n";
    for (const Shot& shot : shots_) {
        out << "<figure><h2>" << xml_escape(shot.title) << "</h2><img src=\"" << xml_escape(shot.filename)
            << "\" alt=\"" << xml_escape(shot.title) << "\"><figcaption>" << xml_escape(shot.caption)
            << "</figcaption></figure>\n";
    }
    out << "</body>\n</html>\n";
    if (!out) {
        return false;
    }
    std::cout << "Wrote " << path.string() << "\n";
    return true;
}

}  // namespace demo_view
