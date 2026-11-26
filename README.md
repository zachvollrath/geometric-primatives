# geometric-primatives

Header-only C++17 types for 3D points, lines, segments, planes, spheres, and tetrahedra.

```cpp
#include "geo/geo.hpp"
```

## Build

```bash
cmake -S . -B build
cmake --build build
ctest --test-dir build --output-on-failure
./build/geo_demo
```

`geo_demo` prints a worked example of each operation below and writes a ray-traced PNG for each one to `visualizations/`, along with `visualizations/index.html`.

## Types

| Type | Representation |
| --- | --- |
| `Point3` | Position `(x, y, z)` |
| `Vec3` | Free vector used for directions and normals |
| `Line3` | Infinite line `origin + t * direction`, with a unit direction |
| `Segment3` | Finite segment from `start` to `end` |
| `Plane3` | Hessian plane `normal · x = offset`, with a unit normal |
| `Sphere3` | Center and positive radius |
| `Tetrahedron3` | Four vertices |

Angles are in radians. Geometric tests use an absolute tolerance, `geo::default_tolerance` (`1e-9`), which callers can override. A zero direction, a zero normal, or a degenerate solid throws `std::invalid_argument`.

`Line3` is infinite, so it has no length. `Segment3` is the bounded segment and stores both endpoints.

## Operations

**Points**

- `distance_to` is the distance between two points.
- `direction_to` is the unit direction from one point to another.

**Segments**

- Construct a `Segment3` from its two endpoints.
- `intersect` returns the shared point, or the overlapping endpoints when the segments are collinear and overlap.
- `length` and `direction` are the segment length and unit direction.
- `closest_to` is the point on a segment nearest another point, with the distance.
- `closest_points_to` is the nearest pair of points on two segments, with the distance.
- `is_parallel_to` and `is_perpendicular_to` compare segment directions. Opposite directions count as parallel.

**Planes**

- `Plane3::from_points` builds a plane from three points. The normal follows the right-hand rule.
- `intersect` of three planes returns their common point.
- `closest_point_to` and `distance_to` project a point onto a plane.
- `intersect` of two planes returns their line.
- `angle_to` is the acute angle between two planes, in `[0, pi/2]`.
- `is_parallel_to` and `distance_between` report whether two planes are parallel and, when they are, the distance. Coincident planes are parallel and have distance 0.

**Sphere**

- `Sphere3::from_points` is the unique sphere through four non-coplanar points.
- `volume` is `(4/3) π r³` and `surface_area` is `4 π r²`.

**Tetrahedron**

- `Tetrahedron3::from_points` takes four non-coplanar points.
- `from_planes` takes the four face planes. Each vertex is the intersection of the other three planes.
- `from_lines` takes the six lines that contain the edges. Opposite edges may be skew.
- `centroid` is the average of the vertices.
- `volume` is the absolute volume, `|scalar triple| / 6`.
- `surface_area` is the sum of the four triangular face areas.
