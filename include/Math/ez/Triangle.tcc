#include "ez/Triangle.h"

namespace ez
{

template <typename T, std::size_t N>
T Perimeter(const Triangle<T, N>& inTriangle)
{
  const auto length_01 = Distance(inTriangle[0], inTriangle[1]);
  const auto length_02 = Distance(inTriangle[0], inTriangle[2]);
  const auto length_12 = Distance(inTriangle[1], inTriangle[2]);
  const auto perimeter = (length_01 + length_02 + length_12);
  return perimeter;
}

template <typename T, std::size_t N>
T Barycenter(const Triangle<T, N>& inTriangle)
{
  return (inTriangle[0] + inTriangle[1] + inTriangle[2]) / static_cast<T>(3);
}

template <typename T>
Vec3<T> Normal(const Triangle3<T>& inTriangle)
{
  const auto v01 = (inTriangle[0] - inTriangle[1]);
  const auto v21 = (inTriangle[2] - inTriangle[1]);
  return NormalizedSafe(Cross(v01, v21));
}

template <typename T>
Plane<T> GetPlane(const Triangle3<T>& inTriangle)
{
  const auto v10 = (inTriangle[0] - inTriangle[1]);
  const auto v12 = (inTriangle[2] - inTriangle[1]);
  const auto& plane_point = inTriangle[0];
  const auto plane_normal = NormalizedSafe(Cross(v10, v12));
  const auto triangle_plane = Plane<T>(plane_normal, plane_point);
  return triangle_plane;
}

template <typename T>
Vec3<T> Projected(const Vec3<T>& inPoint, const Triangle3<T>& inTriangle)
{
  const auto triangle_plane = GetPlane(inTriangle);
  return Projected(inPoint, triangle_plane);
}

template <typename T, std::size_t N>
Vec3<T> BarycentricCoordinates(const Triangle<T, N>& inTriangle, const Vec<T, N>& inPoint)
{
  const auto& t = inTriangle;
  const auto& p = inPoint;

  const auto v0 = (t[1] - t[0]);
  const auto v1 = (t[2] - t[0]);
  const auto v2 = (p - t[0]);
  const auto dot_00 = Dot(v0, v0);
  const auto dot_01 = Dot(v0, v1);
  const auto dot_11 = Dot(v1, v1);
  const auto dot_20 = Dot(v2, v0);
  const auto dot_21 = Dot(v2, v1);
  const auto denominator = dot_00 * dot_11 - dot_01 * dot_01;
  const auto inv_denominator = static_cast<T>(1) / denominator;
  const auto v = (dot_11 * dot_20 - dot_01 * dot_21) * inv_denominator;
  const auto w = (dot_00 * dot_21 - dot_01 * dot_20) * inv_denominator;
  const auto u = static_cast<T>(1) - v - w;

  return Vec3<T>(u, v, w);
}

template <typename T>
auto GetSATNormals(const Triangle3<T>& inTriangle)
{
  return std::array { Normal(inTriangle) };
}

template <typename T>
constexpr auto GetSATNormals(const Triangle2<T>&)
{
  return std::array<Vec2<T>, 0> {};
}

template <typename T, std::size_t N>
auto GetSATEdges(const Triangle<T, N>& inTriangle)
{
  return std::array { (inTriangle[0] - inTriangle[1]),
    (inTriangle[0] - inTriangle[2]),
    (inTriangle[1] - inTriangle[2]) };
}

template <EIntersectMode TIntersectMode, typename T>
auto Intersect(const Ray<T, 3>& inRay, const Triangle3<T>& inTriangle)
{
  static_assert(TIntersectMode == EIntersectMode::ALL_INTERSECTIONS || TIntersectMode == EIntersectMode::ONLY_CLOSEST
          || TIntersectMode == EIntersectMode::ONLY_CHECK,
      "Unsupported EIntersectMode.");

  const auto ray_plane_intersection_distance = IntersectClosest(inRay, GetPlane(inTriangle));
  if (!ray_plane_intersection_distance)
  {
    if constexpr (TIntersectMode == EIntersectMode::ONLY_CHECK)
    {
      return false;
    }
    else
    {
      return std::optional<T>();
    }
  }

  const auto ray_plane_intersection_point = inRay.GetPoint(*ray_plane_intersection_distance);
  const auto barycentric_coordinates = BarycentricCoordinates(inTriangle, ray_plane_intersection_point);
  const auto intersection_point_is_in_triangle = IsBetween(barycentric_coordinates, Zero<Vec3<T>>(), One<Vec3<T>>());
  if constexpr (TIntersectMode == EIntersectMode::ONLY_CHECK)
  {
    return intersection_point_is_in_triangle;
  }
  else
  {
    return intersection_point_is_in_triangle ? ray_plane_intersection_distance : std::optional<T>();
  }
}

template <typename T>
auto Intersect(const Triangle3<T>& inTriangle, const Ray<T, 3>& inRay)
{
  return Intersect(inRay, inTriangle);
}

}