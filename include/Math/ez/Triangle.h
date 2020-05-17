#pragma once

#include "ez/IntersectMode.h"
#include "ez/Vec.h"
#include <array>

namespace ez
{
template <typename T, std::size_t N>
class Triangle final
{
public:
  using ValueType = T;
  static constexpr auto NumDimensions = N;

  Triangle() = default;
  Triangle(const Vec<T, N>& inPoint0, const Vec<T, N>& inPoint1, const Vec<T, N>& inPoint2)
      : mPoints { inPoint0, inPoint1, inPoint2 }
  {
  }
  Triangle(const Triangle&) = default;
  Triangle& operator=(const Triangle&) = default;
  Triangle(Triangle&&) = default;
  Triangle& operator=(Triangle&&) = default;
  ~Triangle() = default;

  typename std::array<Vec<T, N>, 3>::iterator begin() { return mPoints.begin(); }
  typename std::array<Vec<T, N>, 3>::iterator end() { return mPoints.end(); }
  typename std::array<Vec<T, N>, 3>::const_iterator begin() const { return mPoints.begin(); }
  typename std::array<Vec<T, N>, 3>::const_iterator end() const { return mPoints.end(); }
  typename std::array<Vec<T, N>, 3>::const_iterator cbegin() const { return mPoints.cbegin(); }
  typename std::array<Vec<T, N>, 3>::const_iterator cend() const { return mPoints.cend(); }
  const Vec<T, N>* data() const { return mPoints.data(); }
  std::size_t size() const { return mPoints.size(); }

  std::array<Vec<T, N>, 3>& GetPoints() { return mPoints; }
  const std::array<Vec<T, N>, 3>& GetPoints() const { return mPoints; }

  Vec<T, N>& operator[](const std::size_t inPointIndex) { return mPoints[inPointIndex]; }
  const Vec<T, N>& operator[](const std::size_t inPointIndex) const { return mPoints[inPointIndex]; }

private:
  std::array<Vec<T, N>, 3> mPoints;
};

template <typename T>
using Triangle2 = Triangle<T, 2>;
using Triangle2f = Triangle2<float>;

template <typename T>
using Triangle3 = Triangle<T, 3>;
using Triangle3f = Triangle3<float>;

template <typename T, std::size_t N>
class Ray;

// Intersection functions
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