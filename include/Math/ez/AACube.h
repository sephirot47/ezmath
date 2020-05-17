#pragma once

#include "ez/AAHyperRectangle.h"
#include "ez/IntersectMode.h"
#include <array>
#include <optional>

namespace ez
{
// Intersection functions
template <EIntersectMode TIntersectMode, typename T>
auto Intersect(const Ray<T, 3>& inRay, const AACube<T>& inAACube)
{
  static_assert(TIntersectMode == EIntersectMode::ALL_INTERSECTIONS || TIntersectMode == EIntersectMode::ONLY_CLOSEST
          || TIntersectMode == EIntersectMode::ONLY_CHECK,
      "Unsupported EIntersectMode.");

  constexpr auto CubeFaceNormals = std::array {
    // Order important for loop below.
    Left<Vec3<T>>(),
    Down<Vec3<T>>(),
    Forward<Vec3<T>>(),
    Right<Vec3<T>>(),
    Up<Vec3<T>>(),
    Back<Vec3<T>>(),
  };

  const auto aacube_size = inAACube.GetSize();

  auto intersections_found = 0;
  std::array<std::optional<T>, 2> intersection_distances;
  for (std::size_t i = 0; i < CubeFaceNormals.size(); ++i) // For each cube face
  {
    // Check whether it intersects the plane
    const auto& aacube_face_normal = CubeFaceNormals[i];
    const auto& aacube_face_point = (i < 3) ? inAACube.GetMin() : inAACube.GetMax();
    const auto aacube_face_plane = Planef(aacube_face_normal, aacube_face_point);
    if (const auto intersection_distance = Intersect(inRay, aacube_face_plane))
    {
      // If it intersects the plane, then check if the intersection point is inside the cube
      const auto intersection_point = inRay.GetPoint(*intersection_distance);

      auto aacube_face_normal_projector = One<Vec3<T>>();
      aacube_face_normal_projector[i % 3] = static_cast<T>(0);

      const auto reprojected_intersection_point = aacube_face_normal_projector * intersection_point;
      const auto reprojected_cube_face_min = aacube_face_normal_projector * inAACube.GetMin();
      const auto reprojected_cube_face_max = aacube_face_normal_projector * inAACube.GetMax();

      const auto intersection_point_contained_in_aacube_face
          = IsBetween(reprojected_intersection_point, reprojected_cube_face_min, reprojected_cube_face_max);

      if (intersection_point_contained_in_aacube_face)
      {
        if constexpr (TIntersectMode == EIntersectMode::ALL_INTERSECTIONS)
        {
          intersection_distances[intersections_found] = intersection_distance;

          ++intersections_found;
          if (intersections_found == 2)
            break;
        }
        else if constexpr (TIntersectMode == EIntersectMode::ONLY_CLOSEST)
        {
          return intersection_distance;
        }
        else if constexpr (TIntersectMode == EIntersectMode::ONLY_CHECK)
        {
          return intersection_distance;
        }
      }
    }
  }

  if constexpr (TIntersectMode == EIntersectMode::ALL_INTERSECTIONS)
  {
    return intersection_distances;
  }
  else if constexpr (TIntersectMode == EIntersectMode::ONLY_CLOSEST)
  {
    return std::nullopt;
  }
  else if constexpr (TIntersectMode == EIntersectMode::ONLY_CHECK)
  {
    return false;
  }
}

template <EIntersectMode TIntersectMode, typename T>
auto Intersect(const AACube<T>& inAACube, const Ray<T, 3>& inRay)
{
  return Intersect(inRay, inAACube);
}

template <typename T>
constexpr auto GetSATNormals(const AACube<T>&)
{
  return std::array { Right<Vec3<T>>(), Up<Vec3<T>>(), Forward<Vec3<T>>() };
}

template <typename T>
constexpr auto GetSATEdges(const AACube<T>&)
{
  return std::array { Right<Vec3<T>>(), Up<Vec3<T>>(), Forward<Vec3<T>>() };
}

template <typename T>
constexpr auto BoundingAACube(const T& inThingToBound)
{
  static_assert(NumDimensions_v<T> == 3);
  return BoundingAAHyperRectangle(inThingToBound);
}

};