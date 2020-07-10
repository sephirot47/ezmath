#pragma once

#include "ez/AAHyperBox.h"
#include "ez/IntersectMode.h"
#include "ez/Plane.h"
#include <array>
#include <optional>

namespace ez
{
// Intersection functions
template <EIntersectMode TIntersectMode, typename T>
auto Intersect(const Ray<T, 3>& inRay, const AABox<T>& inAABox)
{
  static_assert(TIntersectMode == EIntersectMode::ALL_INTERSECTIONS || TIntersectMode == EIntersectMode::ONLY_CLOSEST
          || TIntersectMode == EIntersectMode::ONLY_CHECK,
      "Unsupported EIntersectMode.");

  constexpr auto BoxFaceNormals = std::array {
    // Order important for loop below.
    Left<Vec3<T>>(),
    Down<Vec3<T>>(),
    Forward<Vec3<T>>(),
    Right<Vec3<T>>(),
    Up<Vec3<T>>(),
    Back<Vec3<T>>(),
  };

  const auto aabox_size = inAABox.GetSize();
  const auto ray_is_inside = Contains(inAABox, inRay.GetOrigin());
  UNUSED(ray_is_inside);

  auto intersections_found = 0;                           // Only for EIntersectMode::ALL_INTERSECTIONS
  std::array<std::optional<T>, 2> intersection_distances; // Only for EIntersectMode::ALL_INTERSECTIONS
  if (ray_is_inside)
  {
    if constexpr (TIntersectMode == EIntersectMode::ONLY_CLOSEST)
    {
      return std::make_optional(static_cast<T>(0));
    }
    else if constexpr (TIntersectMode == EIntersectMode::ONLY_CHECK)
    {
      return true;
    }
  }

  for (std::size_t i = 0; i < BoxFaceNormals.size(); ++i) // For each box face
  {
    // Check whether it intersects the plane
    const auto& aabox_face_normal = BoxFaceNormals[i];
    if constexpr (TIntersectMode != EIntersectMode::ALL_INTERSECTIONS)
    {
      // If we are not looking for all intersections, it suffices with looking at the necessarily closest faces, which
      // are the ones counter-facing the ray direction
      if (Dot(aabox_face_normal, inRay.GetDirection()) > 0)
        continue;
    }

    const auto& aabox_face_point = (i < 3) ? inAABox.GetMin() : inAABox.GetMax();
    const auto aabox_face_plane = Planef(aabox_face_normal, aabox_face_point);

    if (const auto intersection_distance = IntersectClosest(inRay, aabox_face_plane))
    {
      // If it intersects the plane, then check if the intersection point is inside the box
      const auto intersection_point = inRay.GetPoint(*intersection_distance);

      auto aabox_face_normal_projector = One<Vec3<T>>();
      aabox_face_normal_projector[i % 3] = static_cast<T>(0);

      const auto reprojected_intersection_point = aabox_face_normal_projector * intersection_point;
      const auto reprojected_box_face_min = aabox_face_normal_projector * inAABox.GetMin();
      const auto reprojected_box_face_max = aabox_face_normal_projector * inAABox.GetMax();

      const auto intersection_point_contained_in_aabox_face
          = IsBetween(reprojected_intersection_point, reprojected_box_face_min, reprojected_box_face_max);

      if (!intersection_point_contained_in_aabox_face)
        continue;

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
        return true;
      }
    }
  }

  if constexpr (TIntersectMode == EIntersectMode::ALL_INTERSECTIONS)
  {
    return intersection_distances;
  }
  else if constexpr (TIntersectMode == EIntersectMode::ONLY_CLOSEST)
  {
    return std::optional<T> {};
  }
  else if constexpr (TIntersectMode == EIntersectMode::ONLY_CHECK)
  {
    return false;
  }
}

template <EIntersectMode TIntersectMode, typename T>
auto Intersect(const AABox<T>& inAABox, const Ray<T, 3>& inRay)
{
  return Intersect<TIntersectMode>(inRay, inAABox);
}

template <typename T>
constexpr auto GetSATNormals(const AABox<T>&)
{
  return std::array { Right<Vec3<T>>(), Up<Vec3<T>>(), Forward<Vec3<T>>() };
}

template <typename T>
constexpr auto GetSATEdges(const AABox<T>&)
{
  return std::array { Right<Vec3<T>>(), Up<Vec3<T>>(), Forward<Vec3<T>>() };
}

template <typename T>
constexpr auto BoundingAABox(const T& inThingToBound)
{
  static_assert(NumDimensions_v<T> == 3);
  return BoundingAAHyperBox(inThingToBound);
}

template <typename T>
constexpr auto BoundingAABoxTransformed(const T& inThingToBound,
    const Transformation<ValueType_t<T>, NumDimensions_v<T>>& inTransformation)
{
  static_assert(NumDimensions_v<T> == 3);
  return BoundingAAHyperBoxTransformed(inThingToBound, inTransformation);
}

template <typename T>
constexpr auto BoundingAABoxInverseTransformed(const T& inThingToBound,
    const Transformation<ValueType_t<T>, NumDimensions_v<T>>& inTransformation)
{
  static_assert(NumDimensions_v<T> == 3);
  return BoundingAAHyperBoxInverseTransformed(inThingToBound, inTransformation);
}
};