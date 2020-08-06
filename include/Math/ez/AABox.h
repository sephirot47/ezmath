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

  const auto ray_direction_inverse = (static_cast<T>(1) / Direction(inRay));
  const auto tbot = ray_direction_inverse * (inAABox.GetMin() - inRay.GetOrigin());
  const auto ttop = ray_direction_inverse * (inAABox.GetMax() - inRay.GetOrigin());
  const auto tmin = Min(ttop, tbot);
  const auto tmax = Max(ttop, tbot);
  const auto enter = Max(tmin);
  const auto exit = Min(tmax);
  const auto intersects = ((exit >= 0) && (enter < exit));
  if constexpr (TIntersectMode == EIntersectMode::ALL_INTERSECTIONS)
  {
    std::array<std::optional<T>, 2> intersection_distances;
    if (intersects)
    {
      const auto min_max = std::minmax(enter, exit);
      intersection_distances.at(0)
          = (std::isinf(min_max.first) ? std::optional<T> {} : std::make_optional(min_max.first));
      intersection_distances.at(1)
          = (std::isinf(min_max.second) ? std::optional<T> {} : std::make_optional(min_max.second));
    }
    return intersection_distances;
  }
  else if constexpr (TIntersectMode == EIntersectMode::ONLY_CLOSEST)
  {
    if (!intersects)
      return std::optional<T> {};

    const auto min_dist = Min(enter, exit);
    return ((min_dist < 0.0f || std::isinf(min_dist)) ? std::optional<T> {} : std::make_optional(min_dist));
  }
  else if constexpr (TIntersectMode == EIntersectMode::ONLY_CHECK)
  {
    return intersects;
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