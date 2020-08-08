#pragma once

#include "ez/AAHyperBox.h"
#include "ez/IntersectMode.h"
#include "ez/Plane.h"
#include <array>
#include <optional>

namespace ez
{
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