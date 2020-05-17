#pragma once

#include "ez/MathForward.h"
#include "ez/AAHyperRectangle.h"

namespace ez
{
// Intersection functions
template <typename T>
constexpr auto GetSATNormals(const AARect<T>&)
{
  return std::array { Right<Vec3<T>>(), Up<Vec3<T>>() };
}

template <typename T>
constexpr auto GetSATEdges(const AARect<T>&)
{
  return std::array { Right<Vec2<T>>(), Up<Vec2<T>>() };
}

template <typename T>
constexpr auto BoundingAARect(const T& inThingToBound)
{
  static_assert(NumDimensions_v<T> == 2);
  return BoundingAAHyperRectangle(inThingToBound);
}
}
