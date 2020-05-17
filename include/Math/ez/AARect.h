#pragma once

#include "AAHyperRectangle.h"

namespace ez
{
template <typename T>
using AARect = AAHyperRectangle<T, 2>;

using AARecti = AARect<int>;
using AARectf = AARect<float>;
using AARectd = AARect<double>;

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

}
