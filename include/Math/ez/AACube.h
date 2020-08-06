#pragma once

#include "ez/AABox.h"
#include "ez/AAHyperCube.h"
#include "ez/MathForward.h"

namespace ez
{
// Intersection functions
template <EIntersectMode TIntersectMode, typename T>
auto Intersect(const Ray3<T>& inRay, const AACube<T>& inAACube)
{
  return Intersect<TIntersectMode, T>(inRay, MakeAAHyperBoxFromAAHyperCube(inAACube));
}

template <EIntersectMode TIntersectMode, typename T>
auto Intersect(const AACube<T>& inAACube, const Ray3<T>& inRay)
{
  return Intersect<TIntersectMode, T>(inRay, inAACube);
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
}