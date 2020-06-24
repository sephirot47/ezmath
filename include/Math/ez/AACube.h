#include "ez/AABox.h"
#include "ez/AAHyperCube.h"
#include "ez/MathForward.h"

namespace ez
{
// Intersection functions
template <EIntersectMode TIntersectMode, typename T>
auto Intersect(const Ray<T, 3>& inRay, const AACube<T>& inAACube)
{
  static_assert(TIntersectMode == EIntersectMode::ALL_INTERSECTIONS || TIntersectMode == EIntersectMode::ONLY_CLOSEST
          || TIntersectMode == EIntersectMode::ONLY_CHECK,
      "Unsupported EIntersectMode.");

  const auto aabox_min = Vec3<T> { inAACube.GetMin() };
  const auto aabox_size = All<Vec3<T>>(static_cast<T>(inAACube.GetSize()));
  const auto aabox = MakeAAHyperBoxFromMinSize(aabox_min, aabox_size);
  return Intersect<TIntersectMode, T>(inRay, aabox);
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
}