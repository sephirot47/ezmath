#include "ez/Capsule.h"

namespace ez
{
template <typename T, std::size_t N>
Capsule<T, N>::Capsule(const Vec<T, N>& inOrigin, const Vec<T, N>& inDestiny, const T inRadius)
    : mOrigin { inOrigin }, mDestiny { inDestiny }, mRadius { inRadius }
{
}

template <typename T, std::size_t N>
constexpr Vec<T, N> Direction(const Capsule<T, N>& inCapsule)
{
  return Direction(inCapsule.GetSegment());
}

template <typename T, std::size_t N>
constexpr T SqLength(const Capsule<T, N>& inCapsule)
{
  return SqLength(inCapsule.GetSegment());
}

template <EIntersectMode TIntersectMode, typename T, std::size_t N>
auto Intersect(const Capsule<T, N>& inLHS, const Capsule<T, N>& inRHS)
{
  // WARNING: Untested
  static_assert(TIntersectMode == EIntersectMode::ONLY_CHECK, "Unsupported EIntersectMode.");
  return SqDistance(inLHS.GetSegment(), inRHS.GetSegment()) <= (Sq(inLHS.GetRadius()) + Sq(inRHS.GetRadius()));
}

template <EIntersectMode TIntersectMode, typename T, std::size_t N>
auto Intersect(const Capsule<T, N>& inCapsule, const Sphere<T>& inSphere)
{
  const auto capsule_axis_segment = inCapsule.GetSegment();
  const float radius_sum = (inSphere.GetRadius() + inCapsule.GetRadius());
  return SqDistance(Center(inSphere), capsule_axis_segment) <= Sq(radius_sum);
}

template <EIntersectMode TIntersectMode, typename T, std::size_t N>
auto Intersect(const Sphere<T>& inSphere, const Capsule<T, N>& inCapsule)
{
  return Intersect<TIntersectMode, T>(inCapsule, inSphere);
}

template <EIntersectMode TIntersectMode, typename T, std::size_t N>
auto Intersect(const Capsule<T, N>& inCapsule, const AACube<T>& inAACube)
{
  // WARNING: This is just a conservative approximation (will give false positive, but never false negatives)
  static_assert(TIntersectMode == EIntersectMode::ONLY_CHECK, "Unsupported EIntersectMode.");

  constexpr auto sqrt3_and_a_bit_more = Sqrt(static_cast<T>(3)) + static_cast<T>(1e-3);
  const auto aacube_half_size = (inAACube.GetSize() / static_cast<T>(2));
  const auto aacube_center = (inAACube.GetMin() + aacube_half_size);
  const auto aacube_half_diagonal = (aacube_half_size * sqrt3_and_a_bit_more);
  const auto aacube_sphere = Sphere<T> { aacube_center, aacube_half_diagonal };
  return IntersectCheck(aacube_sphere, inCapsule);
}

template <EIntersectMode TIntersectMode, typename T, std::size_t N>
auto Intersect(const AACube<T>& inAACube, const Capsule<T, N>& inCapsule)
{
  return Intersect<TIntersectMode, T>(inCapsule, inAACube);
}

template <typename T, std::size_t N>
constexpr bool Contains(const Capsule<T, N>& inCapsule, const Vec<T, N>& inPoint)
{
  return SqDistance(inPoint, inCapsule.GetSegment()) < Sq(inCapsule.GetRadius());
}

template <typename T, std::size_t N>
constexpr RotationType_t<T, N> Orientation(const Capsule<T, N>& inCapsule)
{
  const auto direction = Direction(inCapsule);
  if (SqLength(direction) == static_cast<T>(0))
  {
    if constexpr (N == 3)
      return Identity<Quat<T>>();
    else
      return static_cast<T>(0);
  }

  if constexpr (N == 3)
    return FromTo(Forward<Vec<T, N>>(), direction);
  else
    return std::atan2(direction[1], direction[0]);
}

template <typename T, std::size_t N>
constexpr Vec<T, N> Center(const Capsule<T, N>& inCapsule)
{
  return (inCapsule.GetOrigin() + inCapsule.GetDestiny()) / static_cast<T>(2);
}

}