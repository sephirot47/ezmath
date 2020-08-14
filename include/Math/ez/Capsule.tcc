#include "ez/Capsule.h"

namespace ez
{
template <typename T>
Capsule<T>::Capsule(const Vec3<T>& inOrigin, const Vec3<T>& inDestiny, const T inRadius)
    : mOrigin { inOrigin }, mDestiny { inDestiny }, mRadius { inRadius }
{
}

template <typename T>
constexpr Vec3<T> Direction(const Capsule<T>& inCapsule)
{
  return Direction(inCapsule.GetSegment());
}

template <EIntersectMode TIntersectMode, typename T>
auto Intersect(const Capsule<T>& inLHS, const Capsule<T>& inRHS)
{
  // WARNING: Untested
  static_assert(TIntersectMode == EIntersectMode::ONLY_CHECK, "Unsupported EIntersectMode.");
  return SqDistance(inLHS.GetSegment(), inRHS.GetSegment()) <= (Sq(inLHS.GetRadius()) + Sq(inRHS.GetRadius()));
}

template <EIntersectMode TIntersectMode, typename T>
auto Intersect(const Capsule<T>& inCapsule, const Sphere<T>& inSphere)
{
  const auto capsule_axis_segment = inCapsule.GetSegment();
  const float radius_sum = (inSphere.GetRadius() + inCapsule.GetRadius());
  return SqDistance(Center(inSphere), capsule_axis_segment) <= Sq(radius_sum);
}

template <EIntersectMode TIntersectMode, typename T>
auto Intersect(const Sphere<T>& inSphere, const Capsule<T>& inCapsule)
{
  return Intersect<TIntersectMode, T>(inCapsule, inSphere);
}

template <EIntersectMode TIntersectMode, typename T>
auto Intersect(const Capsule<T>& inCapsule, const AACube<T>& inAACube)
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

template <EIntersectMode TIntersectMode, typename T>
auto Intersect(const AACube<T>& inAACube, const Capsule<T>& inCapsule)
{
  return Intersect<TIntersectMode, T>(inCapsule, inAACube);
}

template <typename T>
constexpr bool Contains(const Capsule<T>& inCapsule, const Vec3<T>& inPoint)
{
  return SqDistance(inPoint, inCapsule.GetSegment()) < Sq(inCapsule.GetRadius());
}

template <typename T>
constexpr RotationType_t<T, 3> Orientation(const Capsule<T>& inCapsule)
{
  const auto direction = Direction(inCapsule);
  if (SqLength(direction) == static_cast<T>(0))
    return Identity<Quat<T>>();
  return FromTo(Forward<Vec3<T>>(), direction);
}

template <typename T>
constexpr Vec3<T> Center(const Capsule<T>& inCapsule)
{
  return (inCapsule.GetOrigin() + inCapsule.GetDestiny()) / static_cast<T>(2);
}

}