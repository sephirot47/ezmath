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
auto Intersect(const Capsule<T, N>& inCapsule, const Vec<T, N>& inPoint)
{
  return Intersect<TIntersectMode>(inPoint, inCapsule);
}

template <EIntersectMode TIntersectMode, typename T, std::size_t N>
auto Intersect(const Capsule<T, N>& inCapsule, const Line<T, N>& inLine)
{
  return Intersect<TIntersectMode>(inLine, inCapsule);
}

template <EIntersectMode TIntersectMode, typename T, std::size_t N>
auto Intersect(const Capsule<T, N>& inCapsule, const Ray<T, N>& inRay)
{
  return Intersect<TIntersectMode>(inRay, inCapsule);
}

template <EIntersectMode TIntersectMode, typename T, std::size_t N>
auto Intersect(const Capsule<T, N>& inCapsule, const Segment<T, N>& inSegment)
{
  return Intersect<TIntersectMode>(inSegment, inCapsule);
}

template <EIntersectMode TIntersectMode, typename T, std::size_t N>
auto Intersect(const Capsule<T, N>& inCapsule, const HyperSphere<T, N>& inHyperSphere)
{
  const auto capsule_axis_segment = inCapsule.GetSegment();
  const float radius_sum = (inHyperSphere.GetRadius() + inCapsule.GetRadius());
  return SqDistance(capsule_axis_segment, Center(inHyperSphere)) <= Sq(radius_sum);
}

template <EIntersectMode TIntersectMode, typename T, std::size_t N>
auto Intersect(const Capsule<T, N>& inCapsule, const AAHyperBox<T, N>& inAAHyperBox)
{
  return Intersect<TIntersectMode>(inAAHyperBox, inCapsule);
}

template <EIntersectMode TIntersectMode, typename T, std::size_t N>
auto Intersect(const Capsule<T, N>& inCapsule, const HyperBox<T, N>& inHyperBox)
{
  return Intersect<TIntersectMode>(inHyperBox, inCapsule);
}

template <EIntersectMode TIntersectMode, typename T, std::size_t N>
auto Intersect(const Capsule<T, N>& inCapsuleLHS, const Capsule<T, N>& inCapsuleRHS)
{
  static_assert(TIntersectMode == EIntersectMode::ONLY_CHECK, "Unsupported EIntersectMode.");
  return SqDistance(inCapsuleLHS.GetSegment(), inCapsuleRHS.GetSegment())
      <= (Sq(inCapsuleLHS.GetRadius() + inCapsuleRHS.GetRadius()));
}

template <typename T, std::size_t N>
constexpr Vec<T, N> ClosestPoint(const Capsule<T, N>& inCapsule, const Vec<T, N>& inPoint)
{
  const auto closest_point_in_segment = ClosestPoint(inCapsule.GetSegment(), inPoint);
  const auto closest_point_direction_from_segment = Direction(inPoint - closest_point_in_segment);
  const auto closest_point_in_capsule
      = closest_point_in_segment + (inCapsule.GetRadius() * closest_point_direction_from_segment);
  return closest_point_in_capsule;
}

template <typename T, std::size_t N>
constexpr Vec<T, N> ClosestPoint(const Capsule<T, N>& inCapsule, const AAHyperBox<T, N>& inAAHyperBox)
{
  return ClosestPoint(inCapsule, ClosestPoint(inAAHyperBox, inCapsule));
}

template <typename T, std::size_t N>
constexpr Vec<T, N> ClosestPoint(const Capsule<T, N>& inCapsule, const HyperBox<T, N>& inHyperBox)
{
  return ClosestPoint(inCapsule, ClosestPoint(inHyperBox, inCapsule));
}

template <typename T, std::size_t N>
constexpr bool Contains(const Capsule<T, N>& inCapsule, const Vec<T, N>& inPoint)
{
  return SqDistance(inCapsule.GetSegment(), inPoint) < Sq(inCapsule.GetRadius());
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
constexpr Capsule<T, N> Translated(const Capsule<T, N>& inCapsule, const Vec<T, N>& inTranslation)
{
  return Capsule<T, N> { inCapsule.GetOrigin() + inTranslation,
    inCapsule.GetDestiny() + inTranslation,
    inCapsule.GetRadius() };
}

template <typename T, std::size_t N>
constexpr Capsule<T, N> Rotated(const Capsule<T, N>& inCapsule, const RotationType_t<T, N>& inRotation)
{
  return Capsule<T, N> { Rotated(inCapsule.GetOrigin(), inRotation),
    Rotated(inCapsule.GetDestiny(), inRotation),
    inCapsule.GetRadius() };
}

template <typename T, std::size_t N>
constexpr Vec<T, N> Center(const Capsule<T, N>& inCapsule)
{
  return (inCapsule.GetOrigin() + inCapsule.GetDestiny()) / static_cast<T>(2);
}

}