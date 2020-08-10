#pragma once

#include <ez/IntersectMode.h>
#include <ez/Segment.h>
#include <ez/Vec.h>

namespace ez
{
template <typename T>
class Capsule
{
public:
  Capsule() = default;
  Capsule(const Vec3<T>& inOrigin, const Vec3<T>& inDestiny, const T inRadius);

  void SetOrigin(const Vec3<T>& inOrigin) { mOrigin = inOrigin; }
  void SetDestiny(const Vec3<T>& inDestiny) { mDestiny = inDestiny; }
  void SetRadius(const T inRadius) { mRadius = inRadius; }

  Segment3<T> GetSegment() const { return Segment3<T> { mOrigin, mDestiny }; }
  const Vec3<T>& GetOrigin() const { return mOrigin; }
  const Vec3<T>& GetDestiny() const { return mDestiny; }
  const T& GetRadius() const { return mRadius; }

private:
  Vec3<T> mOrigin { Zero<Vec3<T>>() };
  Vec3<T> mDestiny { One<Vec3<T>>() };
  T mRadius { static_cast<T>(1) };
};

// Traits
template <typename T>
struct IsCapsule<Capsule<T>> : std::true_type
{
};

template <typename T>
constexpr Vec3<T> Direction(const Capsule<T>& inCapsule);

// Intersection functions
template <EIntersectMode TIntersectMode, typename T>
auto Intersect(const Capsule<T>& inLHS, const Capsule<T>& inRHS);

template <EIntersectMode TIntersectMode, typename T>
auto Intersect(const Capsule<T>& inCapsule, const Sphere<T>& inSphere);

template <EIntersectMode TIntersectMode, typename T>
auto Intersect(const Sphere<T>& inSphere, const Capsule<T>& inCapsule);

template <EIntersectMode TIntersectMode, typename T>
auto Intersect(const Capsule<T>& inCapsule, const AACube<T>& inAACube);

template <EIntersectMode TIntersectMode, typename T>
auto Intersect(const AACube<T>& inAACube, const Capsule<T>& inCapsule);

template <typename T>
constexpr bool Contains(const Capsule<T>& inCapsule, const Vec3<T>& inPoint);

template <typename T>
constexpr RotationType_t<T, 3> Orientation(const Capsule<T>& inCapsule);

template <typename T>
constexpr Vec3<T> Center(const Capsule<T>& inCapsule);

}

#include "ez/Capsule.tcc"