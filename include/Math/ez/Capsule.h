#pragma once

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
  T GetSqRadius() const { return Sq(mRadius); }

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

}

#include "ez/Capsule.tcc"