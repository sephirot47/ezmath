#pragma once

#include <ez/IntersectMode.h>
#include <ez/Segment.h>
#include <ez/Vec.h>

namespace ez
{
template <typename T, std::size_t N>
class Capsule
{
public:
  using ValueType = T;
  static constexpr auto NumComponents = N;

  Capsule() = default;
  Capsule(const Vec<T, N>& inOrigin, const Vec<T, N>& inDestiny, const T inRadius);

  void SetOrigin(const Vec<T, N>& inOrigin) { mOrigin = inOrigin; }
  void SetDestiny(const Vec<T, N>& inDestiny) { mDestiny = inDestiny; }
  void SetRadius(const T inRadius) { mRadius = inRadius; }

  Segment<T, N> GetSegment() const { return Segment<T, N> { mOrigin, mDestiny }; }
  const Vec<T, N>& GetOrigin() const { return mOrigin; }
  const Vec<T, N>& GetDestiny() const { return mDestiny; }
  const T& GetRadius() const { return mRadius; }

private:
  Vec<T, N> mOrigin { Zero<Vec<T, N>>() };
  Vec<T, N> mDestiny { One<Vec<T, N>>() };
  T mRadius { static_cast<T>(1) };
};

// Traits
template <typename T, std::size_t N>
struct IsCapsule<Capsule<T, N>> : std::true_type
{
};

template <typename T, std::size_t N>
constexpr Vec<T, N> Direction(const Capsule<T, N>& inCapsule);

template <typename T, std::size_t N>
constexpr T SqLength(const Capsule<T, N>& inCapsule);

// Intersection functions
template <EIntersectMode TIntersectMode, typename T, std::size_t N>
auto Intersect(const Capsule<T, N>& inLHS, const Capsule<T, N>& inRHS);

template <EIntersectMode TIntersectMode, typename T, std::size_t N>
auto Intersect(const Capsule<T, N>& inCapsule, const Sphere<T>& inSphere);

template <EIntersectMode TIntersectMode, typename T, std::size_t N>
auto Intersect(const Sphere<T>& inSphere, const Capsule<T, N>& inCapsule);

template <EIntersectMode TIntersectMode, typename T, std::size_t N>
auto Intersect(const Capsule<T, N>& inCapsule, const AACube<T>& inAACube);

template <EIntersectMode TIntersectMode, typename T, std::size_t N>
auto Intersect(const AACube<T>& inAACube, const Capsule<T, N>& inCapsule);

template <typename T, std::size_t N>
constexpr bool Contains(const Capsule<T, N>& inCapsule, const Vec<T, N>& inPoint);

template <typename T, std::size_t N>
constexpr RotationType_t<T, N> Orientation(const Capsule<T, N>& inCapsule);

template <typename T, std::size_t N>
constexpr Vec<T, N> Center(const Capsule<T, N>& inCapsule);

}

#include "ez/Capsule.tcc"