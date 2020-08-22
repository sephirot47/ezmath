#pragma once

#include "ez/BinaryIndex.h"
#include "ez/IntersectMode.h"
#include "ez/MathCommon.h"
#include "ez/MathForward.h"
#include "ez/MathInitializers.h"
#include "ez/Quat.h"
#include "ez/Vec.h"
#include <array>
#include <optional>

namespace ez
{
template <typename T, std::size_t N>
class HyperSphere final
{
public:
  using ValueType = T;
  static constexpr auto NumComponents = N;
  static constexpr auto NumDimensions = N;

  HyperSphere() = default;
  HyperSphere(const Vec<T, N>& inCenter, const T& inRadius) : mCenter(inCenter), mRadius(inRadius) {}

  void SetCenter(const Vec<T, N>& inCenter) { mCenter = inCenter; }
  void SetRadius(const T& inRadius) { mRadius = inRadius; }

  const Vec<T, N>& GetCenter() const { return mCenter; }
  const T& GetRadius() const { return mRadius; }

private:
  Vec<T, N> mCenter = Zero<Vec<T, N>>();
  T mRadius = static_cast<T>(0);
};

// Traits
template <typename T, std::size_t N>
struct IsHyperSphere<HyperSphere<T, N>> : std::true_type
{
};

// Intersection functions
template <EIntersectMode TIntersectMode, typename T, std::size_t N>
auto Intersect(const HyperSphere<T, N>& inHyperSphereLHS, const HyperSphere<T, N>& inHyperSphereRHS);

template <EIntersectMode TIntersectMode, typename T, std::size_t N>
auto Intersect(const HyperSphere<T, N>& inHyperSphere, const Vec<T, N>& inPoint);

template <EIntersectMode TIntersectMode, typename T, std::size_t N>
auto Intersect(const HyperSphere<T, N>& inHyperSphere, const Line<T, N>& inLine);

template <EIntersectMode TIntersectMode, typename T, std::size_t N>
auto Intersect(const HyperSphere<T, N>& inHyperSphere, const Ray<T, N>& inRay);

template <EIntersectMode TIntersectMode, typename T, std::size_t N>
auto Intersect(const HyperSphere<T, N>& inHyperSphere, const Segment<T, N>& inSegment);

template <EIntersectMode TIntersectMode, typename T, std::size_t N>
auto Intersect(const HyperSphere<T, N>& inHyperSphere, const AAHyperBox<T, N>& inAAHyperBox);

template <EIntersectMode TIntersectMode, typename T, std::size_t N>
auto Intersect(const HyperSphere<T, N>& inHyperSphere, const Capsule<T, N>& inCapsule);

template <EIntersectMode TIntersectMode, typename T, std::size_t N>
auto Intersect(const HyperSphere<T, N>& inHyperSphere, const Triangle<T, N>& inTriangle);

template <typename T, std::size_t N>
bool Contains(const HyperSphere<T, N>& inHyperSphere, const Vec<T, N>& inPoint);

template <typename T, std::size_t N>
bool Contains(const HyperSphere<T, N>& inHyperSphere, const Line<T, N>& inLine);

template <typename T, std::size_t N>
bool Contains(const HyperSphere<T, N>& inHyperSphere, const Ray<T, N>& inRay);

template <typename T, std::size_t N>
bool Contains(const HyperSphere<T, N>& inHyperSphere, const Segment<T, N>& inSegment);

template <typename T, std::size_t N>
bool Contains(const HyperSphere<T, N>& inHyperSphereContainer, const HyperSphere<T, N>& inHyperSphereContainee);

template <typename T, std::size_t N>
bool Contains(const HyperSphere<T, N>& inHyperSphere, const AAHyperBox<T, N>& inAAHyperBox);

template <typename T, std::size_t N>
bool Contains(const HyperSphere<T, N>& inHyperSphere, const HyperBox<T, N>& inHyperBox);

template <typename T, std::size_t N>
bool Contains(const HyperSphere<T, N>& inHyperSphere, const Capsule<T, N>& inCapsule);

template <typename T, std::size_t N>
bool Contains(const HyperSphere<T, N>& inHyperSphere, const Triangle<T, N>& inTriangle);

template <typename T, std::size_t N>
constexpr HyperSphere<T, N> Translated(const HyperSphere<T, N>& inHyperSphere, const Vec<T, N>& inTranslation);

template <typename T, std::size_t N>
constexpr HyperSphere<T, N> Rotated(const HyperSphere<T, N>& inHyperSphere, const RotationType_t<T, N>& inRotation);

template <typename T, std::size_t N>
constexpr Vec<T, N> Center(const HyperSphere<T, N>& inHyperSphere);

template <typename T, std::size_t N>
AAHyperBox<T, N> BoundingAAHyperBox(const HyperSphere<T, N>& inHyperSphere);
}

#include "ez/HyperSphere.tcc"
