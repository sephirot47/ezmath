#pragma once

#include <ez/IntersectMode.h>
#include <ez/MathForward.h>
#include <ez/MathInitializers.h>
#include <ez/Vec.h>
#include <type_traits>

namespace ez
{
template <typename T, std::size_t N>
class Line
{
public:
  using ValueType = T;
  static constexpr auto NumComponents = N;

  Line() = default;
  Line(const Vec<T, N>& inOrigin, const Vec<T, N>& inDirection);

  const Vec<T, N>& GetOrigin() const { return mOrigin; }
  const Vec<T, N>& GetDirection() const { return mDirection; }
  Vec<T, N> GetPoint(const T& inDistance) const { return mOrigin + mDirection * inDistance; }

  void SetOrigin(const Vec<T, N>& inOrigin) { mOrigin = inOrigin; }
  void SetDirection(const Vec<T, N>& inDirection);

private:
  Vec<T, N> mOrigin = Zero<Vec<T, N>>();
  Vec<T, N> mDirection = Right<Vec<T, N>>();
};

template <typename T, std::size_t N>
struct IsLine<Line<T, N>> : std::true_type
{
};

template <typename T, std::size_t N>
constexpr Vec<T, N> Direction(const Line<T, N>& inLine);

template <typename T, std::size_t N>
constexpr RotationType_t<T, N> Orientation(const Line<T, N>& inLine);

template <typename T>
constexpr T ClosestPointT(const Line2<T>& inLineLHS, const Line2<T>& inLineRHS);

template <typename T>
constexpr T ClosestPointT(const Line3<T>& inLineLHS, const Line3<T>& inLineRHS);

template <typename T, std::size_t N>
constexpr T ClosestPointT(const Line<T, N>& inLine, const Vec<T, N>& inPoint);

template <typename T, std::size_t N>
constexpr T ClosestPointT(const Line<T, N>& inLine, const Ray<T, N>& inRay);

template <typename T, std::size_t N>
constexpr T ClosestPointT(const Line<T, N>& inLine, const Segment<T, N>& inSegment);

template <typename T, std::size_t N>
constexpr T ClosestPointT(const Line<T, N>& inLine, const AAHyperBox<T, N>& inAAHyperBox);

template <typename T, std::size_t N>
constexpr T ClosestPointT(const Line<T, N>& inLine, const HyperBox<T, N>& inHyperBox);

template <typename T, std::size_t N>
constexpr Vec<T, N> ClosestPoint(const Line<T, N>& inLine, const AAHyperBox<T, N>& inAAHyperBox);

template <typename T, std::size_t N>
constexpr Vec<T, N> ClosestPoint(const Line<T, N>& inLine, const HyperBox<T, N>& inHyperBox);

template <typename T, std::size_t N, typename TPrimitive>
constexpr Vec<T, N> ClosestPoint(const Line<T, N>& inLine, const TPrimitive& inPrimitive);

template <typename T, std::size_t N, typename TPrimitive>
constexpr T SqDistance(const Line<T, N>& inLine, const TPrimitive& inPrimitive);

template <EIntersectMode TIntersectMode, typename T>
auto Intersect(const Line2<T>& inLineLHS, const Line2<T>& inLineRHS);

template <EIntersectMode TIntersectMode, typename T, std::size_t N>
auto Intersect(const Line<T, N>& inLine, const Vec<T, N>& inPoint);

template <EIntersectMode TIntersectMode, typename T, std::size_t N>
auto Intersect(const Line<T, N>& inLine, const Ray<T, N>& inRay);

template <EIntersectMode TIntersectMode, typename T, std::size_t N>
auto Intersect(const Line<T, N>& inLine, const Segment<T, N>& inSegment);

template <EIntersectMode TIntersectMode, typename T>
auto Intersect(const Line3<T>& inLine, const Plane<T>& inPlane);

template <EIntersectMode TIntersectMode, typename T, std::size_t N>
auto Intersect(const Line<T, N>& inLine, const AAHyperBox<T, N>& inAAHyperBox);

template <EIntersectMode TIntersectMode, typename T, std::size_t N>
auto Intersect(const Line<T, N>& inLine, const HyperBox<T, N>& inHyperBox);

template <EIntersectMode TIntersectMode, typename T, std::size_t N>
auto Intersect(const Line<T, N>& inLine, const HyperSphere<T, N>& inHyperSphere);

template <EIntersectMode TIntersectMode, typename T>
auto Intersect(const Line3<T>& inLine, const Cylinder<T>& inCylinder);

template <EIntersectMode TIntersectMode, typename T, std::size_t N>
auto Intersect(const Line<T, N>& inLine, const Capsule<T, N>& inCapsule);

template <typename T, std::size_t N>
bool Contains(const Line<T, N>& inLine, const Vec<T, N>& inPoint);

template <typename T, std::size_t N>
constexpr Line<T, N> Translated(const Line<T, N>& inLine, const Vec<T, N>& inTranslation);

template <typename T, std::size_t N>
constexpr Line<T, N> Rotated(const Line<T, N>& inLine, const RotationType_t<T, N>& inRotation);
}

#include <ez/Line.tcc>