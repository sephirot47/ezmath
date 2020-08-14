#pragma once

#include "ez/IntersectMode.h"
#include "ez/MathCommon.h"
#include "ez/MathForward.h"
#include "ez/MathInitializers.h"
#include "ez/Vec.h"
#include <cstdint>

namespace ez
{
template <typename T, std::size_t N>
class Segment final
{
public:
  using ValueType = T;
  static constexpr auto NumComponents = N;
  static constexpr auto NumDimensions = N;

  Segment() = default;
  Segment(const Vec<T, N>& inOrigin, const Vec<T, N>& inDestiny);
  Segment(const Segment& inRHS) = default;
  Segment& operator=(const Segment& inRHS) = default;
  Segment(Segment&& inRHS) = default;
  Segment& operator=(Segment&& inRHS) = default;

  Vec<T, N> GetOrigin() const;
  Vec<T, N> GetDestiny() const;
  Vec<T, N> GetVector() const;
  Vec<T, N> GetPoint(const T &inDistanceFromOrigin) const;

private:
  Vec<T, N> mOrigin = Zero<Vec<T, N>>();
  Vec<T, N> mDestiny = Zero<Vec<T, N>>();
};

template <typename T, std::size_t N>
constexpr Vec<T, N> Direction(const Segment<T, N>& inSegment);

template <typename T>
Vec3<T> Projected(const Vec3<T>& inPoint, const Segment3<T>& inSegment);

template <typename T, std::size_t N>
T SqLength(const Segment<T, N>& inSegment);

template <typename T, std::size_t N, typename TPrimitive>
constexpr T ClosestPointT(const Segment<T, N>& inSegment, const TPrimitive& inPrimitive);

template <typename T, std::size_t N, typename TPrimitive>
constexpr Vec<T, N> ClosestPoint(const Segment<T, N>& inSegment, const TPrimitive& inPrimitive);

/*
template <typename T>
T SqDistance(const Segment3<T>& inSegment, const Plane<T>& inPlane);

template <typename T>
T SqDistance(const Plane<T>& inPlane, const Segment3<T>& inSegment);

template <typename T, std::size_t N>
T SqDistance(const Segment<T, N>& inSegment, const Vec<T, N>& inPoint);

template <typename T, std::size_t N>
T SqDistance(const Vec<T, N>& inPoint, const Segment<T, N>& inSegment);
*/

template <typename T, std::size_t N>
T SqDistance(const Segment<T, N>& inSegmentLHS, const Segment<T, N>& inSegmentRHS);

template <typename T, std::size_t N, typename TPrimitive>
T SqDistance(const Segment<T, N>& inSegment, const TPrimitive& inPrimitive);

template <typename T, std::size_t N, typename TPrimitive>
T SqDistance(const TPrimitive& inPrimitive, const Segment<T, N>& inSegment);
/*
*/

template <typename T, std::size_t N>
constexpr RotationType_t<T, N> Orientation(const Segment<T, N>& inSegment);

template <typename T, std::size_t N>
constexpr Vec<T, N> Center(const Segment<T, N>& inSegment);

// Intersection functions
template <EIntersectMode TIntersectMode, typename T, typename TPrimitive, std::size_t N>
auto Intersect(const Segment<T, N>& inSegment, const TPrimitive& inPrimitive);

template <typename T, std::size_t N>
bool Contains(const Segment<T, N>& inSegment, const Vec<T, N>& inPoint);
}

#include "ez/Segment.tcc"