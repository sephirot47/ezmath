#pragma once

#include <ez/IntersectMode.h>
#include <ez/MathCommon.h>
#include <ez/MathForward.h>
#include <ez/MathInitializers.h>
#include <ez/PointsIterator.h>
#include <ez/Quat.h>
#include <ez/SegmentsIterator.h>
#include <ez/Vec.h>
#include <cstdint>
#include <type_traits>

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
  Vec<T, N> GetPoint(const T& inDistanceFromOrigin) const;
  Line<T, N> GetLine() const;

private:
  Vec<T, N> mOrigin = Zero<Vec<T, N>>();
  Vec<T, N> mDestiny = Zero<Vec<T, N>>();
};

template <typename T, std::size_t N>
struct IsSegment<Segment<T, N>> : std::true_type
{
};

template <typename T, std::size_t N>
inline std::ostream& operator<<(std::ostream& ioLHS, const Segment<T, N>& inRHS);

template <typename T, std::size_t N>
constexpr Vec<T, N> Direction(const Segment<T, N>& inSegment);

template <typename T>
Vec3<T> Projected(const Vec3<T>& inPoint, const Segment3<T>& inSegment);

template <typename T>
bool IsOnPositiveSide(const Segment2<T>& inSegment, const Vec2<T>& inPoint);

template <typename T, std::size_t N>
T SqLength(const Segment<T, N>& inSegment);

template <typename T, std::size_t N, typename TPrimitive>
constexpr T ClosestPointT(const Segment<T, N>& inSegment, const TPrimitive& inPrimitive);

template <typename T, std::size_t N, typename TPrimitive>
constexpr Vec<T, N> ClosestPoint(const Segment<T, N>& inSegment, const TPrimitive& inPrimitive);

template <typename T, std::size_t N>
T SqDistance(const Segment<T, N>& inSegmentLHS, const Segment<T, N>& inSegmentRHS);

template <typename T, std::size_t N, typename TPrimitive>
T SqDistance(const Segment<T, N>& inSegment, const TPrimitive& inPrimitive);

template <typename T, std::size_t N>
constexpr RotationType_t<T, N> Orientation(const Segment<T, N>& inSegment);

template <typename T, std::size_t N>
constexpr Vec<T, N> Center(const Segment<T, N>& inSegment);

// Intersection functions
template <EIntersectMode TIntersectMode, typename T, std::size_t N>
auto Intersect(const Segment<T, N>& inSegment, const Vec<T, N>& inPoint);

template <EIntersectMode TIntersectMode, typename T, std::size_t N>
auto Intersect(const Segment<T, N>& inSegment, const Line<T, N>& inLine);

template <EIntersectMode TIntersectMode, typename T, typename TPrimitive, std::size_t N>
auto Intersect(const Segment<T, N>& inSegment, const TPrimitive& inPrimitive);

// Contains
template <typename T, std::size_t N>
bool Contains(const Segment<T, N>& inSegment, const Vec<T, N>& inPoint);

template <typename T, std::size_t N, typename TPrimitive>
bool Contains(const Segment<T, N>& inSegment, const TPrimitive& inPrimitive);

template <typename T, std::size_t N>
auto GetSATNormals(const Segment<T, N>& inSegment);

template <typename T, std::size_t N>
auto GetSATEdges(const Segment<T, N>& inSegment);

template <typename T, std::size_t N>
auto GetSATPoints(const Segment<T, N>& inSegment);

template <typename T, std::size_t N>
constexpr Segment<T, N> Translated(const Segment<T, N>& inSegment, const Vec<T, N>& inTranslation);

template <typename T, std::size_t N>
constexpr Segment<T, N> Rotated(const Segment<T, N>& inSegment, const RotationType_t<T, N>& inRotation);

// Points iterator
template <typename T, std::size_t N>
struct PointsIteratorSpecialization<Segment<T, N>>
{
  static constexpr std::size_t NumPoints = 2;
  PointsIteratorSpecialization(const Segment<T, N>& inSegment) {}
  Vec<T, N> GetPoint(const Segment<T, N>& inSegment, const std::size_t inPointIndex) const;
};

// Segments iterator
template <typename T, std::size_t N>
struct SegmentsIteratorSpecialization<Segment<T, N>>
{
  static constexpr std::size_t NumSegments = 1;
  SegmentsIteratorSpecialization(const Segment<T, N>& inSegment) {}
  Segment<T, N> GetSegment(const Segment<T, N>& inSegment, const std::size_t inSegmentIndex) const { return inSegment; }
};

}

#include "ez/Segment.tcc"