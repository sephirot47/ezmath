#include "ez/Line.h"
#include "ez/Ray.h"
#include "ez/Segment.h"

namespace ez
{
template <typename T, std::size_t N>
Segment<T, N>::Segment(const Vec<T, N>& inOrigin, const Vec<T, N>& inDestiny) : mOrigin(inOrigin), mDestiny(inDestiny)
{
}

template <typename T, std::size_t N>
Vec<T, N> Segment<T, N>::GetOrigin() const
{
  return mOrigin;
}

template <typename T, std::size_t N>
Vec<T, N> Segment<T, N>::GetDestiny() const
{
  return mDestiny;
}

template <typename T, std::size_t N>
Vec<T, N> Segment<T, N>::GetVector() const
{
  return (mDestiny - mOrigin);
}

template <typename T, std::size_t N>
Vec<T, N> Segment<T, N>::GetPoint(const T& inDistanceFromOrigin) const
{
  return mOrigin + Direction(*this) * inDistanceFromOrigin;
}

template <typename T, std::size_t N>
Line<T, N> Segment<T, N>::GetLine() const
{
  return Line<T, N> { GetOrigin(), Direction(*this) };
}

template <typename T, std::size_t N>
constexpr Vec<T, N> Direction(const Segment<T, N>& inSegment)
{
  const auto direction = NormalizedSafe(inSegment.GetVector());
  return direction;
}

template <typename T>
Vec3<T> Projected(const Vec3<T>& inPoint, const Segment3<T>& inSegment)
{
  const auto& origin = inSegment.GetOrigin();
  const auto& destiny = inSegment.GetDestiny();
  const auto segment_vec = (destiny - origin);
  const auto t = Dot(inPoint - origin, segment_vec) / Dot(segment_vec, segment_vec);
  return Lerp(origin, destiny, Clamp01(t));
}

template <typename T>
bool IsOnPositiveSide(const Segment2<T>& inSegment, const Vec2<T>& inPoint)
{
  return Dot(Perpendicular(inSegment.GetVector()), (inPoint - inSegment.GetOrigin())) > 0;
}

template <typename T, std::size_t N>
T SqLength(const Segment<T, N>& inSegment)
{
  return SqLength(inSegment.GetVector());
}

template <typename T, std::size_t N, typename TPrimitive>
constexpr T ClosestPointT(const Segment<T, N>& inSegment, const TPrimitive& inPrimitive)
{
  const auto segment_sq_length = SqLength(inSegment);
  if (IsVeryEqual(segment_sq_length, static_cast<T>(0)))
    return static_cast<T>(0);

  const auto line_closest_point_t = ClosestPointT(inSegment.GetLine(), inPrimitive);
  const auto line_closest_point_t_clamped = Clamp(line_closest_point_t, static_cast<T>(0), Sqrt(segment_sq_length));
  return line_closest_point_t_clamped;
}

template <typename T, std::size_t N, typename TPrimitive>
constexpr Vec<T, N> ClosestPoint(const Segment<T, N>& inSegment, const TPrimitive& inPrimitive)
{
  const auto closest_point_t = ClosestPointT(inSegment, inPrimitive);
  return inSegment.GetPoint(closest_point_t);
}

template <typename T, std::size_t N>
T SqDistance(const Segment<T, N>& inSegmentLHS, const Segment<T, N>& inSegmentRHS)
{
  const auto closest_point_in_segment_lhs = ClosestPoint(inSegmentLHS, inSegmentRHS);
  return SqDistance(inSegmentRHS, closest_point_in_segment_lhs);
}

template <typename T, std::size_t N, typename TPrimitive>
T SqDistance(const Segment<T, N>& inSegment, const TPrimitive& inPrimitive)
{
  const auto closest_point_in_segment = ClosestPoint(inSegment, inPrimitive);
  return SqDistance(inPrimitive, closest_point_in_segment);
}

// Intersect
template <EIntersectMode TIntersectMode, typename T, std::size_t N>
auto Intersect(const Segment<T, N>& inSegment, const Vec<T, N>& inPoint)
{
  const auto intersection = Intersect<TIntersectMode>(inPoint, inSegment);
  if constexpr (TIntersectMode == EIntersectMode::ALL_INTERSECTIONS)
    return std::array { intersection[0].has_value() ? line_detail::GetT(inSegment, inPoint) : std::optional<T> {} };
  else if constexpr (TIntersectMode == EIntersectMode::ONLY_CLOSEST)
    return intersection.has_value() ? line_detail::GetT(inSegment, inPoint) : std::optional<T> {};
  else if constexpr (TIntersectMode == EIntersectMode::ONLY_CHECK)
    return intersection;
}

template <EIntersectMode TIntersectMode, typename T, std::size_t N>
auto Intersect(const Segment<T, N>& inSegment, const Line<T, N>& inLine)
{
  static_assert(TIntersectMode == EIntersectMode::ALL_INTERSECTIONS || TIntersectMode == EIntersectMode::ONLY_CLOSEST
          || TIntersectMode == EIntersectMode::ONLY_CHECK,
      "Unsupported EIntersectMode.");

  const auto segment_sq_length = SqLength(inSegment);
  if (IsVeryEqual(segment_sq_length, static_cast<T>(0)))
  {
    if constexpr (TIntersectMode == EIntersectMode::ALL_INTERSECTIONS)
      return std::array { std::optional<T> {} };
    else if constexpr (TIntersectMode == EIntersectMode::ONLY_CLOSEST)
      return std::optional<T> {};
    else if constexpr (TIntersectMode == EIntersectMode::ONLY_CHECK)
      return false;
  }

  auto intersection = IntersectClosest(Line2<T> { inSegment.GetOrigin(), Direction(inSegment) }, inLine);
  const auto segment_length = Sqrt(segment_sq_length);
  const auto intersects
      = (intersection.has_value() && (*intersection >= static_cast<T>(0)) && (*intersection <= segment_length));

  if constexpr (TIntersectMode == EIntersectMode::ALL_INTERSECTIONS)
    return std::array { intersects ? intersection : std::optional<T> {} };
  else if constexpr (TIntersectMode == EIntersectMode::ONLY_CLOSEST)
    return intersects ? intersection : std::optional<T> {};
  else if constexpr (TIntersectMode == EIntersectMode::ONLY_CHECK)
    return intersects;
}

template <EIntersectMode TIntersectMode, typename T, typename TPrimitive, std::size_t N>
auto Intersect(const Segment<T, N>& inSegment, const TPrimitive& inPrimitive)
{
  static_assert(TIntersectMode == EIntersectMode::ALL_INTERSECTIONS || TIntersectMode == EIntersectMode::ONLY_CLOSEST
          || TIntersectMode == EIntersectMode::ONLY_CHECK,
      "Unsupported EIntersectMode.");

  constexpr auto Epsilon = static_cast<T>(1e-7);
  const auto segment_sq_length = SqLength(inSegment);
  const auto segment_direction = (segment_sq_length != 0.0f ? Direction(inSegment) : Right<Vec<T, N>>());
  auto intersections = IntersectAll(inSegment.GetLine(), inPrimitive);

  // Invalidate points if outside the segment
  for (auto& intersection : intersections)
  {
    if (!intersection.has_value())
      continue;

    const auto point_inside_segment_range = (*intersection >= Epsilon && Sq(*intersection) <= segment_sq_length);
    if constexpr (TIntersectMode == EIntersectMode::ONLY_CHECK)
    {
      if (point_inside_segment_range)
        return true;
    }
    else if (!point_inside_segment_range)
      intersection = std::nullopt;
  }

  if constexpr (TIntersectMode == EIntersectMode::ALL_INTERSECTIONS)
    return intersections;
  else if constexpr (TIntersectMode == EIntersectMode::ONLY_CLOSEST)
    return line_detail::GetMinIntersectionDistance(intersections);
  else if constexpr (TIntersectMode == EIntersectMode::ONLY_CHECK)
    return (Contains(inPrimitive, inSegment.GetOrigin()) || Contains(inPrimitive, inSegment.GetDestiny()));
}

// Contains
template <typename T, std::size_t N>
bool Contains(const Segment<T, N>& inSegment, const Vec<T, N>& inPoint)
{
  constexpr auto Epsilon = static_cast<T>(1e-7);
  return (SqDistance(inSegment, inPoint) < Epsilon);
}

template <typename T, std::size_t N, typename TPrimitive>
bool Contains(const Segment<T, N>& inSegment, const TPrimitive& inPrimitive)
{
  return false;
}

template <typename T, std::size_t N>
auto GetSATNormals(const Segment<T, N>& inSegment)
{
  return std::array { NormalizedSafe(Perpendicular(inSegment.GetVector())) };
}

template <typename T, std::size_t N>
auto GetSATEdges(const Segment<T, N>&)
{
  return std::array<Vec<T, N>, 0> {};
}

template <typename T, std::size_t N>
auto GetSATPoints(const Segment<T, N>& inSegment)
{
  return std::array { inSegment.GetOrigin(), inSegment.GetDestiny() };
}

template <typename T, std::size_t N>
constexpr RotationType_t<T, N> Orientation(const Segment<T, N>& inSegment)
{
  return FromTo(Forward<Vec3<T>>(), Direction(inSegment));
}

template <typename T, std::size_t N>
constexpr Segment<T, N> Translated(const Segment<T, N>& inSegment, const Vec<T, N>& inTranslation)
{
  return Segment<T, N> { inSegment.GetOrigin() + inTranslation, inSegment.GetDestiny() + inTranslation };
}

template <typename T, std::size_t N>
constexpr Segment<T, N> Rotated(const Segment<T, N>& inSegment, const RotationType_t<T, N>& inRotation)
{
  return Segment<T, N> { Rotated(inSegment.GetOrigin(), inRotation), Rotated(inSegment.GetDestiny(), inRotation) };
}

template <typename T, std::size_t N>
constexpr Vec<T, N> Center(const Segment<T, N>& inSegment)
{
  return (inSegment.GetOrigin() + inSegment.GetDestiny()) / static_cast<T>(2);
}

// Points iterator
template <typename T, std::size_t N>
Vec<T, N> PointsIteratorSpecialization<Segment<T, N>>::GetPoint(const Segment<T, N>& inSegment,
    const std::size_t inPointIndex) const
{
  return inPointIndex == 0 ? inSegment.GetOrigin() : inSegment.GetDestiny();
}
}