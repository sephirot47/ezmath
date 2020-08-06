#include "ez/Segment.h"

namespace ez
{
template <typename T, std::size_t N>
Segment<T, N>::Segment(const Vec<T, N>& inFromPoint, const Vec<T, N>& inToPoint)
    : mFromPoint(inFromPoint), mToPoint(inToPoint)
{
}

template <typename T, std::size_t N>
Vec<T, N> Segment<T, N>::GetFromPoint() const
{
  return mFromPoint;
}

template <typename T, std::size_t N>
Vec<T, N> Segment<T, N>::GetToPoint() const
{
  return mToPoint;
}

template <typename T, std::size_t N>
Vec<T, N> Segment<T, N>::GetVector() const
{
  return (mToPoint - mFromPoint);
}

template <typename T, std::size_t N>
constexpr Vec3<T> Direction(const Segment<T, N>& inSegment)
{
  const auto direction = NormalizedSafe(inSegment.GetVector());
  return direction;
}

template <typename T>
Vec3<T> Projected(const Vec3<T>& inPoint, const Segment3<T>& inSegment)
{
  const auto& from_point = inSegment.GetFromPoint();
  const auto& to_point = inSegment.GetToPoint();
  const auto segment_vec = (to_point - from_point);
  const auto t = Dot(inPoint - from_point, segment_vec) / Dot(segment_vec, segment_vec);
  return Lerp(from_point, to_point, Clamp01(t));
}

template <typename T, std::size_t N>
T SqLength(const Segment<T, N>& inSegment)
{
  return SqLength(inSegment.GetVector());
}

template <typename T>
T SqDistance(const Vec3<T>& inPoint, const Segment3<T>& inSegment)
{
  const auto origin_to_destiny = (inSegment.GetToPoint() - inSegment.GetFromPoint());

  const auto origin_to_point = (inPoint - inSegment.GetFromPoint());
  const auto point_proj_in_segment_sq_length = Dot(origin_to_point, origin_to_destiny);
  if (point_proj_in_segment_sq_length <= static_cast<T>(0))
  {
    const auto point_to_origin_sq_dist = Dot(origin_to_point, origin_to_point);
    return point_to_origin_sq_dist;
  }

  const auto segment_sq_length = Dot(origin_to_destiny, origin_to_destiny);
  if (point_proj_in_segment_sq_length >= segment_sq_length)
  {
    const auto destiny_to_point = (inPoint - inSegment.GetToPoint());
    const auto point_to_destiny_sq_dist = Dot(destiny_to_point, destiny_to_point);
    return point_to_destiny_sq_dist;
  }

  const auto point_proj_in_segment_length_ratioed = (point_proj_in_segment_sq_length / segment_sq_length);
  const auto point_proj_in_segment
      = (inSegment.GetFromPoint() + origin_to_destiny * point_proj_in_segment_length_ratioed);
  const auto point_to_segment_vec = (point_proj_in_segment - inPoint);
  const auto point_to_segment_sq_dist = Dot(point_to_segment_vec, point_to_segment_vec);
  return point_to_segment_sq_dist;
}

template <typename T>
T SqDistance(const Segment3<T>& inSegment, const Vec3<T>& inPoint)
{
  return SqDistance(inPoint, inSegment);
}

template <typename T>
T SqDistance(const Segment3<T>& inSegmentLHS, const Segment3<T>& inSegmentRHS)
{
  // WARNING: Untested
  // http://geomalgorithms.com/a07-_distance.html
  constexpr float Epsilon = 1e-5f;

  const auto u = (inSegmentLHS.GetToPoint() - inSegmentLHS.GetFromPoint());
  const auto v = (inSegmentRHS.GetToPoint() - inSegmentRHS.GetFromPoint());
  const auto w = (inSegmentLHS.GetFromPoint() - inSegmentRHS.GetFromPoint());
  const auto a = Dot(u, u); // Always >= 0
  const auto b = Dot(u, v);
  const auto c = Dot(v, v); // Always >= 0
  const auto d = Dot(u, w);
  const auto e = Dot(v, w);
  const auto D = a * c - b * b; // Always >= 0
  T sc, sN, sD = D;             // sc = sN / sD, default sD = D >= 0
  T tc, tN, tD = D;             // tc = tN / tD, default tD = D >= 0

  // Compute the line parameters of the two closest points
  if (D < Epsilon) // The lines are almost parallel
  {
    sN = static_cast<T>(0); // Force using point FromPoint on SegmentLHS
    sD = static_cast<T>(1); // To prevent possible division by 0 later
    tN = e;
    tD = c;
  }
  else // Get the closest points on the infinite lines
  {
    sN = (b * e - c * d);
    tN = (a * e - b * d);
    if (sN < static_cast<T>(0)) // sc < 0 => the s=0 edge is visible
    {
      sN = static_cast<T>(0);
      tN = e;
      tD = c;
    }
    else if (sN > sD) // sc > 1  => the s=1 edge is visible
    {
      sN = sD;
      tN = e + b;
      tD = c;
    }
  }

  if (tN < static_cast<T>(0)) // tc < 0 => the t=0 edge is visible
  {
    tN = static_cast<T>(0); // Recompute sc for this edge
    if (-d < static_cast<T>(0))
    {
      sN = static_cast<T>(0);
    }
    else if (-d > a)
    {
      sN = sD;
    }
    else
    {
      sN = -d;
      sD = a;
    }
  }
  else if (tN > tD) // tc > 1  => the t=1 edge is visible
  {
    tN = tD; // Recompute sc for this edge
    if ((-d + b) < static_cast<T>(0))
    {
      sN = static_cast<T>(0);
    }
    else if ((-d + b) > a)
    {
      sN = sD;
    }
    else
    {
      sN = (-d + b);
      sD = a;
    }
  }

  // Finally do the division to get sc and tc
  sc = (Abs(sN) < Epsilon ? static_cast<T>(0) : sN / sD);
  tc = (Abs(tN) < Epsilon ? static_cast<T>(0) : tN / tD);

  // Get the difference of the two closest points
  const auto dP = w + (sc * u) - (tc * v); // =  inSegmentLHS(sc) - inSegmentRHS(tc)

  return SqLength(dP); // Return the closest distance
}

// Intersection functions
template <EIntersectMode TIntersectMode, typename T, typename TPrimitive>
auto Intersect(const Segment<T, 3>& inSegment, const TPrimitive& inPrimitive)
{
  static_assert(TIntersectMode == EIntersectMode::ALL_INTERSECTIONS || TIntersectMode == EIntersectMode::ONLY_CLOSEST
          || TIntersectMode == EIntersectMode::ONLY_CHECK,
      "Unsupported EIntersectMode.");

  const auto segment_sq_length = SqLength(inSegment);
  const auto segment_direction = (segment_sq_length != 0.0f ? Direction(inSegment) : Right<Vec<T, 3>>());
  const auto segment_ray = Ray<T, 3>(inSegment.GetFromPoint(), segment_direction);
  auto intersection_distances = Intersect<EIntersectMode::ALL_INTERSECTIONS>(segment_ray, inPrimitive);

  // Invalidate points if outside the segment
  for (auto& intersection_distance : intersection_distances)
  {
    if (!intersection_distance.has_value())
      continue;

    assert(*intersection_distance >= static_cast<T>(0));

    const auto point_outside_segment_range = (Sq(*intersection_distance) > segment_sq_length);
    if (!point_outside_segment_range)
      continue;

    const auto point_inside_primitive = Contains(inPrimitive, inSegment.GetToPoint());
    intersection_distance = point_inside_primitive ? std::make_optional(Sqrt(segment_sq_length)) : std::nullopt;
  }

  if constexpr (TIntersectMode == EIntersectMode::ALL_INTERSECTIONS)
  {
    return intersection_distances;
  }
  else if constexpr (TIntersectMode == EIntersectMode::ONLY_CLOSEST)
  {
    std::optional<T> min_intersection_distance;
    for (auto& intersection_distance : intersection_distances)
    {
      if (!min_intersection_distance.has_value())
        min_intersection_distance = intersection_distance;
      else if (intersection_distance.has_value())
        min_intersection_distance = Min(*intersection_distance, *min_intersection_distance);
    }
    return min_intersection_distance;
  }
  else if constexpr (TIntersectMode == EIntersectMode::ONLY_CHECK)
  {
    return std::any_of(intersection_distances.cbegin(),
        intersection_distances.cend(),
        [](const auto& in_intersection_distance) { return in_intersection_distance.has_value(); });
  }
}

template <EIntersectMode TIntersectMode, typename T, typename TPrimitive>
auto Intersect(const TPrimitive& inPrimitive, const Segment<T, 3>& inSegment)
{
  return Intersect<TIntersectMode>(inSegment, inPrimitive);
}

}