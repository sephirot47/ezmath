#include <ez/Triangle.h>

namespace ez
{

template <typename T, std::size_t N>
T GetPerimeter(const Triangle<T, N>& inTriangle)
{
  const auto length_01 = Distance(inTriangle[0], inTriangle[1]);
  const auto length_02 = Distance(inTriangle[0], inTriangle[2]);
  const auto length_12 = Distance(inTriangle[1], inTriangle[2]);
  const auto perimeter = (length_01 + length_02 + length_12);
  return perimeter;
}

template <typename T, std::size_t N>
T Barycenter(const Triangle<T, N>& inTriangle)
{
  return (inTriangle[0] + inTriangle[1] + inTriangle[2]) / static_cast<T>(3);
}

template <typename T>
Vec3<T> Normal(const Triangle3<T>& inTriangle)
{
  const auto v01 = (inTriangle[0] - inTriangle[1]);
  const auto v21 = (inTriangle[2] - inTriangle[1]);
  return NormalizedSafe(Cross(v01, v21));
}

template <typename T>
Plane<T> GetPlane(const Triangle3<T>& inTriangle)
{
  const auto v10 = (inTriangle[0] - inTriangle[1]);
  const auto v12 = (inTriangle[2] - inTriangle[1]);
  const auto& plane_point = inTriangle[0];
  const auto plane_normal = NormalizedSafe(Cross(v10, v12));
  const auto triangle_plane = Plane<T>(plane_normal, plane_point);
  return triangle_plane;
}

template <typename T>
Vec3<T> Projected(const Vec3<T>& inPoint, const Triangle3<T>& inTriangle)
{
  const auto triangle_plane = GetPlane(inTriangle);
  return Projected(inPoint, triangle_plane);
}

template <typename T, std::size_t N>
Vec3<T> BarycentricCoordinates(const Triangle<T, N>& inTriangle, const Vec<T, N>& inPoint)
{
  const auto& t = inTriangle;
  const auto& p = inPoint;

  const auto v0 = (t[1] - t[0]);
  const auto v1 = (t[2] - t[0]);
  const auto v2 = (p - t[0]);
  const auto dot_00 = Dot(v0, v0);
  const auto dot_01 = Dot(v0, v1);
  const auto dot_11 = Dot(v1, v1);
  const auto dot_20 = Dot(v2, v0);
  const auto dot_21 = Dot(v2, v1);
  const auto denominator = dot_00 * dot_11 - dot_01 * dot_01;
  const auto inv_denominator = static_cast<T>(1) / denominator;
  const auto v = (dot_11 * dot_20 - dot_01 * dot_21) * inv_denominator;
  const auto w = (dot_00 * dot_21 - dot_01 * dot_20) * inv_denominator;
  const auto u = static_cast<T>(1) - v - w;

  return Vec3<T>(u, v, w);
}

template <typename T>
auto GetSATNormals(const Triangle2<T>& inTriangle)
{
  return std::array { Perpendicular(inTriangle[0] - inTriangle[1]),
    Perpendicular(inTriangle[1] - inTriangle[2]),
    Perpendicular(inTriangle[2] - inTriangle[0]) };
}
template <typename T>
auto GetSATNormals(const Triangle3<T>& inTriangle)
{
  return std::array { Normal(inTriangle) };
}

template <typename T, std::size_t N>
auto GetSATEdges(const Triangle<T, N>& inTriangle)
{
  return std::array { (inTriangle[0] - inTriangle[1]),
    (inTriangle[0] - inTriangle[2]),
    (inTriangle[1] - inTriangle[2]) };
}

template <typename T, std::size_t N>
auto GetSATPoints(const Triangle<T, N>& inTriangle)
{
  return std::array { inTriangle[0], inTriangle[1], inTriangle[2] };
}

template <typename T, std::size_t N>
constexpr Triangle<T, N> Translated(const Triangle<T, N>& inTriangle, const Vec<T, N>& inTranslation)
{
  return Triangle<T, N> { inTriangle[0] + inTranslation, inTriangle[1] + inTranslation, inTriangle[2] + inTranslation };
}

template <typename T, std::size_t N>
constexpr Triangle<T, N> Rotated(const Triangle<T, N>& inTriangle, const RotationType_t<T, N>& inRotation)
{
  return Triangle<T, N> { Rotated(inTriangle[0], inRotation),
    Rotated(inTriangle[1], inRotation),
    Rotated(inTriangle[2], inRotation) };
}

template <EIntersectMode TIntersectMode, typename T, std::size_t N>
auto Intersect(const Triangle<T, N>& inTriangle, const Vec<T, N>& inPoint)
{
  static_assert(TIntersectMode == EIntersectMode::ONLY_CHECK, "Unsupported EIntersectMode.");
  return Contains(inTriangle, inPoint);
}

template <EIntersectMode TIntersectMode, typename T, std::size_t N>
auto Intersect(const Triangle<T, N>& inTriangle, const Line<T, N>& inLine)
{
  static_assert(TIntersectMode == EIntersectMode::ONLY_CHECK, "Unsupported EIntersectMode.");
  return Intersect<TIntersectMode>(inLine, inTriangle);
}

template <EIntersectMode TIntersectMode, typename T, std::size_t N>
auto Intersect(const Triangle<T, N>& inTriangle, const Ray<T, N>& inRay)
{
  static_assert(TIntersectMode == EIntersectMode::ONLY_CHECK, "Unsupported EIntersectMode.");
  return Intersect<TIntersectMode>(inRay, inTriangle);
  /*
  static_assert(TIntersectMode == EIntersectMode::ALL_INTERSECTIONS || TIntersectMode == EIntersectMode::ONLY_CLOSEST
          || TIntersectMode == EIntersectMode::ONLY_CHECK,
      "Unsupported EIntersectMode.");

  const auto ray_plane_intersection_distance = IntersectClosest(inRay, GetPlane(inTriangle));
  if (!ray_plane_intersection_distance)
  {
    if constexpr (TIntersectMode == EIntersectMode::ONLY_CHECK)
      return false;
    else
      return std::optional<T>();
  }

  const auto ray_plane_intersection_point = inRay.GetPoint(*ray_plane_intersection_distance);
  const auto barycentric_coordinates = BarycentricCoordinates(inTriangle, ray_plane_intersection_point);
  const auto intersection_point_is_in_triangle = IsBetween(barycentric_coordinates, Zero<Vec3<T>>(), One<Vec3<T>>());
  if constexpr (TIntersectMode == EIntersectMode::ONLY_CHECK)
    return intersection_point_is_in_triangle;
  else
    return intersection_point_is_in_triangle ? ray_plane_intersection_distance : std::optional<T>();
  */
}

template <EIntersectMode TIntersectMode, typename T, std::size_t N>
auto Intersect(const Triangle<T, N>& inTriangle, const Segment<T, N>& inSegment)
{
  static_assert(TIntersectMode == EIntersectMode::ONLY_CHECK, "Unsupported EIntersectMode.");
  return Intersect<TIntersectMode>(inSegment, inTriangle);
}

template <EIntersectMode TIntersectMode, typename T, std::size_t N>
auto Intersect(const Triangle<T, N>& inTriangle, const HyperSphere<T, N>& inHyperSphere)
{
  static_assert(TIntersectMode == EIntersectMode::ONLY_CHECK, "Unsupported EIntersectMode.");
  return Intersect<TIntersectMode>(inHyperSphere, inTriangle);
}

template <EIntersectMode TIntersectMode, typename T, std::size_t N>
auto Intersect(const Triangle<T, N>& inTriangle, const AAHyperBox<T, N>& inAAHyperBox)
{
  static_assert(TIntersectMode == EIntersectMode::ONLY_CHECK, "Unsupported EIntersectMode.");
  return Intersect<TIntersectMode>(inAAHyperBox, inTriangle);
}

template <EIntersectMode TIntersectMode, typename T, std::size_t N>
auto Intersect(const Triangle<T, N>& inTriangle, const HyperBox<T, N>& inHyperBox)
{
  static_assert(TIntersectMode == EIntersectMode::ONLY_CHECK, "Unsupported EIntersectMode.");
  return Intersect<TIntersectMode>(inHyperBox, inTriangle);
}

template <EIntersectMode TIntersectMode, typename T, std::size_t N>
auto Intersect(const Triangle<T, N>& inTriangle, const Capsule<T, N>& inCapsule)
{
  static_assert(TIntersectMode == EIntersectMode::ONLY_CHECK, "Unsupported EIntersectMode.");
  return Intersect<TIntersectMode>(inCapsule, inTriangle);
}

template <EIntersectMode TIntersectMode, typename T, std::size_t N>
auto Intersect(const Triangle<T, N>& inTriangleLHS, const Triangle<T, N>& inTriangleRHS)
{
  static_assert(TIntersectMode == EIntersectMode::ONLY_CHECK, "Unsupported EIntersectMode.");
  return IntersectCheckSAT(inTriangleLHS, inTriangleRHS);
}

template <typename T, std::size_t N>
bool Contains(const Triangle<T, N>& inTriangle, const Vec<T, N>& inPoint)
{
  if constexpr (N == 2)
  {
    const auto side01 = IsOnPositiveSide(Segment2<T> { inTriangle[0], inTriangle[1] }, inPoint);
    const auto side12 = IsOnPositiveSide(Segment2<T> { inTriangle[1], inTriangle[2] }, inPoint);
    const auto side20 = IsOnPositiveSide(Segment2<T> { inTriangle[2], inTriangle[0] }, inPoint);
    const auto intersects = (side01 == side12 && side12 == side20);
    return intersects;
  }
  return false;
}

template <typename T, std::size_t N>
bool Contains(const Triangle<T, N>& inTriangle, const Line<T, N>& inLine)
{
  return false;
}

template <typename T, std::size_t N>
bool Contains(const Triangle<T, N>& inTriangle, const Ray<T, N>& inRay)
{
  return false;
}

template <typename T, std::size_t N>
bool Contains(const Triangle<T, N>& inTriangle, const Segment<T, N>& inSegment)
{
  return Contains(inTriangle, inSegment.GetOrigin()) && Contains(inTriangle, inSegment.GetDestiny());
}

template <typename T, std::size_t N>
bool Contains(const Triangle<T, N>& inTriangle, const HyperSphere<T, N>& inHyperSphere)
{
  if (!Contains(inTriangle, Center(inHyperSphere)))
    return false;

  const auto hyper_sphere_sq_radius = Sq(inHyperSphere.GetRadius());
  for (int i = 0; i < 3; ++i)
  {
    const auto segment = Segment<T, N> { inTriangle[i], inTriangle[(i + 1) % 3] };
    if (SqDistance(segment, Center(inHyperSphere)) < hyper_sphere_sq_radius)
      return false;
  }
  return true;
}

template <typename T, std::size_t N>
bool Contains(const Triangle<T, N>& inTriangle, const AAHyperBox<T, N>& inAAHyperBox)
{
  return std::all_of(MakePointsBegin(inAAHyperBox),
      MakePointsEnd(inAAHyperBox),
      [&](const auto& in_aa_hyper_box_point) { return Contains(inTriangle, in_aa_hyper_box_point); });
}

template <typename T, std::size_t N>
bool Contains(const Triangle<T, N>& inTriangle, const HyperBox<T, N>& inHyperBox)
{
  return std::all_of(MakePointsBegin(inHyperBox), MakePointsEnd(inHyperBox), [&](const auto& in_hyper_box_point) {
    return Contains(inTriangle, in_hyper_box_point);
  });
}

template <typename T, std::size_t N>
bool Contains(const Triangle<T, N>& inTriangle, const Capsule<T, N>& inCapsule)
{
  return Contains(inTriangle, HyperSphere<T, N> { inCapsule.GetOrigin(), inCapsule.GetRadius() })
      && Contains(inTriangle, HyperSphere<T, N> { inCapsule.GetDestiny(), inCapsule.GetRadius() });
}

template <typename T, std::size_t N>
bool Contains(const Triangle<T, N>& inTriangleContainer, const Triangle<T, N>& inTriangleContainee)
{
  return Contains(inTriangleContainer, inTriangleContainee[0]) && Contains(inTriangleContainer, inTriangleContainee[1])
      && Contains(inTriangleContainer, inTriangleContainee[2]);
}

template <typename T, std::size_t N, typename TPrimitive>
constexpr Vec<T, N> ClosestPoint(const Triangle<T, N>& inTriangle, const TPrimitive& inPrimitive)
{
  auto closest_point_sq_distance = Max<T>();
  auto closest_point = Max<Vec<T, N>>();
  for (int i = 0; i < 3; ++i)
  {
    const auto triangle_segment = Segment<T, N> { inTriangle[i], inTriangle[(i + 1) % 3] };
    const auto closest_point_on_triangle_segment = ClosestPoint(triangle_segment, inPrimitive);
    const auto closest_point_on_primitive = ClosestPoint(inPrimitive, closest_point_on_triangle_segment);
    const auto sq_distance = SqDistance(closest_point_on_triangle_segment, closest_point_on_primitive);
    if (sq_distance < closest_point_sq_distance)
    {
      closest_point_sq_distance = sq_distance;
      closest_point = closest_point_on_triangle_segment;
    }
  }
  return closest_point;
}

// Points iterator
template <typename T, std::size_t N>
Vec<T, N> PointsIteratorSpecialization<Triangle<T, N>>::GetPoint(const Triangle<T, N>& inTriangle,
    const std::size_t inPointIndex) const
{
  return inTriangle[inPointIndex];
}

}