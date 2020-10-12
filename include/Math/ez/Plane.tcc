#include <ez/Plane.h>

namespace ez
{

template <typename T>
Plane<T>::Plane(const Vec3<T>& inNormal, const T inDistanceFromOriginInNormalDirection)
    : mNormal(inNormal), mDistanceFromOrigin(inDistanceFromOriginInNormalDirection)
{
  EXPECTS(IsNormalized(inNormal));
}

template <typename T>
Plane<T>::Plane(const Vec3<T>& inNormal, const Vec3<T>& inPlanePoint) : mNormal(inNormal)
{
  EXPECTS(IsNormalized(inNormal));

  const auto origin_to_plane_point = inPlanePoint;
  mDistanceFromOrigin = Dot(inNormal, origin_to_plane_point);
}

template <typename T>
void Plane<T>::SetNormal(const Vec3<T>& inNormal)
{
  EXPECTS(IsNormalized(inNormal));
  mNormal = inNormal;
}

template <typename T>
void Plane<T>::SetDistanceFromOrigin(const T inDistance)
{
  mDistanceFromOrigin = inDistance;
}

template <typename T>
const Vec3<T>& Plane<T>::GetNormal() const
{
  return mNormal;
}

template <typename T>
T Plane<T>::GetDistanceFromOrigin() const
{
  return mDistanceFromOrigin;
}

template <typename T>
Vec3<T> Plane<T>::GetArbitraryPoint() const
{
  return mDistanceFromOrigin * mNormal;
}

template <typename T>
Vec3<T> Normal(const Plane<T>& inPlane)
{
  return inPlane.GetNormal();
}

template <typename T>
T Distance(const Vec3<T>& inPoint, const Plane<T>& inPlane)
{
  const auto plane_point = inPlane.GetArbitraryPoint();
  const auto& plane_normal = inPlane.GetNormal();
  const auto plane_to_point_vector = (inPoint - plane_point);
  const auto vector_projected_to_plane_normal_length = Dot(plane_to_point_vector, plane_normal);
  return vector_projected_to_plane_normal_length;
}

template <typename T>
T Distance(const Plane<T>& inPlane, const Vec3<T>& inPoint)
{
  return Distance(inPoint, inPlane);
}

template <typename T>
T SqDistance(const Vec3<T>& inPoint, const Plane<T>& inPlane)
{
  return Sq(Distance(inPoint, inPlane));
}

template <typename T>
T SqDistance(const Plane<T>& inPlane, const Vec3<T>& inPoint)
{
  return SqDistance(inPoint, inPlane);
}

template <typename T>
Vec3<T> Projected(const Vec3<T>& inPoint, const Plane<T>& inPlaneToProjectTo)
{
  const auto point_to_plane_distance = Distance(inPoint, inPlaneToProjectTo);
  const auto point_projected_to_plane_normal = (inPoint - point_to_plane_distance * inPlaneToProjectTo.GetNormal());
  return point_projected_to_plane_normal;
}

template <EIntersectMode TIntersectMode, typename T>
auto Intersect(const Plane<T>& inPlane, const Vec3<T>& inPoint)
{
  static_assert(TIntersectMode == EIntersectMode::ONLY_CHECK, "Unsupported EIntersectMode.");
  return Contains(inPlane, inPoint);
}

template <EIntersectMode TIntersectMode, typename T>
auto Intersect(const Plane<T>& inPlane, const Line3<T>& inLine)
{
  static_assert(TIntersectMode == EIntersectMode::ONLY_CHECK, "Unsupported EIntersectMode.");
  return Intersect<TIntersectMode>(inLine, inPlane);
}

template <EIntersectMode TIntersectMode, typename T>
auto Intersect(const Plane<T>& inPlane, const Ray3<T>& inRay)
{
  static_assert(TIntersectMode == EIntersectMode::ONLY_CHECK, "Unsupported EIntersectMode.");
  return Intersect<TIntersectMode>(inRay, inPlane);
}

template <EIntersectMode TIntersectMode, typename T>
auto Intersect(const Plane<T>& inPlane, const Segment3<T>& inSegment)
{
  static_assert(TIntersectMode == EIntersectMode::ONLY_CHECK, "Unsupported EIntersectMode.");
  return Intersect<TIntersectMode>(inSegment, inPlane);
}

template <EIntersectMode TIntersectMode, typename T>
auto Intersect(const Plane<T>& inPlaneLHS, const Plane<T>& inPlaneRHS)
{
  static_assert(TIntersectMode == EIntersectMode::ONLY_CHECK, "Unsupported EIntersectMode.");
  return !IsVeryParallel(Normal(inPlaneLHS), Normal(inPlaneRHS));
}

template <EIntersectMode TIntersectMode, typename T>
auto Intersect(const Plane<T>& inPlane, const AAHyperBox<T, 3>& inAABox)
{
  return Intersect<TIntersectMode>(inAABox, inPlane);
}

template <EIntersectMode TIntersectMode, typename T>
auto Intersect(const Plane<T>& inPlane, const HyperBox<T, 3>& inBox)
{
  return Intersect<TIntersectMode>(inBox, inPlane);
}

template <typename T>
bool Contains(const Plane<T>& inPlane, const Vec3<T>& inPoint)
{
  return IsVeryEqual(SqDistance(inPoint, inPlane), static_cast<T>(0));
}

template <typename T>
bool Contains(const Plane<T>& inPlane, const Line3<T>& inLine)
{
  return IsVeryPerpendicular(Normal(inPlane), Direction(inLine)) && Contains(inPlane, inLine.GetOrigin());
}

template <typename T>
bool Contains(const Plane<T>& inPlane, const Ray3<T>& inRay)
{
  return Contains(inPlane, inRay.GetLine());
}

template <typename T>
bool Contains(const Plane<T>& inPlane, const Segment3<T>& inSegment)
{
  return Contains(inPlane, inSegment.GetLine());
}

template <typename T>
bool Contains(const Plane<T>& inPlaneLHS, const Plane<T>& inPlaneRHS)
{
  return Contains(inPlaneLHS, inPlaneRHS.GetArbitraryPoint()) && IsVeryEqual(Normal(inPlaneLHS), Normal(inPlaneRHS));
}

template <typename T>
bool Contains(const Plane<T>& inPlane, const Sphere<T>& inSphere)
{
  return false;
}

template <typename T>
bool Contains(const Plane<T>& inPlane, const AABox<T>& inAABox)
{
  return false;
}

template <typename T>
bool Contains(const Plane<T>& inPlane, const Box<T>& inBox)
{
  return false;
}

template <typename T>
bool Contains(const Plane<T>& inPlane, const Triangle3<T>& inTriangle)
{
  return IsVeryParallel(Normal(inPlane), Normal(inTriangle)) && Contains(inPlane, inTriangle[0]);
}

template <typename T>
Vec3<T> ClosestPoint(const Plane<T>& inPlane, const Vec3<T>& inPoint)
{
  return Projected(inPoint, inPlane);
}

template <typename T>
Vec3<T> ClosestPoint(const Plane<T>& inPlane, const Line3<T>& inLine)
{
  return ClosestPoint(inPlane, ClosestPoint(inLine, inPlane));
}

template <typename T>
Vec3<T> ClosestPoint(const Plane<T>& inPlane, const Ray3<T>& inRay)
{
  return ClosestPoint(inPlane, ClosestPoint(inRay, inPlane));
}

template <typename T>
Vec3<T> ClosestPoint(const Plane<T>& inPlane, const Segment3<T>& inSegment)
{
  return ClosestPoint(inPlane, ClosestPoint(inSegment, inPlane));
}

template <typename T>
Vec3<T> ClosestPoint(const Plane<T>& inPlaneLHS, const Plane<T>& inPlaneRHS)
{
  return inPlaneLHS.GetArbitraryPoint();
}

template <typename T>
Vec3<T> ClosestPoint(const Plane<T>& inPlane, const AABox<T>& inAABox)
{
  return ClosestPoint(inPlane, ClosestPoint(inAABox, inPlane));
}

template <typename T>
Vec3<T> ClosestPoint(const Plane<T>& inPlane, const Box<T>& inBox)
{
  return ClosestPoint(inPlane, ClosestPoint(inBox, inPlane));
}

template <typename T>
Plane<T> Translated(const Plane<T>& inPlane, const Vec3<T>& inTranslation)
{
  return Plane<T> { Normal(inPlane), inPlane.GetArbitraryPoint() + inTranslation };
}

template <typename T>
Plane<T> Rotated(const Plane<T>& inPlane, const RotationType_t<T, 3>& inRotation)
{
  return Plane<T> { Rotated(Normal(inPlane), inRotation), Rotated(inPlane.GetArbitraryPoint(), inRotation) };
}
}