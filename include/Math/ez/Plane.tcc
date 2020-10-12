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

template <typename T>
bool Contains(const Plane<T>& inPlane, const Vec3<T>& inPoint)
{
  return IsVeryEqual(Distance(inPoint, inPlane), static_cast<T>(0));
}

template <typename T>
bool Contains(const Plane<T>& inPlane, const Line3<T>& inLine)
{
  return IsVeryEqual(Dot(Normal(inPlane), Direction(inLine)), static_cast<T>(0));
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
}