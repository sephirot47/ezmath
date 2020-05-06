#include "ez/Plane.h"

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
}