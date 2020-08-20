#include "ez/Cylinder.h"

namespace ez
{
template <typename T>
Cylinder<T>::Cylinder(const Vec3<T>& inOrigin, const Vec3<T>& inDestiny, const T inRadius)
    : mOrigin { inOrigin }, mDestiny { inDestiny }, mRadius { inRadius }
{
}

template <typename T>
constexpr bool Contains(const Cylinder<T>& inCylinder, const Vec3<T>& inPoint)
{
  const auto cylinder_orientation_inv = -Orientation(inCylinder);
  const auto point_local = cylinder_orientation_inv * (inPoint - inCylinder.GetDestiny());

  // Check z range
  if (point_local[2] < static_cast<T>(0))
    return false;
  if (Sq(point_local[2]) > SqLength(inCylinder))
    return false;

  // Check xy range
  const auto sq_distance_xy = SqLength(XY(point_local));
  return (sq_distance_xy <= Sq(inCylinder.GetRadius()));
}

template <typename T>
constexpr T SqLength(const Cylinder<T>& inCylinder)
{
  return SqLength(inCylinder.GetSegment());
}

template <typename T>
constexpr Vec3<T> Direction(const Cylinder<T>& inCylinder)
{
  return Direction(inCylinder.GetSegment());
}

template <typename T>
constexpr RotationType_t<T, 3> Orientation(const Cylinder<T>& inCylinder)
{
  return FromTo(Forward<Vec3<T>>(), Direction(inCylinder));
}

template <typename T>
constexpr Cylinder<T> Translated(const Cylinder<T>& inCylinder, const Vec3<T>& inTranslation)
{
  return Cylinder<T> { inCylinder.GetOrigin() + inTranslation,
    inCylinder.GetDestiny() + inTranslation,
    inCylinder.GetRadius() };
}

template <typename T>
constexpr Cylinder<T> Rotated(const Cylinder<T>& inCylinder, const RotationType_t<T, 3>& inRotation)
{
  return Cylinder<T> { Rotated(inCylinder.GetOrigin(), inRotation),
    Rotated(inCylinder.GetDestiny(), inRotation),
    inCylinder.GetRadius() };
}

template <typename T>
constexpr Vec3<T> Center(const Cylinder<T>& inCylinder)
{
  return (inCylinder.GetOrigin() + inCylinder.GetDestiny()) / static_cast<T>(2);
}

}