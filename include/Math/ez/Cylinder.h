#pragma once

#include <ez/IntersectMode.h>
#include <ez/Segment.h>
#include <ez/Vec.h>

namespace ez
{
template <typename T>
class Cylinder
{
public:
  using ValueType = T;
  static constexpr auto NumComponents = 3;

  Cylinder() = default;
  Cylinder(const Vec3<T>& inOrigin, const Vec3<T>& inDestiny, const T inRadius);

  void SetOrigin(const Vec3<T>& inOrigin) { mOrigin = inOrigin; }
  void SetDestiny(const Vec3<T>& inDestiny) { mDestiny = inDestiny; }
  void SetRadius(const T inRadius) { mRadius = inRadius; }

  Segment3<T> GetSegment() const { return Segment3<T> { mOrigin, mDestiny }; }
  const Vec3<T>& GetOrigin() const { return mOrigin; }
  const Vec3<T>& GetDestiny() const { return mDestiny; }
  const T& GetRadius() const { return mRadius; }

private:
  Vec3<T> mOrigin { Zero<Vec3<T>>() };
  Vec3<T> mDestiny { One<Vec3<T>>() };
  T mRadius { static_cast<T>(1) };
};

// Traits
template <typename T>
struct IsCylinder<Cylinder<T>> : std::true_type
{
};

template <typename T>
constexpr bool Contains(const Cylinder<T>& inCylinder, const Vec3<T>& inPoint);

template <typename T>
constexpr T SqLength(const Cylinder<T>& inCylinder);

template <typename T>
constexpr Vec3<T> Direction(const Cylinder<T>& inCylinder);

template <typename T>
constexpr RotationType_t<T, 3> Orientation(const Cylinder<T>& inCylinder);

template <typename T>
constexpr Cylinder<T> Translated(const Cylinder<T>& inCylinder, const Vec3<T>& inTranslation);

template <typename T>
constexpr Cylinder<T> Rotated(const Cylinder<T>& inCylinder, const RotationType_t<T, 3>& inRotation);

template <typename T>
constexpr Vec3<T> Center(const Cylinder<T>& inCylinder);

// Intersect
template <EIntersectMode TIntersectMode, typename T>
auto Intersect(const Cylinder<T>& inCylinder, const Vec3<T>& inPoint);

template <EIntersectMode TIntersectMode, typename T>
auto Intersect(const Cylinder<T>& inCylinder, const Line3<T>& inLine);

template <EIntersectMode TIntersectMode, typename T>
auto Intersect(const Cylinder<T>& inCylinder, const Ray3<T>& inRay);

template <EIntersectMode TIntersectMode, typename T>
auto Intersect(const Cylinder<T>& inCylinder, const Segment3<T>& inSegment);

template <EIntersectMode TIntersectMode, typename T>
auto Intersect(const Cylinder<T>& inCylinder, const HyperSphere<T, 3>& inHyperSphere);

template <EIntersectMode TIntersectMode, typename T>
auto Intersect(const Cylinder<T>& inCylinder, const AAHyperBox<T, 3>& inAAHyperBox);

template <EIntersectMode TIntersectMode, typename T>
auto Intersect(const Cylinder<T>& inCylinder, const HyperBox<T, 3>& inHyperBox);

template <EIntersectMode TIntersectMode, typename T>
auto Intersect(const Cylinder<T>& inCylinderLHS, const Cylinder<T>& inCylinderRHS);

template <EIntersectMode TIntersectMode, typename T>
auto Intersect(const Cylinder<T>& inCylinder, const Triangle<T, 3>& inTriangle);

}

#include "ez/Cylinder.tcc"