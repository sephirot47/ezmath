#pragma once

#include <ez/Segment.h>
#include <ez/Vec.h>
#include <ez/IntersectMode.h>

namespace ez
{
template <typename T>
class Cylinder
{
public:
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
constexpr bool Contains(const Cylinder<T>& inCylinder, const Vec3<T> &inPoint);

template <typename T>
constexpr T SqLength(const Cylinder<T>& inCylinder);

template <typename T>
constexpr Vec3<T> Direction(const Cylinder<T>& inCylinder);

template <typename T>
constexpr RotationType_t<T, 3> Orientation(const Cylinder<T>& inCylinder);

template <typename T>
constexpr Vec3<T> Center(const Cylinder<T>& inCylinder);

}

#include "ez/Cylinder.tcc"