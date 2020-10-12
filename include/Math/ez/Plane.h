#pragma once

#include <ez/IntersectMode.h>
#include <ez/Macros.h>
#include <ez/MathForward.h>
#include <ez/MathInitializers.h>
#include <ez/Vec.h>
#include <optional>
#include <type_traits>

namespace ez
{
template <typename T>
class Plane final
{
public:
  using ValueType = T;
  static constexpr auto NumDimensions = 3;
  static constexpr auto NumComponents = 3;

  Plane() = default;
  Plane(const Vec3<T>& inNormal, const T inDistanceFromOriginInNormalDirection);
  Plane(const Vec3<T>& inNormal, const Vec3<T>& inPlanePoint);
  Plane(const Plane&) = default;
  Plane& operator=(const Plane&) = default;
  Plane(Plane&&) = default;
  Plane& operator=(Plane&&) = default;
  ~Plane() = default;

  void SetNormal(const Vec3<T>& inNormal);
  void SetDistanceFromOrigin(const T inDistance);

  const Vec3<T>& GetNormal() const;
  T GetDistanceFromOrigin() const;
  Vec3<T> GetArbitraryPoint() const;

private:
  Vec3<T> mNormal = Forward<Vec3<T>>();      // A, B, C
  T mDistanceFromOrigin = static_cast<T>(0); // Distance from origin i-n the direction of the normal (D)
};

template <typename T>
struct IsPlane<Plane<T>> : std::true_type
{
};

template <typename T>
Vec3<T> Normal(const Plane<T>& inPlane);

template <typename T>
T Distance(const Vec3<T>& inPoint, const Plane<T>& inPlane);

template <typename T>
T Distance(const Plane<T>& inPlane, const Vec3<T>& inPoint);

template <typename T>
T SqDistance(const Vec3<T>& inPoint, const Plane<T>& inPlane);

template <typename T>
T SqDistance(const Plane<T>& inPlane, const Vec3<T>& inPoint);

template <typename T>
Vec3<T> Projected(const Vec3<T>& inPoint, const Plane<T>& inPlaneToProjectTo);

template <EIntersectMode TIntersectMode, typename T>
auto Intersect(const Plane<T>& inPlane, const Vec3<T>& inPoint);

template <EIntersectMode TIntersectMode, typename T>
auto Intersect(const Plane<T>& inPlane, const Line3<T>& inLine);

template <EIntersectMode TIntersectMode, typename T>
auto Intersect(const Plane<T>& inPlane, const Ray3<T>& inRay);

template <EIntersectMode TIntersectMode, typename T>
auto Intersect(const Plane<T>& inPlane, const Segment3<T>& inSegment);

template <EIntersectMode TIntersectMode, typename T>
auto Intersect(const Plane<T>& inPlaneLHS, const Plane<T>& inPlaneRHS);

template <EIntersectMode TIntersectMode, typename T>
auto Intersect(const Plane<T>& inPlane, const AAHyperBox<T, 3>& inAABox);

template <typename T>
bool Contains(const Plane<T>& inPlane, const Vec3<T>& inPoint);

template <typename T>
bool Contains(const Plane<T>& inPlane, const Line3<T>& inLine);

template <typename T>
bool Contains(const Plane<T>& inPlane, const Ray3<T>& inRay);

template <typename T>
bool Contains(const Plane<T>& inPlane, const Segment3<T>& inSegment);

template <typename T>
bool Contains(const Plane<T>& inPlaneLHS, const Plane<T>& inPlaneRHS);

template <typename T>
bool Contains(const Plane<T>& inPlane, const Sphere<T>& inSphere);

template <typename T>
bool Contains(const Plane<T>& inPlane, const AABox<T>& inAABox);

template <typename T>
bool Contains(const Plane<T>& inPlane, const Box<T>& inBox);

template <typename T>
bool Contains(const Plane<T>& inPlane, const Triangle3<T>& inTriangle);

template <typename T>
Vec3<T> ClosestPoint(const Plane<T>& inPlane, const Vec3<T>& inPoint);

template <typename T>
Vec3<T> ClosestPoint(const Plane<T>& inPlane, const Line3<T>& inLine);

template <typename T>
Vec3<T> ClosestPoint(const Plane<T>& inPlane, const Ray3<T>& inRay);

template <typename T>
Vec3<T> ClosestPoint(const Plane<T>& inPlane, const Segment3<T>& inSegment);

template <typename T>
Vec3<T> ClosestPoint(const Plane<T>& inPlaneLHS, const Plane<T>& inPlaneRHS);

template <typename T>
Vec3<T> ClosestPoint(const Plane<T>& inPlane, const AABox<T>& inAABox);

}

#include "ez/Plane.tcc"