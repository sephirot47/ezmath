#pragma once

#include "ez/IntersectMode.h"
#include "ez/MathCommon.h"
#include "ez/MathInitializers.h"
#include "ez/Vec.h"

namespace ez
{
template <typename T, std::size_t N>
class Ray final
{
public:
  using ValueType = T;
  static constexpr auto NumDimensions = N;

  Ray() = default;
  Ray(const Vec<T, N>& inOrigin, const Vec<T, N>& inDirection);
  Ray(const Ray&) = default;
  Ray& operator=(const Ray&) = default;
  Ray(Ray&&) = default;
  Ray& operator=(Ray&&) = default;
  ~Ray() = default;

  void SetOrigin(const Vec<T, N>& inOrigin) { mOrigin = inOrigin; }
  void SetDirection(const Vec<T, N>& inDirection);

  Vec<T, N> GetPoint(const T& inDistance) const { return mOrigin + inDistance * mDirection; }
  const Vec<T, N>& GetOrigin() const { return mOrigin; }
  const Vec<T, N>& GetDirection() const { return mDirection; }

private:
  Vec<T, N> mOrigin = Zero<Vec<T, N>>();
  Vec<T, N> mDirection = Right<Vec<T, N>>();
};

template <typename T, std::size_t N>
inline std::ostream& operator<<(std::ostream& ioLHS, const Ray<T, N>& inRHS)
{
  ioLHS << "(" << inRHS.GetOrigin() << ", " << inRHS.GetDirection() << ")";
  return ioLHS;
}

template <EIntersectMode TIntersectMode, typename T>
auto Intersect(const Ray3<T>& inRay, const Plane<T>& inPlane);

template <EIntersectMode TIntersectMode, typename T>
auto Intersect(const Plane<T>& inPlane, const Ray3<T>& inRay);

template <EIntersectMode TIntersectMode, typename T, std::size_t N>
auto Intersect(const Ray<T, N>& inRay, const AAHyperBox<T, N>& inAAHyperBox);

template <EIntersectMode TIntersectMode, typename T, std::size_t N>
auto Intersect(const AAHyperBox<T, N>& inAAHyperBox, const Ray<T, N>& inRay);

template <EIntersectMode TIntersectMode, typename T, std::size_t N>
auto Intersect(const Ray<T, N>& inRay, const AAHyperCube<T, N>& inAAHyperCube);

template <EIntersectMode TIntersectMode, typename T, std::size_t N>
auto Intersect(const AAHyperCube<T, N>& inAAHyperCube, const Ray<T, N>& inRay);

template <EIntersectMode TIntersectMode, typename T, std::size_t N>
auto Intersect(const HyperSphere<T, N>& inHyperSphere, const Ray<T, N>& inRay);

template <EIntersectMode TIntersectMode, typename T, std::size_t N>
auto Intersect(const Ray<T, N>& inRay, const HyperSphere<T, N>& inHyperSphere);

template <EIntersectMode TIntersectMode, typename T>
auto Intersect(const Cylinder<T>& inCylinder, const Ray3<T>& inRay);

template <EIntersectMode TIntersectMode, typename T>
auto Intersect(const Ray3<T>& inRay, const Cylinder<T>& inCylinder);

template <EIntersectMode TIntersectMode, typename T>
auto Intersect(const Capsule<T>& inCapsule, const Ray3<T>& inRay);

template <EIntersectMode TIntersectMode, typename T>
auto Intersect(const Ray3<T>& inRay, const Capsule<T>& inCapsule);

template <typename T, std::size_t N>
constexpr Vec<T, N> Direction(const Ray<T, N>& inRay);

// Transformation specializations
template <typename T, std::size_t N>
void Transform(Ray<T, N>& ioRayToTransform, const Transformation<ValueType_t<T>, N>& inTransformation);

template <typename T, std::size_t N>
void Transform(Ray<T, N>& ioRayToTransform, const SquareMat<ValueType_t<T>, N>& inTransformMatrix);

template <typename T, std::size_t N>
void Transform(Ray<T, N>& ioRayToTransform, const SquareMat<T, N + 1>& inTransformMatrix);

template <typename T, std::size_t N>
void InverseTransform(Ray<T, N>& ioRayToTransform, const Transformation<ValueType_t<T>, N>& inTransformation);

}

#include "ez/Ray.tcc"