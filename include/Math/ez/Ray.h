#pragma once

#include "ez/IntersectMode.h"
#include "ez/MathCommon.h"
#include "ez/MathInitializers.h"
#include "ez/Vec.h"
#include <type_traits>

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
struct IsRay<Ray<T, N>> : std::true_type
{
};

template <typename T, std::size_t N>
inline std::ostream& operator<<(std::ostream& ioLHS, const Ray<T, N>& inRHS)
{
  ioLHS << "(" << inRHS.GetOrigin() << ", " << inRHS.GetDirection() << ")";
  return ioLHS;
}

template <typename T, std::size_t N>
constexpr Vec<T, N> Direction(const Ray<T, N>& inRay);

// Intersection functions
template <EIntersectMode TIntersectMode, typename T, typename TPrimitive, std::size_t N>
auto Intersect(const Ray<T, N>& inRay, const TPrimitive& inPrimitive);

template <EIntersectMode TIntersectMode, typename T, typename TPrimitive, std::size_t N>
auto Intersect(const TPrimitive& inPrimitive, const Ray<T, N>& inRay);

template <typename T, std::size_t N>
bool Contains(const Ray<T, N>& inRay, const Vec<T, N>& inPoint);

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