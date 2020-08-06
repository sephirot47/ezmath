#include "ez/MathCommon.h"
#include "ez/Ray.h"

namespace ez
{

template <typename T, std::size_t N>
Ray<T, N>::Ray(const Vec<T, N>& inOrigin, const Vec<T, N>& inDirection) : mOrigin(inOrigin)
{
  // Use setter to check direction is normalized
  SetDirection(inDirection);
}

template <typename T, std::size_t N>
void Ray<T, N>::SetDirection(const Vec<T, N>& inDirection)
{
  EXPECTS(IsNormalized(inDirection));
  mDirection = inDirection;
}

template <typename T, std::size_t N>
constexpr Vec3<T> Direction(const Ray<T, N>& inRay)
{
  return inRay.GetDirection();
}

// Transformation specialization
template <typename T, std::size_t N>
void Transform(Ray<T, N>& ioRayToTransform, const Transformation<ValueType_t<T>, N>& inTransformation)
{
  ioRayToTransform.SetOrigin(inTransformation.TransformedPoint(ioRayToTransform.GetOrigin()));
  ioRayToTransform.SetDirection(NormalizedSafe(inTransformation.TransformedDirection(ioRayToTransform.GetDirection())));
}

template <typename T, std::size_t N>
void Transform(Ray<T, N>& ioRayToTransform, const SquareMat<ValueType_t<T>, N>& inTransformMatrix)
{
  ioRayToTransform.SetOrigin(inTransformMatrix * ioRayToTransform.GetOrigin());
  ioRayToTransform.SetDirection(NormalizedSafe(inTransformMatrix * ioRayToTransform.GetDirection()));
}

template <typename T, std::size_t N>
void Transform(Ray<T, N>& ioRayToTransform, const SquareMat<T, N + 1>& inTransformMatrix)
{
  ioRayToTransform.SetOrigin(XYZ(inTransformMatrix * XYZ1(ioRayToTransform.GetOrigin())));
  ioRayToTransform.SetDirection(NormalizedSafe(XYZ(inTransformMatrix * XYZ0(ioRayToTransform.GetDirection()))));
}

template <typename T, std::size_t N>
void InverseTransform(Ray<T, N>& ioRayToTransform, const Transformation<ValueType_t<T>, N>& inTransformation)
{
  ioRayToTransform.SetOrigin(inTransformation.InverseTransformedPoint(ioRayToTransform.GetOrigin()));
  ioRayToTransform.SetDirection(
      NormalizedSafe(inTransformation.InverseTransformedDirection(ioRayToTransform.GetDirection())));
}
}