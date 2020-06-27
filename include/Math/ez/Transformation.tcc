#include "ez/Math.h"
#include "ez/Transformation.h"
#include <experimental/type_traits>

namespace ez
{
template <typename T, std::size_t N>
Transformation<T, N>::Transformation(const Vec<T, N>& inPosition) : mPosition(inPosition)
{
}

template <typename T, std::size_t N>
Transformation<T, N>::Transformation(const Vec<T, N>& inPosition, const Transformation<T, N>::RotationType& inRotation)
    : mPosition(inPosition), mRotation(inRotation)
{
}

template <typename T, std::size_t N>
Transformation<T, N>::Transformation(const Vec<T, N>& inPosition,
    const Transformation<T, N>::RotationType& inRotation,
    const Vec<T, N>& inScale)
    : mPosition(inPosition), mRotation(inRotation), mScale(inScale)
{
}

template <typename T, std::size_t N>
Transformation<T, N> Transformation<T, N>::operator*(const Transformation<T, N>& inRHS) const
{
  return TransformedTransform(inRHS);
}

template <typename T, std::size_t N>
inline std::ostream& operator<<(std::ostream& ioLHS, const Transformation<T, N>& inRHS)
{
  ioLHS << "(" << inRHS.GetPosition() << ", " << inRHS.GetRotation() << ", " << inRHS.GetScale() << ")";
  return ioLHS;
}

template <typename T, std::size_t N>
Vec<T, N> Transformation<T, N>::TransformedPoint(const Vec<T, N>& inPoint) const
{
  return mPosition + (Rotated((inPoint * mScale), mRotation));
}

template <typename T, std::size_t N>
Vec<T, N> Transformation<T, N>::InverseTransformedPoint(const Vec<T, N>& inPoint) const
{
  return Rotated((inPoint - mPosition), -mRotation) / mScale;
}

template <typename T, std::size_t N>
Vec<T, N> Transformation<T, N>::TransformedDirection(const Vec<T, N>& inDirection) const
{
  return Rotated((inDirection * mScale), mRotation);
}

template <typename T, std::size_t N>
Vec<T, N> Transformation<T, N>::InverseTransformedDirection(const Vec<T, N>& inDirection) const
{
  return Rotated(inDirection, -mRotation) / mScale;
}

template <typename T, std::size_t N>
Transformation<T, N> Transformation<T, N>::TransformedTransform(const Transformation<T, N>& inRHS) const
{
  const auto new_position = TransformedPoint(inRHS.mPosition);
  const auto new_rotation = Rotated(mRotation, inRHS.mRotation);
  const auto new_scale = mScale * (Rotated(inRHS.mScale, mRotation));
  return Transformation<T, N> { new_position, new_rotation, new_scale };
}

template <typename T, std::size_t N>
SquareMat<T, N + 1> Transformation<T, N>::GetMatrix() const
{
  return TranslationMat(mPosition) * RotationMat(mRotation) * ScaleMat(mScale);
}

template <typename T, std::size_t N>
SquareMat<T, N + 1> Transformation<T, N>::GetInverseMatrix() const
{
  return ScaleMat(1.0f / mScale) * RotationMat(-mRotation) * TranslationMat(-mPosition);
}

template <typename T, std::size_t N>
void Transform(T& ioObject, const SquareMat<ValueType_t<T>, N>& inTransformMatrix)
{
  for (auto& value : ioObject) Transform(value, inTransformMatrix);
}

template <typename T, std::size_t N>
void Transform(T& ioObject, const SquareMat<ValueType_t<T>, N + 1>& inTransformMatrix)
{
  for (auto& value : ioObject) Transform(value, inTransformMatrix);
}

template <typename T, std::size_t N>
void Transform(T& ioObject, const Transformation<ValueType_t<T>, N>& inTransformation)
{
  for (auto& value : ioObject)
    value = inTransformation.TransformedPoint(value); // Assume points. If this is not valid, specialize functions.
}

template <typename T, std::size_t N>
void InverseTransform(T& ioObjectToTransform, const Transformation<ValueType_t<T>, N>& inTransformMatrix)
{
  UNUSED(ioObjectToTransform);
  UNUSED(inTransformMatrix);
  static_assert(!std::is_same_v<T, T>, "InverseTransform not specialized for this type, please implement it.");
}

template <typename T, std::size_t N>
void InverseTransform(T& ioObjectToTransform, const SquareMat<ValueType_t<T>, N>& inTransformMatrix)
{
  return Transform(ioObjectToTransform, Inverted(inTransformMatrix));
}

template <typename T, std::size_t N>
[[nodiscard]] T Transformed(const T& inObject, const SquareMat<ValueType_t<T>, N>& inTransformMatrix)
{
  auto transformed_object = inObject;
  Transform(transformed_object, inTransformMatrix);
  return transformed_object;
}

template <typename T, std::size_t N>
[[nodiscard]] Vec<T, N> Transformed(const Vec<T, N>& inPoint, const SquareMat<T, N + 1>& inTransformMatrix)
{
  auto transformed_point = inPoint;
  Transform(transformed_point, inTransformMatrix);
  return transformed_point;
}

template <typename T, std::size_t N>
[[nodiscard]] T Transformed(const T& inObject, const Transformation<ValueType_t<T>, N>& inTransformation)
{
  auto transformed_object = inObject;
  Transform(transformed_object, inTransformation);
  return transformed_object;
}

template <typename T, std::size_t N>
[[nodiscard]] T InverseTransformed(const T& inObjectToTransform, const SquareMat<ValueType_t<T>, N>& inTransformMatrix)
{
  auto inverse_transformed_object = inObjectToTransform;
  InverseTransform(inverse_transformed_object, inTransformMatrix);
  return inverse_transformed_object;
}

template <typename T, std::size_t N>
[[nodiscard]] T InverseTransformed(const T& inObjectToTransform,
    const Transformation<ValueType_t<T>, N>& inTransformation)
{
  auto inverse_transformed_object = inObjectToTransform;
  InverseTransform(inverse_transformed_object, inTransformation);
  return inverse_transformed_object;
}

}