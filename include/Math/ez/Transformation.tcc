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
Vec<T, N> Transformation<T, N>::Transformed(const Vec<T, N>& inPoint)
{
  return mPosition + (Rotated((inPoint * mScale), mRotation));
}

template <typename T, std::size_t N>
Vec<T, N> Transformation<T, N>::InverseTransformed(const Vec<T, N>& inPoint)
{
  return Rotated((inPoint - mPosition), -mRotation) / mScale;
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

template <typename T>
using HasTransformIteratorDetector = decltype(std::declval<T>().GetTransformIteratorBegin());

template <typename T>
static constexpr auto HasTransformIterator_v = std::experimental::is_detected_v<HasTransformIteratorDetector, T>;

template <typename TObject>
auto GetTransformIteratorBegin(TObject& ioObject)
{
  if constexpr (HasTransformIterator_v<TObject>)
  {
    return ioObject.GetTransformIteratorBegin();
  }
  else
  {
    return ioObject.begin();
  }
}

template <typename TObject>
auto GetTransformIteratorEnd(TObject& ioObject)
{
  if constexpr (HasTransformIterator_v<TObject>)
  {
    return ioObject.GetTransformIteratorEnd();
  }
  else
  {
    return ioObject.end();
  }
}

template <typename T, std::size_t N>
void Transform(Vec<T, N>& ioPoint, const SquareMat<T, N>& inTransformMatrix)
{
  ioPoint = (inTransformMatrix * ioPoint);
}

template <typename T, std::size_t N>
[[nodiscard]] Vec<T, N> Transformed(const Vec<T, N>& inPoint, const SquareMat<T, N>& inTransformMatrix)
{
  auto transformed_point = inPoint;
  Transform(transformed_point, inTransformMatrix);
  return transformed_point;
}

template <typename T, std::size_t N>
void Transform(Vec<T, N>& ioPoint, const SquareMat<T, N + 1>& inTransformMatrix)
{
  // If the point has one less dimension than the transform matrix, convert the point to one
  // more dimension by adding a 1 at the end, and then retrieve it from the result

  Vec<T, N + 1> point_and_1;
  for (std::size_t i = 0; i < N; ++i) { point_and_1[i] = ioPoint[i]; }
  point_and_1[N] = static_cast<T>(1);

  const auto transformed_point_and_1 = Transformed(point_and_1, inTransformMatrix);
  for (std::size_t i = 0; i < N; ++i) { ioPoint[i] = transformed_point_and_1[i]; }
}

template <typename T, std::size_t N>
[[nodiscard]] Vec<T, N> Transformed(const Vec<T, N>& inPoint, const SquareMat<T, N + 1>& inTransformMatrix)
{
  auto transformed_point = inPoint;
  Transform(transformed_point, inTransformMatrix);
  return transformed_point;
}

template <typename T, std::size_t N>
void Transform(T& ioObject, const SquareMat<ValueType_t<T>, N>& inTransformMatrix)
{
  for (auto it = GetTransformIteratorBegin(ioObject); it != GetTransformIteratorEnd(ioObject); ++it)
  {
    auto& value = *it;
    Transform(value, inTransformMatrix);
  }
}

template <typename T, std::size_t N>
[[nodiscard]] T Transformed(const T& inObject, const SquareMat<ValueType_t<T>, N>& inTransformMatrix)
{
  auto transformed_object = inObject;
  Transform(transformed_object, inTransformMatrix);
  return transformed_object;
}

template <typename T, std::size_t N>
void Transform(T& ioObject, const Transformation<ValueType_t<T>, N>& inTransformation)
{
  const auto transform_matrix = inTransformation.GetMatrix();
  for (auto it = GetTransformIteratorBegin(ioObject); it != GetTransformIteratorEnd(ioObject); ++it)
  {
    auto& value = *it;
    Transform(value, transform_matrix);
  }
}

template <typename T, std::size_t N>
[[nodiscard]] T Transformed(const T& inObject, const Transformation<ValueType_t<T>, N>& inTransformation)
{
  auto transformed_object = inObject;
  Transform(transformed_object, inTransformation);
  return transformed_object;
}

}