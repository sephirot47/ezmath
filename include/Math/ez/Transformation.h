#pragma once

#include "ez/Mat.h"
#include "ez/MathInitializers.h"
#include "ez/Quat.h"
#include "ez/Vec.h"

namespace ez
{
// clang-format off
template <typename TTransformation> struct TransformationRotationType { using Type = void; };
// clang-format on

template <typename T, std::size_t N>
class Transformation final
{
public:
  using ValueType = T;
  using RotationType = typename TransformationRotationType<Transformation>::Type;
  static constexpr auto Dimensions = N;

  Transformation() = default;
  explicit Transformation(const Vec<T, N>& inPosition);
  Transformation(const Vec<T, N>& inPosition, const RotationType& inRotation);
  Transformation(const Vec<T, N>& inPosition, const RotationType& inRotation, const Vec<T, N>& inScale);

  void SetPosition(const Vec<T, N>& inPosition) { mPosition = inPosition; }
  const Vec<T, N>& GetPosition() const { return mPosition; }

  void SetRotation(const RotationType& inRotation) { mRotation = inRotation; }
  const RotationType& GetRotation() const { return mRotation; }

  void SetScale(const Vec<T, N>& inScale) { mScale = inScale; }
  const Vec<T, N>& GetScale() const { return mScale; }

  Vec<T, N> Transformed(const Vec<T, N>& inPoint);
  Vec<T, N> InverseTransformed(const Vec<T, N>& inPoint);
  SquareMat<T, N + 1> GetMatrix() const;
  SquareMat<T, N + 1> GetInverseMatrix() const;

private:
  Vec<T, N> mPosition = Zero<Vec<T, N>>();
  RotationType mRotation = Identity<RotationType>();
  Vec<T, N> mScale = One<Vec<T, N>>();
};

template <typename T>
using Transformation2 = Transformation<T, 2>;
using Transformation2f = Transformation2<float>;

template <typename T>
using Transformation3 = Transformation<T, 3>;
using Transformation3f = Transformation3<float>;

// clang-format off
template <typename T> struct TransformationRotationType<Transformation2<T>> { using Type = T; };
template <typename T> struct TransformationRotationType<Transformation3<T>> { using Type = Quat<T>; };
// clang-format on

// Helper functions
template <typename TObject, typename TTransformIterator>
TTransformIterator GetTransformIteratorBegin(TObject& ioObject);

template <typename TObject, typename TTransformIterator>
TTransformIterator GetTransformIteratorEnd(TObject& ioObject);

template <typename T, std::size_t N>
void Transform(Vec<T, N>& ioPoint, const SquareMat<T, N>& inTransformMatrix);

template <typename T, std::size_t N>
[[nodiscard]] Vec<T, N> Transformed(const Vec<T, N>& inPoint, const SquareMat<T, N>& inTransformMatrix);

template <typename T, std::size_t N>
void Transform(Vec<T, N>& ioPoint, const SquareMat<T, N + 1>& inTransformMatrix);

template <typename T, std::size_t N>
[[nodiscard]] Vec<T, N> Transformed(const Vec<T, N>& inPoint,
    const SquareMat<ValueType_t<T>, N + 1>& inTransformMatrix);

template <typename T, std::size_t N>
void Transform(T& ioObjectToTransform, const SquareMat<ValueType_t<T>, N>& inTransformMatrix);

template <typename T, std::size_t N>
[[nodiscard]] T Transformed(const T& inObjectToTransform, const SquareMat<ValueType_t<T>, N>& inTransformMatrix);

template <typename T, std::size_t N>
void Transform(T& ioObjectToTransform, const Transformation<ValueType_t<T>, N>& inTransformation);

template <typename T, std::size_t N>
[[nodiscard]] T Transformed(const T& inObjectToTransform, const Transformation<ValueType_t<T>, N>& inTransformation);

}

#include "ez/Transformation.tcc"