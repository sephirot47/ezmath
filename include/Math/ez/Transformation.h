#pragma once

#include "ez/Mat.h"
#include "ez/MathForward.h"
#include "ez/MathInitializers.h"
#include "ez/Quat.h"
#include "ez/Vec.h"

namespace ez
{
template <typename T, std::size_t N>
class Transformation final
{
public:
  static_assert(N == 2 || N == 3);
  using ValueType = T;
  using RotationType = RotationType_t<T, N>;
  static constexpr auto Dimensions = N;

  Transformation() = default;
  explicit Transformation(const Vec<T, N>& inPosition);
  Transformation(const Vec<T, N>& inPosition, const RotationType& inRotation);
  Transformation(const Vec<T, N>& inPosition, const RotationType& inRotation, const Vec<T, N>& inScale);

  void SetPosition(const Vec<T, N>& inPosition) { mPosition = inPosition; }
  void Translate(const Vec<T, N>& inPosition) { mPosition += inPosition; }
  const Vec<T, N>& GetPosition() const { return mPosition; }

  void SetRotation(const RotationType& inRotation) { mRotation = inRotation; }
  void Rotate(const RotationType& inRotation) { mRotation = Rotated(mRotation, inRotation); }
  const RotationType& GetRotation() const { return mRotation; }

  void SetScale(const Vec<T, N>& inScale) { mScale = inScale; }
  void Scale(const T& inScale) { mScale *= inScale; }
  void Scale(const Vec<T, N>& inScale) { mScale *= inScale; }
  const Vec<T, N>& GetScale() const { return mScale; }

  Transformation<T, N> operator*(const Transformation<T, N>& inRHS) const;
  Transformation<T, N> operator*=(const Transformation<T, N>& inRHS) { return (*this = (*this) * inRHS); }

  // clang-format off
  template <typename TAux = T>
  std::enable_if_t<N >= 2, Vec<TAux, N>> GetLeft() const { return Rotated(Left<Vec<T, N>>(), mRotation); }
  template <typename TAux = T>
  std::enable_if_t<N >= 2, Vec<TAux, N>> GetRight() const { return Rotated(Right<Vec<T, N>>(), mRotation); }
  template <typename TAux = T>
  std::enable_if_t<N >= 2, Vec<TAux, N>> GetDown() const { return Rotated(Down<Vec<T, N>>(), mRotation); }
  template <typename TAux = T>
  std::enable_if_t<N >= 2, Vec<TAux, N>> GetUp() const { return Rotated(Up<Vec<T, N>>(), mRotation); }
  template <typename TAux = T>
  std::enable_if_t<N >= 3, Vec<TAux, N>> GetForward() const { return Rotated(Forward<Vec<T, N>>(), mRotation); }
  template <typename TAux = T>
  std::enable_if_t<N >= 3, Vec<TAux, N>> GetBack() const { return Rotated(Back<Vec<T, N>>(), mRotation); }
  // clang-format on

  Vec<T, N> TransformedPoint(const Vec<T, N>& inPoint) const;
  Vec<T, N> InverseTransformedPoint(const Vec<T, N>& inPoint) const;
  Vec<T, N> TransformedDirection(const Vec<T, N>& inDirection) const;
  Vec<T, N> InverseTransformedDirection(const Vec<T, N>& inDirection) const;
  Transformation<T, N> TransformedTransform(const Transformation<T, N>& inTransformation) const;
  Transformation<T, N> InverseTransformedTransform(const Transformation<T, N>& inTransformation) const;
  SquareMat<T, N + 1> GetMatrix() const;
  SquareMat<T, N + 1> GetInverseMatrix() const;

private:
  Vec<T, N> mPosition = Zero<Vec<T, N>>();
  RotationType mRotation = Identity<RotationType>();
  Vec<T, N> mScale = One<Vec<T, N>>();
};

template <typename T, std::size_t N>
inline std::ostream& operator<<(std::ostream& ioLHS, const Transformation<T, N>& inRHS);

// Traits
template <typename T, std::size_t N>
struct IsTransformation<Transformation<T, N>> : std::true_type
{
};

// Implement these 4 "Transform" methods for specialization (see Ray.h for an example)
template <typename T, std::size_t N>
void Transform(T& ioObjectToTransform, const SquareMat<ValueType_t<T>, N>& inTransformMatrix);

template <typename T, std::size_t N>
void Transform(T& ioObjectToTransform, const SquareMat<ValueType_t<T>, N + 1>& inTransformMatrix);

template <typename T, std::size_t N>
void Transform(T& ioObjectToTransform, const Transformation<ValueType_t<T>, N>& inTransformation);

template <typename T, std::size_t N>
void InverseTransform(T& ioObjectToTransform, const Transformation<ValueType_t<T>, N>& inTransformation);
// =====

// Inverse that can be generically implemented
template <typename T, std::size_t N>
void InverseTransform(T& ioObjectToTransform, const SquareMat<ValueType_t<T>, N>& inTransformMatrix);
// ======

// Transformed (return) versions
template <typename T, std::size_t N>
[[nodiscard]] T Transformed(const T& inObjectToTransform, const SquareMat<ValueType_t<T>, N>& inTransformMatrix);

template <typename T, std::size_t N>
[[nodiscard]] T Transformed(const T& inObjectToTransform, const Transformation<ValueType_t<T>, N>& inTransformation);

template <typename T, std::size_t N>
[[nodiscard]] T InverseTransformed(const T& inObjectToTransform, const SquareMat<ValueType_t<T>, N>& inTransformMatrix);

template <typename T, std::size_t N>
[[nodiscard]] T InverseTransformed(const T& inObjectToTransform,
    const Transformation<ValueType_t<T>, N>& inTransformation);
// =====
}

#include "ez/Transformation.tcc"