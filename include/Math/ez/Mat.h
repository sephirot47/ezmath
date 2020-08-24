#pragma once

#include <ez/MathForward.h>
#include <ez/MathInitializerTokens.h>
#include <ez/MathTypeTraits.h>
#include <ez/VariadicRepeat.h>
#include <array>
#include <cstdint>
#include <initializer_list>
#include <type_traits>

namespace ez
{

template <typename, std::size_t>
class Vec;

template <typename T, std::size_t NRows, std::size_t NCols>
class Mat final
{
public:
  using ValueType = T;
  static constexpr auto NumRows = NRows;
  static constexpr auto NumCols = NCols;
  static constexpr auto NumComponents = NRows;

  constexpr explicit Mat(const MITAll<ValueType>& inMITAll) noexcept;
  constexpr explicit Mat(const MITMultiplicativeIdentity&) noexcept;
  constexpr explicit Mat(const MITAdditiveIdentity&) noexcept : Mat(MITAll<ValueType>(0)) {}
  constexpr Mat() noexcept : Mat(MITAll<ValueType> { static_cast<ValueType>(0) }) {}

  template <typename... TArgs, typename = std::enable_if_t<(sizeof...(TArgs) > 1)>>
  constexpr explicit Mat(TArgs&&... inArgs) noexcept;

  T* Data();
  const T* Data() const;

  // Iterators
  constexpr typename std::array<Vec<T, NCols>, NRows>::iterator begin();
  constexpr typename std::array<Vec<T, NCols>, NRows>::iterator end();
  constexpr typename std::array<Vec<T, NCols>, NRows>::const_iterator begin() const;
  constexpr typename std::array<Vec<T, NCols>, NRows>::const_iterator end() const;
  constexpr typename std::array<Vec<T, NCols>, NRows>::const_iterator cbegin() const;
  constexpr typename std::array<Vec<T, NCols>, NRows>::const_iterator cend() const;

  // Operators
  constexpr bool operator==(const Mat& inRHS) const;
  constexpr bool operator!=(const Mat& inRHS) const;
  constexpr Vec<T, NCols>& operator[](std::size_t inRow);
  constexpr const Vec<T, NCols>& operator[](std::size_t inRow) const;
  constexpr Mat<T, NRows, NCols> operator+(const Mat<T, NRows, NCols>& inRHS) const;
  constexpr Mat<T, NRows, NCols> operator-(const Mat<T, NRows, NCols>& inRHS) const;
  void operator+=(const Mat<T, NRows, NCols>& inRHS);
  void operator-=(const Mat<T, NRows, NCols>& inRHS);
  constexpr Mat<T, NRows, NCols> operator+(const T& inRHS) const;
  constexpr Mat<T, NRows, NCols> operator-(const T& inRHS) const;
  constexpr Mat<T, NRows, NCols> operator*(const T& inRHS) const;
  constexpr Mat<T, NRows, NCols> operator/(const T& inRHS) const;
  constexpr Vec<T, NCols> operator*(const Vec<T, NCols>& inRHS) const;
  void operator+=(const T& inRHS);
  void operator-=(const T& inRHS);
  void operator*=(const T& inRHS);
  void operator/=(const T& inRHS);
  constexpr Mat<T, NRows, NCols> operator-() const;
  template <std::size_t TRHSCols>
  constexpr Mat<T, NRows, TRHSCols> operator*(const Mat<T, NCols, TRHSCols>& inRHS) const;

private:
  std::array<Vec<T, NCols>, NRows> mRows;
};

template <typename T, std::size_t NRows, std::size_t NCols>
inline std::ostream& operator<<(std::ostream& inLHS, const Mat<T, NRows, NCols>& inRHS);

// Traits
template <typename T, std::size_t N>
struct IsMat<Mat<T, N, N>> final : std::true_type
{
};

template <typename T, std::size_t N, std::size_t NCols>
constexpr Vec<T, N> Column(const Mat<T, N, NCols>& inMat, const std::size_t inColumn);

template <typename T, std::size_t N, std::size_t NCols>
constexpr Vec<T, N> Row(const Mat<T, N, NCols>& inMat, const std::size_t inRow);

template <std::size_t IColumn, typename T, std::size_t N, std::size_t NCols>
constexpr Vec<T, N> Column(const Mat<T, N, NCols>& inMat);

template <std::size_t IRow, typename T, std::size_t N, std::size_t NCols>
constexpr Vec<T, N> Row(const Mat<T, N, NCols>& inMat);

template <typename T, std::size_t N>
constexpr SquareMat<T, N> Transposed(const SquareMat<T, N>& inMat);

template <typename T, std::size_t N>
constexpr SquareMat<T, N - 1>
Cofactor(const SquareMat<T, N>& inMat, const std::size_t inRowToRemove, const std::size_t inColumnToRemove);

template <typename T, std::size_t N>
constexpr SquareMat<T, N> Adjoint(const SquareMat<T, N>& inMat);

template <typename T, std::size_t N>
constexpr T Determinant(const SquareMat<T, N>& inMat);

template <typename T>
constexpr auto Inverted(const T& inValue);

template <typename T>
constexpr auto Translation(const SquareMat<T, 3>& inMat);

template <typename T>
constexpr auto Translation(const SquareMat<T, 4>& inMat);

template <typename T>
constexpr auto Scale(const SquareMat<T, 3>& inMat);

template <typename T>
constexpr auto Scale(const SquareMat<T, 4>& inMat);

template <typename T, std::size_t N>
constexpr SquareMat<T, N> NormalMat(const SquareMat<T, N>& inModelViewMatrix);

template <typename T, std::size_t N>
constexpr SquareMat<T, N + 1> TranslationMat(const Vec<T, N>& inTranslation);

template <typename T>
constexpr std::enable_if_t<IsNumber_v<T>, SquareMat<T, 3>> RotationMat(const T inAngle);

template <typename T>
constexpr SquareMat<T, 4> RotationMat(const Quat<T>& inRotation);

template <typename T, std::size_t N>
constexpr SquareMat<T, N + 1> ScaleMat(const Vec<T, N>& inScale);

template <typename T>
constexpr Mat4<T> PerspectiveMat(const T inAngleOfViewRads, const T inAspectRatio, const T inZNear, const T inZFar);

template <typename T, std::size_t N>
constexpr SquareMat<T, N + 1> OrthographicMat(const Vec<T, N>& inMin, const Vec<T, N>& inMax);

template <typename T>
constexpr T Diagonal(const ValueType_t<T>& inDiagonalValue);
}

#include "ez/Mat.tcc"