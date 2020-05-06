#pragma once

#include "ez/MathInitializerTokens.h"
#include "ez/MathTypeTraits.h"
#include "ez/VariadicRepeat.h"
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

template <typename T, std::size_t N>
using SquareMat = Mat<T, N, N>;

template <typename T>
using Mat2 = Mat<T, 2, 2>;
using Mat2f = Mat2<float>;
using Mat2d = Mat2<double>;
using Mat2i = Mat2<int32_t>;

template <typename T>
using Mat3 = Mat<T, 3, 3>;
using Mat3f = Mat3<float>;
using Mat3d = Mat3<double>;
using Mat3i = Mat3<int32_t>;

template <typename T>
using Mat4 = Mat<T, 4, 4>;
using Mat4f = Mat4<float>;
using Mat4d = Mat4<double>;
using Mat4i = Mat4<int32_t>;

template <typename T, std::size_t N>
struct IsMat<Mat<T, N, N>> final
{
  static constexpr bool value = true;
};

}

#include "ez/Mat.tcc"