#pragma once

#include "ez/MathInitializerTokens.h"
#include "ez/MathTypeTraits.h"
#include "ez/VariadicRepeat.h"
#include <array>
#include <cstdint>
#include <initializer_list>
#include <iostream>
#include <ostream>
#include <type_traits>

namespace ez
{
template <typename T, std::size_t N>
class Vec final
{
public:
  static_assert(N >= 1);

  using ValueType = T;
  static constexpr auto NumComponents = N;

  constexpr explicit Vec(const MITAll<ValueType>& inMITAll) noexcept;
  constexpr explicit Vec(const MITAdditiveIdentity&) noexcept
      : Vec(MITAll<ValueType>(static_cast<ValueType>(0)))
  {
  }
  constexpr explicit Vec(const MITMultiplicativeIdentity&) noexcept
      : Vec(MITAll<ValueType>(static_cast<ValueType>(1)))
  {
  }
  constexpr Vec() noexcept : Vec(MITAll<ValueType> { static_cast<ValueType>(0) }) {}

  template <typename... TArgs, typename = std::enable_if_t<sizeof...(TArgs) == N>>
  constexpr explicit Vec(TArgs&&... inArgs) noexcept;

  constexpr Vec(const Vec&) noexcept = default;
  constexpr Vec& operator=(const Vec&) noexcept = default;

  constexpr Vec(Vec&&) noexcept = default;
  constexpr Vec& operator=(Vec&&) noexcept = default;

  ~Vec() = default;

  T* Data();
  const T* Data() const;

  // Iterators
  constexpr typename std::array<T, N>::iterator begin();
  constexpr typename std::array<T, N>::iterator end();
  constexpr typename std::array<T, N>::const_iterator begin() const;
  constexpr typename std::array<T, N>::const_iterator end() const;
  constexpr typename std::array<T, N>::const_iterator cbegin() const;
  constexpr typename std::array<T, N>::const_iterator cend() const;

  // Operators
  constexpr bool operator==(const Vec& inRHS) const;
  constexpr bool operator!=(const Vec& inRHS) const;
  constexpr bool operator>(const Vec& inRHS) const;
  constexpr bool operator>=(const Vec& inRHS) const;
  constexpr bool operator<(const Vec& inRHS) const;
  constexpr bool operator<=(const Vec& inRHS) const;
  constexpr T& operator[](std::size_t i);
  constexpr const T& operator[](std::size_t i) const;
  constexpr Vec<T, N> operator+(const Vec& inRHS) const;
  constexpr Vec<T, N> operator-(const Vec& inRHS) const;
  constexpr Vec<T, N> operator*(const Vec& inRHS) const;
  constexpr Vec<T, N> operator/(const Vec& inRHS) const;
  constexpr Vec<T, N> operator+(const T& inRHS) const;
  constexpr Vec<T, N> operator-(const T& inRHS) const;
  constexpr Vec<T, N> operator*(const T& inRHS) const;
  constexpr Vec<T, N> operator/(const T& inRHS) const;
  constexpr void operator+=(const Vec& inRHS);
  constexpr void operator-=(const Vec& inRHS);
  constexpr void operator*=(const Vec& inRHS);
  constexpr void operator/=(const Vec& inRHS);
  constexpr void operator+=(const T& inRHS);
  constexpr void operator-=(const T& inRHS);
  constexpr void operator*=(const T& inRHS);
  constexpr void operator/=(const T& inRHS);
  constexpr Vec<T, N> operator-() const;

protected:
  std::array<T, N> mComponents;
};

template <typename T, std::size_t N>
inline constexpr Vec<T, N> operator+(const T& inLHS, const Vec<T, N>& inRHS);
template <typename T, std::size_t N>
inline constexpr Vec<T, N> operator-(const T& inLHS, const Vec<T, N>& inRHS);
template <typename T, std::size_t N>
inline constexpr Vec<T, N> operator*(const T& inLHS, const Vec<T, N>& inRHS);
template <typename T, std::size_t N>
inline constexpr Vec<T, N> operator/(const T& inLHS, const Vec<T, N>& inRHS);
template <typename T, std::size_t N>
inline std::ostream& operator<<(std::ostream& inLHS, const Vec<T, N>& inRHS);

template <typename T>
using Vec2 = Vec<T, 2>;
using Vec2b = Vec2<bool>;
using Vec2i = Vec2<int32_t>;
using Vec2u = Vec2<uint32_t>;
using Vec2f = Vec2<float>;
using Vec2d = Vec2<double>;

template <typename T>
using Vec3 = Vec<T, 3>;
using Vec3b = Vec3<bool>;
using Vec3i = Vec3<int32_t>;
using Vec3u = Vec3<uint32_t>;
using Vec3f = Vec3<float>;
using Vec3d = Vec3<double>;

template <typename T>
using Vec4 = Vec<T, 4>;
using Vec4b = Vec4<bool>;
using Vec4i = Vec4<int32_t>;
using Vec4u = Vec4<uint32_t>;
using Vec4f = Vec4<float>;
using Vec4d = Vec4<double>;

template <typename T, std::size_t N>
struct IsVec<Vec<T, N>>
{
  static constexpr bool value = true;
};
}

#include "ez/Vec.tcc"