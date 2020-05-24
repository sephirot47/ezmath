#pragma once

#include "ez/MathForward.h"
#include "ez/MathInitializerTokens.h"
#include "ez/MathTypeTraits.h"
#include "ez/VariadicRepeat.h"
#include "ez/VecPart.h"
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
  static constexpr auto NumDimensions = N;

  constexpr explicit Vec(const MITAll<ValueType>& inMITAll) noexcept;
  constexpr explicit Vec(const MITAdditiveIdentity&) noexcept : Vec(MITAll<ValueType>(static_cast<ValueType>(0))) {}
  constexpr explicit Vec(const MITMultiplicativeIdentity&) noexcept : Vec(MITAll<ValueType>(static_cast<ValueType>(1)))
  {
  }
  constexpr Vec() noexcept : Vec(MITAll<ValueType> { static_cast<ValueType>(0) }) {}

  template <typename TOther>
  constexpr explicit Vec(const Vec<TOther, N>& inOther) noexcept;

  template <typename... TArgs,
      typename = std::enable_if_t<(
          (sizeof...(TArgs) >= 2) && (NumComponents_v<TArgs...> == N) && ((IsVec_v<TArgs> || IsNumber_v<TArgs>)&&...))>>
  constexpr explicit Vec(TArgs&&...) noexcept;

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
inline std::ostream& operator<<(std::ostream& ioLHS, const Vec<T, N>& inRHS);

// Traits
template <typename T, std::size_t N>
struct IsVec<Vec<T, N>> : std::true_type
{
};

template <typename T, std::size_t N>
constexpr Vec<T, N> FromTo(const Vec<T, N>& inFrom, const Vec<T, N>& inTo);

template <typename T>
constexpr bool IsVeryParallel(const Vec3<T>& inDirection0, const Vec3<T>& inDirection1);

template <typename T>
constexpr bool IsVeryPerpendicular(const Vec3<T>& inDirection0, const Vec3<T>& inDirection1);

template <typename T, std::size_t N>
constexpr auto Inverted(const Vec<T, N>& inValue);

template <typename T>
constexpr std::tuple<Vec3<T>, Vec3<T>, Vec3<T>> Axes(const Vec3<T>& inForwardVectorNormalized,
    const Vec3<T>& inUpVectorNormalized);

template <typename T>
constexpr Vec3<T> Cross(const Vec3<T>& inLHS, const Vec3<T>& inRHS);

template <typename T>
constexpr auto Right();

template <typename T>
constexpr auto Up();

template <typename T>
constexpr auto Forward();

template <typename T>
constexpr T Left();

template <typename T>
constexpr T Down();

template <typename T>
constexpr T Back();
}

#include "ez/Vec.tcc"