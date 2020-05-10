#pragma once

#include "ez/Mat.h"
#include "ez/MathInitializerTokens.h"
#include "ez/MathTypeTraits.h"
#include "ez/Vec.h"
#include <array>
#include <ostream>

namespace ez
{

template <typename T>
class Quat final
{
public:
  using ValueType = T;
  static constexpr std::size_t NumComponents = 4;

  constexpr Quat(const MITMultiplicativeIdentity&) noexcept : mComponents { 0, 0, 0, 1 } {}
  constexpr Quat(const T& inX, const T& inY, const T& inZ, const T& inW) noexcept : mComponents { inX, inY, inZ, inW }
  {
  }
  constexpr Quat() noexcept : Quat(MITMultiplicativeIdentity()) {}

  // Iterators
  constexpr typename std::array<T, 4>::iterator begin();
  constexpr typename std::array<T, 4>::iterator end();
  constexpr typename std::array<T, 4>::const_iterator begin() const;
  constexpr typename std::array<T, 4>::const_iterator end() const;
  constexpr typename std::array<T, 4>::const_iterator cbegin() const;
  constexpr typename std::array<T, 4>::const_iterator cend() const;

  // Operators
  constexpr bool operator==(const Quat<T>& inRHS) const;
  constexpr bool operator!=(const Quat<T>& inRHS) const;
  constexpr T& operator[](std::size_t i);
  constexpr const T& operator[](std::size_t i) const;
  constexpr Quat<T> operator+(const Quat<T>& inRHS) const;
  constexpr Quat<T> operator-(const Quat<T>& inRHS) const;
  constexpr Quat<T> operator*(const T& inRHS) const;
  constexpr Quat<T> operator*(const Quat<T>& inRHS) const;
  constexpr Vec3<T> operator*(const Vec3<T>& inRHS) const;
  constexpr Vec4<T> operator*(const Vec4<T>& inRHS) const;
  constexpr Quat<T> operator/(const T& inRHS) const;
  void operator+=(const Quat<T>& inRHS);
  void operator-=(const Quat<T>& inRHS);
  void operator*=(const T& inRHS);
  void operator*=(const Quat<T>& inRHS);
  void operator/=(const T& inRHS);
  constexpr Quat<T> operator-() const;

private:
  std::array<T, 4> mComponents; // x, y, z, w
};

template <typename T>
Quat<T> operator*(const T& a, const Quat<T>& inRHS);

template <typename T>
Quat<T> operator/(const T& a, const Quat<T>& inRHS);

template <typename T>
std::ostream& operator<<(std::ostream& log, const Quat<T>& q);

using Quati = Quat<int>;
using Quatf = Quat<float>;
using Quatd = Quat<double>;

template <typename T>
struct IsQuat<Quat<T>> : std::true_type
{
};
}

#include "ez/Quat.tcc"
