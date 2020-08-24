#pragma once

#include <ez/Mat.h>
#include <ez/MathForward.h>
#include <ez/MathInitializerTokens.h>
#include <ez/MathTypeTraits.h>
#include <ez/Vec.h>
#include <array>
#include <ostream>
#include <tuple>

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

// Operators
template <typename T>
Quat<T> operator*(const T& a, const Quat<T>& inRHS);

template <typename T>
Quat<T> operator/(const T& a, const Quat<T>& inRHS);

template <typename T>
std::ostream& operator<<(std::ostream& log, const Quat<T>& q);

// Traits
template <typename T>
struct IsQuat<Quat<T>> : std::true_type
{
};

template <typename T, std::size_t N>
using RotationType_t = std::conditional_t<N == 2, T, Quat<T>>;

template <typename T, std::size_t N>
constexpr auto RotationTypeIdentity();

// Math functions
template <typename T>
constexpr Quat<T> ToQuaternion(const Mat4<T>& inRotationMat);

template <typename T>
constexpr Vec3<T> Direction(const Quat<T>& inQuat);

template <typename T>
constexpr T Pitch(const Quat<T>& inQuat);

template <typename T>
constexpr T Yaw(const Quat<T>& inQuat);

template <typename T>
constexpr T Roll(const Quat<T>& inQuat);

template <typename T>
constexpr Vec3<T> AngleAxis(const Quat<T>& inQuat);

template <typename T>
constexpr Quat<T> Conjugated(const Quat<T>& inQuat);

template <typename T>
constexpr Vec3<T> Forward(const Quat<T>& inQuat);

template <typename T>
constexpr Quat<T> AngleAxis(const T& inAngleRads, const Vec3<T>& inAxisNormalized);

template <typename T>
constexpr Quat<T> FromEulerAngles(const Vec3<T>& inEulerAnglesRads);

template <typename T>
constexpr Quat<T> FromTo(const Vec3<T>& inFromNormalized, const Vec3<T>& inToNormalized);

template <typename T>
constexpr Quat<T> LookInDirection(const Vec3<T>& inForwardNormalized, const Vec3<T>& inUpNormalized = Up<Vec3<T>>());

template <typename T>
constexpr std::tuple<Vec3<T>, Vec3<T>, Vec3<T>> Axes(const Quat<T>& inOrientation);

template <typename T>
constexpr Vec2<T> Rotated(const Vec2<T>& inVec, const AngleRads<T> inAngle);

template <typename T>
constexpr Vec3<T> Rotated(const Vec3<T>& inVec, const Quat<T>& inRotation);

template <typename T>
constexpr AngleRads<T> Rotated(const AngleRads<T> inLHS, const AngleRads<T> inRHS);

template <typename T>
constexpr Quat<T> Rotated(const Quat<T>& inLHS, const Quat<T>& inRHS);

template <typename T, typename TQuat>
constexpr TQuat SLerp(const TQuat& inFrom, const TQuat& inTo, const T& inT);

template <typename T>
constexpr auto Inverted(const Quat<T>& inValue);
}

#include "ez/Quat.tcc"
