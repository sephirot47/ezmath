#include "ez/Macros.h"
#include "ez/Quat.h"
#include <cmath>

namespace ez
{
template <typename T>
constexpr typename std::array<T, 4>::iterator Quat<T>::begin()
{
  return std::begin(mComponents);
}

template <typename T>
constexpr typename std::array<T, 4>::iterator Quat<T>::end()
{
  return std::end(mComponents);
}
template <typename T>
constexpr typename std::array<T, 4>::const_iterator Quat<T>::begin() const
{
  return std::begin(mComponents);
}

template <typename T>
constexpr typename std::array<T, 4>::const_iterator Quat<T>::end() const
{
  return std::end(mComponents);
}

template <typename T>
constexpr typename std::array<T, 4>::const_iterator Quat<T>::cbegin() const
{
  return std::cbegin(mComponents);
}

template <typename T>
constexpr typename std::array<T, 4>::const_iterator Quat<T>::cend() const
{
  return std::cend(mComponents);
}

template <typename T>
constexpr bool Quat<T>::operator==(const Quat<T>& inRHS) const
{
  return mComponents == inRHS.mComponents;
}

template <typename T>
constexpr bool Quat<T>::operator!=(const Quat<T>& inRHS) const
{
  return mComponents != inRHS.mComponents;
}

template <typename T>
constexpr T& Quat<T>::operator[](std::size_t i)
{
  return mComponents[i];
}

template <typename T>
constexpr const T& Quat<T>::operator[](std::size_t i) const
{
  return mComponents[i];
}

template <typename T>
constexpr Quat<T> Quat<T>::operator+(const Quat<T>& inRHS) const
{
  return { (*this)[0] + inRHS[0], (*this)[1] + inRHS[1], (*this)[2] + inRHS[2], (*this)[3] + inRHS[3] };
}

template <typename T>
constexpr Quat<T> Quat<T>::operator-(const Quat<T>& inRHS) const
{
  return { (*this)[0] - inRHS[0], (*this)[1] - inRHS[1], (*this)[2] - inRHS[2], (*this)[3] - inRHS[3] };
}

template <typename T>
constexpr Quat<T> Quat<T>::operator*(const T& inRHS) const
{
  return { (*this)[0] * inRHS[0], (*this)[1] * inRHS[1], (*this)[2] * inRHS[2], (*this)[3] * inRHS[3] };
}

template <typename T>
constexpr Quat<T> Quat<T>::operator*(const Quat<T>& inRHS) const
{
  return { (*this)[3] * inRHS[0] + (*this)[0] * inRHS[3] + (*this)[1] * inRHS[2] - (*this)[2] * inRHS[1],
    (*this)[3] * inRHS[1] + (*this)[1] * inRHS[3] + (*this)[2] * inRHS[0] - (*this)[0] * inRHS[2],
    (*this)[3] * inRHS[2] + (*this)[2] * inRHS[3] + (*this)[0] * inRHS[1] - (*this)[1] * inRHS[0],
    (*this)[3] * inRHS[3] - (*this)[0] * inRHS[0] - (*this)[1] * inRHS[1] - (*this)[2] * inRHS[2] };
}

template <typename T>
constexpr Vec3<T> Quat<T>::operator*(const Vec3<T>& inRHS) const
{
  const auto q_vector = Vec3<T> { (*this)[0], (*this)[1], (*this)[2] };
  const auto uv(Cross(q_vector, inRHS));
  const auto uuv(Cross(q_vector, uv));
  return inRHS + ((uv * (*this)[3]) + uuv) * static_cast<T>(2);
}

template <typename T>
constexpr Vec4<T> Quat<T>::operator*(const Vec4<T>& inRHS) const
{
  const auto v3 = (*this) * Vec3<T> { inRHS[0], inRHS[1], inRHS[2] };
  return { v3[0], v3[1], v3[2], inRHS[3] };
}

template <typename T>
constexpr Quat<T> Quat<T>::operator/(const T& inRHS) const
{
  return { (*this)[0] / inRHS, (*this)[1] / inRHS, (*this)[2] / inRHS, (*this)[3] / inRHS };
}

template <typename T>
void Quat<T>::operator+=(const Quat<T>& inRHS)
{
  *this = *this + inRHS;
}

template <typename T>
void Quat<T>::operator-=(const Quat<T>& inRHS)
{
  *this = *this - inRHS;
}

template <typename T>
void Quat<T>::operator*=(const T& inRHS)
{
  *this = *this * inRHS;
}

template <typename T>
void Quat<T>::operator*=(const Quat<T>& inRHS)
{
  *this = *this * inRHS;
}

template <typename T>
void Quat<T>::operator/=(const T& inRHS)
{
  *this = *this / inRHS;
}

template <typename T>
constexpr Quat<T> Quat<T>::operator-() const
{
  return { -(*this)[0], -(*this)[1], -(*this)[2], (*this)[3] };
}

template <typename T>
std::ostream& operator<<(std::ostream& log, const Quat<T>& q)
{
  log << "(" << q[0] << ", " << q[1] << ", " << q[2] << ", " << q[3] << ")";
  return log;
}

template <typename T>
constexpr Quat<T> ToQuaternion(const Mat4<T>& inRotationMat)
{
  const auto& m = inRotationMat;
  const auto fourXSquaredMinus1 = m[0][0] - m[1][1] - m[2][2];
  const auto fourYSquaredMinus1 = m[1][1] - m[0][0] - m[2][2];
  const auto fourZSquaredMinus1 = m[2][2] - m[0][0] - m[1][1];
  const auto fourWSquaredMinus1 = m[0][0] + m[1][1] + m[2][2];

  auto biggestIndex = 0;
  auto fourBiggestSquaredMinus1 = fourXSquaredMinus1;
  if (fourYSquaredMinus1 > fourBiggestSquaredMinus1)
  {
    fourBiggestSquaredMinus1 = fourYSquaredMinus1;
    biggestIndex = 1;
  }
  if (fourZSquaredMinus1 > fourBiggestSquaredMinus1)
  {
    fourBiggestSquaredMinus1 = fourZSquaredMinus1;
    biggestIndex = 2;
  }
  if (fourWSquaredMinus1 > fourBiggestSquaredMinus1)
  {
    fourBiggestSquaredMinus1 = fourWSquaredMinus1;
    biggestIndex = 3;
  }

  const auto biggestVal = std::sqrt(fourBiggestSquaredMinus1 + static_cast<T>(1)) * static_cast<T>(0.5);
  const auto mult = static_cast<T>(0.25) / biggestVal;
  switch (biggestIndex)
  {
  case 0:
    return Quat<T> { biggestVal, (m[1][0] + m[0][1]) * mult, (m[0][2] + m[2][0]) * mult, (m[2][1] - m[1][2]) * mult };

  case 1:
    return Quat<T> { (m[1][0] + m[0][1]) * mult, biggestVal, (m[2][1] + m[1][2]) * mult, (m[0][2] - m[2][0]) * mult };

  case 2:
    return Quat<T> { (m[0][2] + m[2][0]) * mult, (m[2][1] + m[1][2]) * mult, biggestVal, (m[1][0] - m[0][1]) * mult };

  default:
    return Quat<T> { (m[2][1] - m[1][2]) * mult, (m[0][2] - m[2][0]) * mult, (m[1][0] - m[0][1]) * mult, biggestVal };
  }
}

template <typename T>
constexpr Vec3<T> Direction(const Quat<T>& inQuat)
{
  EXPECTS(IsNormalized(inQuat));
  const auto direction = Normalized(inQuat * Forward<Vec3<T>>());
  ENSURES(IsNormalized(direction));
  return direction;
}

template <typename T>
constexpr T Pitch(const Quat<T>& inQuat)
{
  return (std::atan2((static_cast<T>(2) * (inQuat[1] * inQuat[2] + inQuat[3] * inQuat[0])),
      (inQuat[3] * inQuat[3] - inQuat[0] * inQuat[0] - inQuat[1] * inQuat[1] + inQuat[2] * inQuat[2])));
}

template <typename T>
constexpr T Yaw(const Quat<T>& inQuat)
{
  return std::asin(Clamp(static_cast<T>(-2) * (inQuat[0] * inQuat[2] - inQuat[3] * inQuat[1]),
      static_cast<T>(-1),
      static_cast<T>(1)));
}

template <typename T>
constexpr T Roll(const Quat<T>& inQuat)
{
  return static_cast<T>(std::atan2((static_cast<T>(2) * (inQuat[0] * inQuat[1] + inQuat[3] * inQuat[2])),
      (inQuat[3] * inQuat[3] + inQuat[0] * inQuat[0] - inQuat[1] * inQuat[1] - inQuat[2] * inQuat[2])));
}

template <typename T>
constexpr Vec3<T> AngleAxis(const Quat<T>& inQuat)
{
  EXPECTS(IsNormalized(inQuat));

  const auto angle = static_cast<T>(2) * std::acos(inQuat[3]);
  const auto wSquared = inQuat[3] * inQuat[3];
  const auto oneMinusWSquared = (static_cast<T>(1) - wSquared);
  if (IsVeryEqual(oneMinusWSquared, static_cast<T>(0)))
    return Zero<Vec3<T>>();

  const auto sqrt = std::sqrt(oneMinusWSquared);
  const auto axis = Vec3<T>(inQuat[0] / sqrt, inQuat[1] / sqrt, inQuat[2] / sqrt);
  return Normalized(axis) * angle;
}

template <typename T>
constexpr Quat<T> Conjugated(const Quat<T>& inQuat)
{
  return -inQuat;
}

template <typename T>
constexpr Vec3<T> Forward(const Quat<T>& inQuat)
{
  EXPECTS(IsNormalized(inQuat));

  return Direction(inQuat);
}

template <typename T>
constexpr Quat<T> AngleAxis(const T& inAngleRads, const Vec3<T>& inAxisNormalized)
{
  EXPECTS(IsNormalized(inAxisNormalized));

  const auto half_angle = inAngleRads * static_cast<T>(0.5);
  const auto half_angle_sin = std::sin(half_angle);
  auto result_quat = Quat<T>(inAxisNormalized[0] * half_angle_sin,
      inAxisNormalized[1] * half_angle_sin,
      inAxisNormalized[2] * half_angle_sin,
      std::cos(half_angle));

  ENSURES(IsNormalized(result_quat));
  return result_quat;
}

template <typename T>
constexpr Quat<T> AxisAngle(const Vec3<T>& inAxisNormalized, const T& inAngleRads)
{
  return AngleAxis(inAngleRads, inAxisNormalized);
}

template <typename T>
constexpr Quat<T> FromEulerAngles(const Vec3<T>& inEulerAnglesRads)
{
  const auto qx = Quat<T>::AngleAxis(inEulerAnglesRads[0], Right<Vec3<T>>());
  const auto qy = Quat<T>::AngleAxis(inEulerAnglesRads[1], Up<Vec3<T>>());
  const auto qz = Quat<T>::AngleAxis(inEulerAnglesRads[2], Forward<Vec3<T>>());
  return Normalized(qz * qy * qx);
}

template <typename T>
constexpr Quat<T> FromTo(const Vec3<T>& inFromNormalized, const Vec3<T>& inToNormalized)
{
  EXPECTS(IsNormalized(inFromNormalized));
  EXPECTS(IsNormalized(inToNormalized));

  const auto from_to_dot = Dot(inFromNormalized, inToNormalized);
  if (IsVeryEqual(from_to_dot, static_cast<T>(1)))
    return Identity<Quat<T>>();

  if (IsVeryEqual(from_to_dot, static_cast<T>(-1)))
    return AngleAxis(HalfCircleRads<T>(), Up<Vec3f>());

  const auto s = (static_cast<T>(1) + from_to_dot);
  const auto c = Cross(inFromNormalized, inToNormalized);
  return Normalized(Quat<T> { c[0], c[1], c[2], s });
}

template <typename T>
constexpr Quat<T> LookInDirection(const Vec3<T>& inForwardNormalized, const Vec3<T>& inUpNormalized)
{
  EXPECTS(IsNormalized(inForwardNormalized));
  EXPECTS(IsNormalized(inUpNormalized));

  const auto [x_axis, y_axis, inverted_z_axis] = Axes(inForwardNormalized, inUpNormalized);
  const auto z_axis = -inverted_z_axis;

  auto rotation_basis_matrix = Mat4<T> { Vec4<T> { x_axis[0], y_axis[0], z_axis[0], static_cast<T>(0) },
    Vec4<T> { x_axis[1], y_axis[1], z_axis[1], static_cast<T>(0) },
    Vec4<T> { x_axis[2], y_axis[2], z_axis[2], static_cast<T>(0) },
    Vec4<T> { static_cast<T>(0), static_cast<T>(0), static_cast<T>(0), static_cast<T>(1) } };

  const auto quat_result = Normalized(ToQuaternion(rotation_basis_matrix));
  ENSURES(IsNormalized(quat_result));
  return quat_result;
}

template <typename T>
constexpr std::tuple<Vec3<T>, Vec3<T>, Vec3<T>> Axes(const Quat<T>& inOrientation)
{
  EXPECTS(IsNormalized(inOrientation));
  return { inOrientation * Right<Vec3<T>>(), inOrientation * Up<Vec3<T>>(), inOrientation * Forward<Vec3<T>>() };
}

template <typename T>
constexpr Vec2<T> Rotated(const Vec2<T>& inVec, const AngleRads inAngle)
{
  return Vec2<T> { inVec[0] * std::cos(inAngle) - inVec[1] * std::sin(inAngle),
    inVec[0] * std::sin(inAngle) + inVec[1] * std::cos(inAngle) };
}

template <typename T>
constexpr Vec3<T> Rotated(const Vec3<T>& inVec, const Quat<T>& inRotation)
{
  return inRotation * inVec;
}

template <typename T>
constexpr AngleRads Rotated(const AngleRads inLHS, const AngleRads inRHS)
{
  return inLHS + inRHS;
}

template <typename T>
constexpr Quat<T> Rotated(const Quat<T>& inLHS, const Quat<T>& inRHS)
{
  return inLHS * inRHS;
}

template <typename T, typename TQuat>
constexpr TQuat SLerp(const TQuat& inFrom, const TQuat& inTo, const T& inT)
{
  auto to = inTo;
  auto cosTheta = Dot(inFrom, inTo);

  // If cosTheta < 0, the interpolation will take the long way around the
  // sphere. To fix this, one quat must be negated.
  if (cosTheta < static_cast<T>(0))
  {
    to = -inTo;
    cosTheta = -cosTheta;
  }

  if (cosTheta > static_cast<T>(1) - static_cast<T>(0.01))
  {
    return { Lerp(inFrom[0], to[0], inT),
      Lerp(inFrom[1], to[1], inT),
      Lerp(inFrom[2], to[2], inT),
      Lerp(inFrom[3], to[3], inT) };
  }
  else
  {
    const auto angle = std::acos(cosTheta);
    return (std::sin((static_cast<T>(1) - inT) * angle) * inFrom + std::sin(inT * angle) * to) / std::sin(angle);
  }
}

template <typename T>
constexpr auto Inverted(const Quat<T>& inValue)
{
  return -inValue;
}
}
