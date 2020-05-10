#pragma once

#include "ez/AACube.h"
#include "ez/AAHyperRectangle.h"
#include "ez/AARect.h"
#include "ez/Color.h"
#include "ez/Mat.h"
#include "ez/MathInitializers.h"
#include "ez/MathMultiComponent.h"
#include "ez/MathRandom.h"
#include "ez/MathSwizzling.h"
#include "ez/MathTypeTraits.h"
#include "ez/Quat.h"
#include "ez/Segment.h"
#include "ez/Vec.h"
#include <cmath>
#include <cstdint>
#include <tuple>

namespace ez
{
template <typename T>
constexpr auto Dot(const T& inLHS, const T& inRHS)
{
  using ValueType = typename T::ValueType;

  auto dot = static_cast<ValueType>(0);
  for (std::size_t i = 0; i < T::NumComponents; ++i) { dot += inLHS[i] * inRHS[i]; }
  return dot;
}

template <typename T>
constexpr Vec3<T> Cross(const Vec3<T>& inLHS, const Vec3<T>& inRHS)
{
  return Vec3<T> { inLHS[1] * inRHS[2] - inLHS[2] * inRHS[1],
    inLHS[2] * inRHS[0] - inLHS[0] * inRHS[2],
    inLHS[0] * inRHS[1] - inLHS[1] * inRHS[0] };
}

template <typename T>
constexpr auto SqLength(const T& inV)
{
  return Dot(inV, inV);
}

template <typename T>
constexpr auto Length(const T& inV)
{
  return std::sqrt(SqLength(inV));
}
template <typename T>
constexpr auto SqDistance(const T& inLHS, const T& inRHS)
{
  const auto diff = (inRHS - inLHS);
  return SqLength(diff, diff);
}

template <typename T>
constexpr auto Distance(const T& inLHS, const T& inRHS)
{
  if constexpr (IsNumber_v<T>)
  {
    return Abs(inLHS - inRHS);
  }
  else
  {
    return std::sqrt(SqDistance(inLHS, inRHS));
  }
}

template <typename T>
constexpr bool
IsVeryEqual(const T& inLHS, const T& inRHS, const T& inEpsilon = All<T>(static_cast<ValueType_t<T>>(1e-6)))
{
  return Abs(inLHS - inRHS) < inEpsilon;
}

template <typename T>
constexpr bool IsNormalized(const T& inV)
{
  using ValueType = typename T::ValueType;
  return IsVeryEqual(SqLength(inV), static_cast<ValueType>(1));
}

template <typename T>
[[nodiscard]] constexpr T Normalized(const T& inV)
{
  const auto length = Length(inV);
  EXPECTS(length != 0);
  return inV / length;
}

template <typename T>
[[nodiscard]] constexpr T NormalizedSafe(const T& inV)
{
  const auto sq_length = SqLength(inV);
  if (sq_length == 0.0f)
    return inV;
  const auto length = std::sqrt(sq_length);
  return inV / length;
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

template <typename T, std::size_t N>
constexpr Vec3<T> Direction(const Segment<T, N>& inSegment)
{
  const auto direction = NormalizedSafe(inSegment.GetVector());
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
constexpr std::tuple<Vec3<T>, Vec3<T>, Vec3<T>> Axes(const Vec3<T>& inForwardVectorNormalized,
    const Vec3<T>& inUpVectorNormalized = Up<Vec3<T>>())
{
  EXPECTS(IsNormalized(inForwardVectorNormalized));
  EXPECTS(IsNormalized(inUpVectorNormalized));

  const auto forward_vector = inForwardVectorNormalized;

  auto up_vector = inUpVectorNormalized;
  auto right_vector = Zero<Vec3f>();
  if (IsVeryParallel(forward_vector, up_vector))
  {
    right_vector = Right<Vec3f>();
    if (IsVeryParallel(right_vector, forward_vector))
      right_vector = Normalized(Vec3f(0.5f, 0.5f, 0.0f));

    up_vector = Normalized(Cross(right_vector, forward_vector));
    right_vector = Cross(forward_vector, up_vector);
  }
  else
  {
    right_vector = Normalized(Cross(forward_vector, up_vector));
    up_vector = Cross(right_vector, forward_vector);
  }

  ENSURES(IsNormalized(forward_vector));
  ENSURES(IsNormalized(up_vector));
  ENSURES(IsNormalized(right_vector));

  ENSURES(IsVeryPerpendicular(forward_vector, up_vector));
  ENSURES(IsVeryPerpendicular(forward_vector, right_vector));
  ENSURES(IsVeryPerpendicular(up_vector, right_vector));

  return { right_vector, up_vector, forward_vector };
}

template <typename T>
constexpr std::tuple<Vec3<T>, Vec3<T>, Vec3<T>> Axes(const Quat<T>& inOrientation)
{
  EXPECTS(IsNormalized(inOrientation));
  return { inOrientation * Right<Vec3<T>>(), inOrientation * Up<Vec3<T>>(), inOrientation * Forward<Vec3<T>>() };
}

template <typename T, std::size_t N>
constexpr SquareMat<T, N> Transposed(const SquareMat<T, N>& inMat)
{
  SquareMat<T, N> transposed;
  for (std::size_t row = 0; row < N; ++row)
  {
    for (std::size_t col = 0; col < N; ++col) { transposed[row][col] = inMat[col][row]; }
  }
  return transposed;
}

template <typename T>
constexpr auto BoundingAAHyperRectangle(const T& inThingToBound)
{
  if constexpr (IsVec_v<T>)
  {
    using BoundingAAHyperRectangleType = AAHyperRectangle<ValueType_t<T>, NumDimensions_v<T>>;
    BoundingAAHyperRectangleType bounding_aa_hyper_rectangle;
    bounding_aa_hyper_rectangle.Wrap(inThingToBound);
    return bounding_aa_hyper_rectangle;
  }
  else
  {
    using BoundingAAHyperRectangleType = decltype(BoundingAAHyperRectangle(*inThingToBound.begin()));
    BoundingAAHyperRectangleType bounding_aa_hyper_rectangle;
    if constexpr (IsAAHyperRectangle_v<T>) // Efficiency overloads
    {
      bounding_aa_hyper_rectangle.Wrap(inThingToBound.GetMin());
      bounding_aa_hyper_rectangle.Wrap(inThingToBound.GetMax());
    }
    else
    {
      for (const auto subthing_to_bound : inThingToBound)
      { bounding_aa_hyper_rectangle.Wrap(BoundingAAHyperRectangle(subthing_to_bound)); }
    }
    return bounding_aa_hyper_rectangle;
  }
}

template <typename T>
constexpr auto BoundingAARect(const T& inThingToBound)
{
  static_assert(NumDimensions_v<T> == 2);
  return BoundingAAHyperRectangle(inThingToBound);
}

template <typename T>
constexpr auto BoundingAACube(const T& inThingToBound)
{
  static_assert(NumDimensions_v<T> == 3);
  return BoundingAAHyperRectangle(inThingToBound);
}

template <typename T, std::size_t N>
constexpr SquareMat<T, N - 1>
Cofactor(const SquareMat<T, N>& inMat, const std::size_t inRowToRemove, const std::size_t inColumnToRemove)
{
  std::size_t cofactor_matrix_row = 0;
  std::size_t cofactor_matrix_col = 0;
  SquareMat<T, N - 1> cofactor_matrix;
  for (std::size_t row = 0; row < N; row++)
  {
    if (row == inRowToRemove)
      continue;

    for (std::size_t col = 0; col < N; col++)
    {
      if (col == inColumnToRemove)
        continue;

      cofactor_matrix[cofactor_matrix_row][cofactor_matrix_col] = inMat[row][col];

      ++cofactor_matrix_col;
      if (cofactor_matrix_col == N - 1)
      {
        ++cofactor_matrix_row;
        cofactor_matrix_col = 0;
      }
    }
  }
  return cofactor_matrix;
}

template <typename T, std::size_t N>
constexpr SquareMat<T, N> Adjoint(const SquareMat<T, N>& inMat)
{
  if (N == 1)
    return All<SquareMat<T, N>>(static_cast<T>(1));

  SquareMat<T, N> adjoint;
  for (std::size_t row = 0; row < N; row++)
  {
    for (std::size_t col = 0; col < N; col++)
    {
      const auto cofactor = Cofactor(inMat, row, col);
      const auto sign = static_cast<T>(((row + col) % 2 == 0) ? 1 : -1);
      adjoint[col][row] = sign * Determinant(cofactor);
    }
  }
  return adjoint;
}

template <typename T, std::size_t N>
constexpr T Determinant(const SquareMat<T, N>& inMat)
{
  if constexpr (N == 1)
    return inMat[0][0];
  else
  {
    auto sign = static_cast<T>(1);
    auto determinant = static_cast<T>(0);
    for (std::size_t f = 0; f < N; f++)
    {
      const auto cofactor = Cofactor(inMat, 0, f);
      determinant += sign * inMat[0][f] * Determinant(cofactor);
      sign = -sign;
    }
    return determinant;
  }
}

template <typename T>
constexpr auto Inverted(const T& inValue)
{
  if constexpr (IsVec_v<T> || IsQuat_v<T>)
  {
    return -inValue;
  }
  else if constexpr (IsMat_v<T>)
  {
    const auto determinant = Determinant(inValue);
    if (determinant == 0)
      THROW_EXCEPTION("Singular matrix (determinant is 0), can not compute its inverse");

    const auto adjoint = Adjoint(inValue);

    const auto inverse = adjoint / determinant;
    return inverse;
  }
  else
  {
    static_assert("Inverted not implemented for this type.");
  }
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
constexpr Quat<T> FromEulerAngles(const Vec3<T>& inEulerAnglesRads)
{
  const auto qx = Quat<T>::AngleAxis(inEulerAnglesRads[0], Right<Vec3<T>>());
  const auto qy = Quat<T>::AngleAxis(inEulerAnglesRads[1], Up<Vec3<T>>());
  const auto qz = Quat<T>::AngleAxis(inEulerAnglesRads[2], Forward<Vec3<T>>());
  return Normalized(qz * qy * qx);
}

template <typename T, std::size_t N>
constexpr Vec<T, N> FromTo(const Vec<T, N>& inFrom, const Vec<T, N>& inTo)
{
  return (inTo - inFrom);
}

template <typename T>
constexpr Quat<T> FromTo(const Vec3<T>& inFromNormalized, const Vec3<T>& inToNormalized)
{
  EXPECTS(IsNormalized(inFromNormalized));
  EXPECTS(IsNormalized(inToNormalized));

  const auto from_to_dot = Dot(inFromNormalized, inToNormalized);
  if (from_to_dot >= 1.0)
    return Identity<Quat<T>>();

  if (from_to_dot <= -1.0)
  {
    auto axis = Cross(Right<Vec3<T>>(), inFromNormalized);
    if (axis.SqLength() == 0)
      axis = Cross(Up<Vec3<T>>(), inFromNormalized);
    axis = Normalized(axis);
    return Quat<T>(axis[0], axis[1], axis[2], 0.0);
  }

  const auto s = static_cast<T>(std::sqrt((1 + from_to_dot) * 2));
  const auto inverse_of_s = (static_cast<T>(1.0) / s);
  const auto c = Cross(inFromNormalized, inToNormalized) * inverse_of_s;
  return Normalized(Vec3<T> { c[0], c[1], c[2], s * 0.5 });
}

template <typename T>
constexpr bool IsVeryParallel(const Vec3<T>& inDirection0, const Vec3<T>& inDirection1)
{
  return IsVeryEqual(Abs(Dot(inDirection0, inDirection1)), static_cast<T>(1));
}

template <typename T>
constexpr bool IsVeryPerpendicular(const Vec3<T>& inDirection0, const Vec3<T>& inDirection1)
{
  return IsVeryEqual(Abs(Dot(inDirection0, inDirection1)), static_cast<T>(0));
}

template <typename T>
constexpr Quat<T> LookInDirection(const Vec3<T>& inForwardNormalized, const Vec3<T>& inUpNormalized = Up<Vec3<T>>())
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

template <std::size_t N, typename T>
constexpr SquareMat<T, N> NormalMat(const SquareMat<T, N>& inModelViewMatrix)
{
  return Transposed(Inverted(inModelViewMatrix));
}

template <typename T, std::size_t N>
constexpr SquareMat<T, N + 1> TranslationMat(const Vec<T, N>& inTranslation)
{
  auto translation_matrix = Identity<SquareMat<T, N + 1>>();
  for (std::size_t i = 0; i < N; ++i) { translation_matrix[i][N] = inTranslation[i]; }
  return translation_matrix;
}

template <typename T>
constexpr std::enable_if_t<IsNumber_v<T>, SquareMat<T, 3>> RotationMat(const T inAngle)
{
  return SquareMat<T, 3> { Vec3<T> { std::cos(inAngle), -std::sin(inAngle), static_cast<T>(0) },
    Vec3<T> { std::sin(inAngle), std::cos(inAngle), static_cast<T>(0) },
    Vec3<T> { static_cast<T>(0), static_cast<T>(0), static_cast<T>(1) } };
}

template <typename T>
constexpr SquareMat<T, 4> RotationMat(const Quat<T>& inRotation)
{
  const auto& q = inRotation;
  const auto qxx { q[0] * q[0] };
  const auto qyy { q[1] * q[1] };
  const auto qzz { q[2] * q[2] };
  const auto qxz { q[0] * q[2] };
  const auto qxy { q[0] * q[1] };
  const auto qyz { q[1] * q[2] };
  const auto qwx { q[3] * q[0] };
  const auto qwy { q[3] * q[1] };
  const auto qwz { q[3] * q[2] };

  return Mat4<T> { Vec4<T> { static_cast<T>(1 - 2 * (qyy + qzz)),
                       static_cast<T>(2 * (qxy - qwz)),
                       static_cast<T>(2 * (qxz + qwy)),
                       static_cast<T>(0) },
    Vec4<T> { static_cast<T>(2 * (qxy + qwz)),
        static_cast<T>(1 - 2 * (qxx + qzz)),
        static_cast<T>(2 * (qyz - qwx)),
        static_cast<T>(0) },
    Vec4<T> { static_cast<T>(2 * (qxz - qwy)),
        static_cast<T>(2 * (qyz + qwx)),
        static_cast<T>(1 - 2 * (qxx + qyy)),
        static_cast<T>(0) },
    Vec4<T> { static_cast<T>(0), static_cast<T>(0), static_cast<T>(0), static_cast<T>(1) } };
}

template <typename T, std::size_t N>
constexpr SquareMat<T, N + 1> ScaleMat(const Vec<T, N>& inScale)
{
  auto scale_matrix = Identity<SquareMat<T, N + 1>>();
  for (std::size_t i = 0; i < N; ++i) { scale_matrix[i][i] = inScale[i]; }
  scale_matrix[N][N] = static_cast<T>(1);
  return scale_matrix;
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
constexpr Mat4<T> PerspectiveMat(const T inAngleOfViewRads, const T inAspectRatio, const T inZNear, const T inZFar)
{
  const T tanHalfFovy = std::tan(inAngleOfViewRads / static_cast<T>(2));

  Mat4<T> perspective_matrix = {};
  perspective_matrix[0][0] = static_cast<T>(1) / (inAspectRatio * tanHalfFovy);
  perspective_matrix[1][1] = static_cast<T>(1) / (tanHalfFovy);
  perspective_matrix[2][2] = -(inZFar + inZNear) / (inZFar - inZNear);
  perspective_matrix[3][2] = -static_cast<T>(1);
  perspective_matrix[2][3] = -(static_cast<T>(2) * inZFar * inZNear) / (inZFar - inZNear);
  return perspective_matrix;
}

template <typename T, std::size_t N>
constexpr SquareMat<T, N + 1> OrthographicMat(const Vec<T, N>& inMin, const Vec<T, N>& inMax)
{
  SquareMat<T, N + 1> orthographic_matrix;
  for (std::size_t i = 0; i < N; ++i)
  {
    orthographic_matrix[i][i] = static_cast<T>(2) / (inMax[i] - inMin[i]);
    orthographic_matrix[i][N] = -static_cast<T>(inMax[i] + inMin[i]) / static_cast<T>(inMax[i] - inMin[i]);
  }
  orthographic_matrix[N][N] = static_cast<T>(1);
  return orthographic_matrix;
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

template <typename T, std::size_t N>
Color<T, N> RGBToHSV(const Color<T, N>& inColorRGB)
{
  EXPECTS(inColorRGB[0] >= 0.0f && inColorRGB[0] <= 1.0f);
  EXPECTS(inColorRGB[1] >= 0.0f && inColorRGB[1] <= 1.0f);
  EXPECTS(inColorRGB[2] >= 0.0f && inColorRGB[2] <= 1.0f);

  auto result_hsv = inColorRGB;
  result_hsv[0] = result_hsv[1] = result_hsv[2] = static_cast<T>(0);

  const auto max_rgb_comp = Max(inColorRGB);
  const auto result_value = max_rgb_comp;
  if (result_value == static_cast<T>(0))
    return result_hsv;
  result_hsv[2] = result_value;

  const auto min_rgb_comp = Min(inColorRGB);
  const auto rgb_min_max_delta = (max_rgb_comp - min_rgb_comp);
  const auto result_saturation = (rgb_min_max_delta / max_rgb_comp);
  if (result_saturation == static_cast<T>(0))
    return result_hsv;
  result_hsv[1] = result_saturation;

  // Hue
  const auto& in_r = inColorRGB[0];
  const auto& in_g = inColorRGB[1];
  const auto& in_b = inColorRGB[2];

  auto result_hue = static_cast<T>(0);
  {
    if (in_r == max_rgb_comp)
      result_hue = (in_g - in_b) / rgb_min_max_delta;
    else if (in_g == max_rgb_comp)
      result_hue = static_cast<T>(2) + (in_b - in_r) / rgb_min_max_delta;
    else
      result_hue = static_cast<T>(4) + (in_r - in_g) / rgb_min_max_delta;
    result_hue /= 6.0f;
    result_hue = std::fmod(result_hue + 100.0f, 1.0f);
  }

  result_hsv[0] = result_hue;

  return result_hsv;
}

template <typename T, std::size_t N>
Color<T, N> HSVToRGB(const Color<T, N>& inColorHSV)
{
  EXPECTS(inColorHSV[0] >= 0.0f && inColorHSV[0] <= 1.0f);
  EXPECTS(inColorHSV[1] >= 0.0f && inColorHSV[1] <= 1.0f);
  EXPECTS(inColorHSV[2] >= 0.0f && inColorHSV[2] <= 1.0f);

  auto result_rgb = inColorHSV;

  const auto in_hue = inColorHSV[0];
  const auto hue_from_0_to_6 = in_hue * static_cast<T>(6);
  const auto in_saturation = inColorHSV[1];
  const auto in_value = inColorHSV[2];

  const auto hue_bucket = static_cast<int>(hue_from_0_to_6);
  const auto hue_remainder = (hue_from_0_to_6 - hue_bucket);
  const auto p = in_value * (static_cast<T>(1) - in_saturation);
  const auto q = in_value * (static_cast<T>(1) - (in_saturation * hue_remainder));
  const auto t = in_value * (static_cast<T>(1) - (in_saturation * (static_cast<T>(1) - hue_remainder)));

  switch (hue_bucket)
  {
  case 0:
    result_rgb[0] = in_value;
    result_rgb[1] = t;
    result_rgb[2] = p;
    break;

  case 1:
    result_rgb[0] = q;
    result_rgb[1] = in_value;
    result_rgb[2] = p;
    break;

  case 2:
    result_rgb[0] = p;
    result_rgb[1] = in_value;
    result_rgb[2] = t;
    break;

  case 3:
    result_rgb[0] = p;
    result_rgb[1] = q;
    result_rgb[2] = in_value;
    break;

  case 4:
    result_rgb[0] = t;
    result_rgb[1] = p;
    result_rgb[2] = in_value;
    break;

  case 5:
  default:
    result_rgb[0] = in_value;
    result_rgb[1] = p;
    result_rgb[2] = q;
    break;
  }
  return result_rgb;
}

}