#include "ez/MathCommon.h"
#include "ez/MathInitializers.h"
#include "ez/Vec.h"
#include <cmath>

namespace ez
{

template <typename T, std::size_t N>
constexpr Vec<T, N>::Vec(const MITAll<T>& inMITAll) noexcept
    : mComponents { GetArrayWithRepeatedValue<ValueType, N>(static_cast<ValueType>(inMITAll.mAllValue)) }
{
}

namespace internal
{
  template <typename T, std::size_t N, std::size_t Index>
  constexpr auto MakeVecRec2(Vec<T, N>&) // Base case
  {
    static_assert(Index == N);
  }

  template <typename T, std::size_t N, std::size_t Index, typename TFirstArg, typename... TArgs>
  constexpr auto MakeVecRec2(Vec<T, N>& ioVec, TFirstArg&& ioFirstArg, TArgs&&... ioArgs)
  {
    if constexpr (IsNumber_v<TFirstArg>)
    {
      ioVec[Index] = static_cast<T>(ioFirstArg);
      internal::MakeVecRec2<T, N, Index + 1>(ioVec, std::forward<TArgs>(ioArgs)...);
    }
    else if constexpr (IsVec_v<TFirstArg>)
    {
      constexpr auto PartNumComponents = NumComponents_v<TFirstArg>;
      for (std::size_t i = 0; i < PartNumComponents; ++i) { ioVec[i + Index] = static_cast<T>(ioFirstArg[i]); }
      internal::MakeVecRec2<T, N, Index + PartNumComponents>(ioVec, std::forward<TArgs>(ioArgs)...);
    }
  }

  template <typename T, typename TFirstArg, typename... TArgs>
  constexpr auto MakeVecRec(TFirstArg&& ioFirstArg, TArgs&&... ioArgs)
  {
    constexpr auto N = NumComponents_v<TFirstArg> + (NumComponents_v<TArgs> + ...);

    Vec<T, N> vec;
    internal::MakeVecRec2<T, N, 0>(vec, std::forward<TFirstArg>(ioFirstArg), std::forward<TArgs>(ioArgs)...);
    return vec;
  }
}

template <typename T, std::size_t N>
template <typename... TArgs, typename>
constexpr Vec<T, N>::Vec(TArgs&&... ioArgs) noexcept
{
  *this = internal::MakeVecRec<T>(std::forward<TArgs>(ioArgs)...);
}

template <typename T, std::size_t N>
template <typename TOther>
constexpr Vec<T, N>::Vec(const Vec<TOther, N>& inOther) noexcept
{
  Vec<T, N> vec;
  for (std::size_t i = 0; i < N; ++i) { vec[i] = static_cast<T>(inOther[i]); }
  *this = vec;
}

template <typename T, std::size_t N>
T* Vec<T, N>::Data()
{
  return &mComponents[0];
}

template <typename T, std::size_t N>
const T* Vec<T, N>::Data() const
{
  return &mComponents[0];
}

template <typename T, std::size_t N>
constexpr typename std::array<T, N>::iterator Vec<T, N>::begin()
{
  return std::begin(mComponents);
}

template <typename T, std::size_t N>
constexpr typename std::array<T, N>::iterator Vec<T, N>::end()
{
  return std::end(mComponents);
}
template <typename T, std::size_t N>
constexpr typename std::array<T, N>::const_iterator Vec<T, N>::begin() const
{
  return std::begin(mComponents);
}

template <typename T, std::size_t N>
constexpr typename std::array<T, N>::const_iterator Vec<T, N>::end() const
{
  return std::end(mComponents);
}

template <typename T, std::size_t N>
constexpr typename std::array<T, N>::const_iterator Vec<T, N>::cbegin() const
{
  return std::cbegin(mComponents);
}

template <typename T, std::size_t N>
constexpr typename std::array<T, N>::const_iterator Vec<T, N>::cend() const
{
  return std::cend(mComponents);
}

template <typename T, std::size_t N>
constexpr bool Vec<T, N>::operator==(const Vec& inRHS) const
{
  return mComponents == inRHS.mComponents;
}

template <typename T, std::size_t N>
constexpr bool Vec<T, N>::operator!=(const Vec& inRHS) const
{
  return !(*this == inRHS);
}

template <typename T, std::size_t N>
constexpr bool Vec<T, N>::operator>(const Vec& inRHS) const
{
  for (std::size_t i = 0; i < N; ++i)
  {
    if (!(mComponents[i] > inRHS[i]))
      return false;
  }
  return true;
}

template <typename T, std::size_t N>
constexpr bool Vec<T, N>::operator>=(const Vec& inRHS) const
{
  for (std::size_t i = 0; i < N; ++i)
  {
    if (!(mComponents[i] >= inRHS[i]))
      return false;
  }
  return true;
}

template <typename T, std::size_t N>
constexpr bool Vec<T, N>::operator<(const Vec& inRHS) const
{
  for (std::size_t i = 0; i < N; ++i)
  {
    if (!(mComponents[i] < inRHS[i]))
      return false;
  }
  return true;
}

template <typename T, std::size_t N>
constexpr bool Vec<T, N>::operator<=(const Vec& inRHS) const
{
  for (std::size_t i = 0; i < N; ++i)
  {
    if (!(mComponents[i] <= inRHS[i]))
      return false;
  }
  return true;
}

template <typename T, std::size_t N>
constexpr T& Vec<T, N>::operator[](std::size_t i)
{
  return mComponents[i];
}

template <typename T, std::size_t N>
constexpr const T& Vec<T, N>::operator[](std::size_t i) const
{
  return mComponents[i];
}

template <typename T, std::size_t N>
constexpr Vec<T, N> Vec<T, N>::operator+(const Vec<T, N>& inRHS) const
{
  Vec<T, N> result {};
  for (std::size_t i = 0; i < N; ++i) { result[i] = mComponents[i] + inRHS[i]; }
  return result;
}

template <typename T, std::size_t N>
constexpr Vec<T, N> Vec<T, N>::operator-(const Vec& inRHS) const
{
  Vec<T, N> result {};
  for (std::size_t i = 0; i < N; ++i) { result[i] = mComponents[i] - inRHS[i]; }
  return result;
}

template <typename T, std::size_t N>
constexpr Vec<T, N> Vec<T, N>::operator*(const Vec& inRHS) const
{
  Vec<T, N> result {};
  for (std::size_t i = 0; i < N; ++i) { result[i] = mComponents[i] * inRHS[i]; }
  return result;
}

template <typename T, std::size_t N>
constexpr Vec<T, N> Vec<T, N>::operator/(const Vec& inRHS) const
{
  Vec<T, N> result {};
  for (std::size_t i = 0; i < N; ++i) { result[i] = mComponents[i] / inRHS[i]; }
  return result;
}

template <typename T, std::size_t N>
constexpr Vec<T, N> Vec<T, N>::operator+(const T& inRHS) const
{
  Vec<T, N> result {};
  for (std::size_t i = 0; i < N; ++i) { result[i] = mComponents[i] + inRHS; }
  return result;
}

template <typename T, std::size_t N>
constexpr Vec<T, N> Vec<T, N>::operator-(const T& inRHS) const
{
  Vec<T, N> result {};
  for (std::size_t i = 0; i < N; ++i) { result[i] = mComponents[i] - inRHS; }
  return result;
}

template <typename T, std::size_t N>
constexpr Vec<T, N> Vec<T, N>::operator*(const T& inRHS) const
{
  Vec<T, N> result {};
  for (std::size_t i = 0; i < N; ++i) { result[i] = mComponents[i] * inRHS; }
  return result;
}

template <typename T, std::size_t N>
constexpr Vec<T, N> Vec<T, N>::operator/(const T& inRHS) const
{
  Vec<T, N> result {};
  for (std::size_t i = 0; i < N; ++i) { result[i] = mComponents[i] / inRHS; }
  return result;
}

template <typename T, std::size_t N>
constexpr Vec<T, N> Vec<T, N>::operator-() const
{
  Vec<T, N> result {};
  for (std::size_t i = 0; i < N; ++i) { result[i] = -mComponents[i]; }
  return result;
}

template <typename T, std::size_t N>
constexpr void Vec<T, N>::operator+=(const Vec<T, N>& inRHS)
{
  *this = *this + inRHS;
}

template <typename T, std::size_t N>
constexpr void Vec<T, N>::operator-=(const Vec<T, N>& inRHS)
{
  *this = *this - inRHS;
}

template <typename T, std::size_t N>
constexpr void Vec<T, N>::operator*=(const Vec<T, N>& inRHS)
{
  *this = *this * inRHS;
}

template <typename T, std::size_t N>
constexpr void Vec<T, N>::operator/=(const Vec<T, N>& inRHS)
{
  *this = *this / inRHS;
}

template <typename T, std::size_t N>
constexpr void Vec<T, N>::operator+=(const T& inRHS)
{
  *this = *this + inRHS;
}

template <typename T, std::size_t N>
constexpr void Vec<T, N>::operator-=(const T& inRHS)
{
  *this = *this - inRHS;
}

template <typename T, std::size_t N>
constexpr void Vec<T, N>::operator*=(const T& inRHS)
{
  *this = *this * inRHS;
}

template <typename T, std::size_t N>
constexpr void Vec<T, N>::operator/=(const T& inRHS)
{
  *this = *this / inRHS;
}

template <typename T, std::size_t N>
inline constexpr Vec<T, N> operator+(const T& inLHS, const Vec<T, N>& inRHS)
{
  Vec<T, N> lhs_vec { MITAll<T> { inLHS } };
  return lhs_vec + inRHS;
}

template <typename T, std::size_t N>
inline constexpr Vec<T, N> operator-(const T& inLHS, const Vec<T, N>& inRHS)
{
  Vec<T, N> lhs_vec { MITAll<T> { inLHS } };
  return lhs_vec - inRHS;
}

template <typename T, std::size_t N>
inline constexpr Vec<T, N> operator*(const T& inLHS, const Vec<T, N>& inRHS)
{
  Vec<T, N> lhs_vec { MITAll<T> { inLHS } };
  return lhs_vec * inRHS;
}

template <typename T, std::size_t N>
inline constexpr Vec<T, N> operator/(const T& inLHS, const Vec<T, N>& inRHS)
{
  Vec<T, N> lhs_vec { MITAll<T> { inLHS } };
  return lhs_vec / inRHS;
}

template <typename T, std::size_t N>
inline std::ostream& operator<<(std::ostream& inLHS, const Vec<T, N>& inRHS)
{
  inLHS << "(";
  for (std::size_t i = 0; i < N; ++i) inLHS << inRHS[i] << (i < N - 1 ? ", " : "");
  inLHS << ")";
  return inLHS;
}

template <typename T, std::size_t N>
constexpr Vec<T, N> FromTo(const Vec<T, N>& inFrom, const Vec<T, N>& inTo)
{
  return (inTo - inFrom);
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

template <typename T, std::size_t N>
constexpr Vec<T, N> Direction(const Vec<T, N>& inVector)
{
  return NormalizedSafe(inVector);
}

template <typename T, std::size_t N>
constexpr auto Inverted(const Vec<T, N>& inValue)
{
  return -inValue;
}

template <typename T>
constexpr std::tuple<Vec3<T>, Vec3<T>, Vec3<T>> Axes(const Vec3<T>& inForwardVectorNormalized,
    const Vec3<T>& inUpVectorNormalized)
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
constexpr Vec2<T> Perpendicular(const Vec2<T>& inVector)
{
  return Vec2<T> { -inVector[1], inVector[0] };
}

template <typename T>
constexpr Vec3<T> Cross(const Vec3<T>& inLHS, const Vec3<T>& inRHS)
{
  return Vec3<T> { inLHS[1] * inRHS[2] - inLHS[2] * inRHS[1],
    inLHS[2] * inRHS[0] - inLHS[0] * inRHS[2],
    inLHS[0] * inRHS[1] - inLHS[1] * inRHS[0] };
}

template <typename T>
constexpr auto Right()
{
  return WithPart<0, 0>(Zero<T>(), static_cast<ValueType_t<T>>(1));
}

template <typename T>
constexpr auto Up()
{
  return WithPart<1, 1>(Zero<T>(), static_cast<ValueType_t<T>>(1));
}

template <typename T>
constexpr auto Forward()
{
  return WithPart<2, 2>(Zero<T>(), static_cast<ValueType_t<T>>(-1));
}

template <typename T>
constexpr T Left()
{
  return -Right<T>();
}

template <typename T>
constexpr T Down()
{
  return -Up<T>();
}

template <typename T>
constexpr T Back()
{
  return -Forward<T>();
}

template <typename T, std::size_t N>
constexpr Vec<T, N> Reflect(const Vec<T, N>& inIncomingVectorNormalized, const Vec<T, N>& inNormalNormalized)
{
  // inIncomingVectorNormalized pointing into the surface
  // inNormalNormalized pointing out the surface
  EXPECTS(IsNormalized(inIncomingVectorNormalized));
  EXPECTS(IsNormalized(inNormalNormalized));
  const auto reflected = inIncomingVectorNormalized
      + static_cast<T>(2) * Dot(inIncomingVectorNormalized, inNormalNormalized) * inNormalNormalized;
  return reflected;
}

template <typename T, std::size_t N, typename TPrimitive>
constexpr Vec<T, N> ClosestPoint(const Vec<T, N>& inPoint, const TPrimitive& inPrimitive)
{
  return inPoint;
}

template <EIntersectMode TIntersectMode, typename T, typename TPrimitive, std::size_t N>
auto Intersect(const Vec<T, N>& inPoint, const TPrimitive& inPrimitive)
{
  const auto contained = Contains(inPrimitive, inPoint);
  if constexpr (TIntersectMode == EIntersectMode::ALL_INTERSECTIONS)
    return std::array<std::optional<T>, 1> { contained ? std::make_optional(static_cast<T>(0)) : std::optional<T> {} };
  else if constexpr (TIntersectMode == EIntersectMode::ONLY_CLOSEST)
    return contained ? std::make_optional(static_cast<T>(0)) : std::optional<T> {};
  else if constexpr (TIntersectMode == EIntersectMode::ONLY_CHECK)
    return contained;
}

template <typename T, std::size_t N>
void Transform(Vec<T, N>& ioPoint, const Transformation<T, N>& inTransformation)
{
  ioPoint = inTransformation.TransformedPoint(ioPoint);
}

template <typename T, std::size_t N>
void Transform(Vec<T, N>& ioPoint, const SquareMat<T, N>& inTransformMatrix)
{
  ioPoint = (inTransformMatrix * ioPoint);
}

template <typename T, std::size_t N>
void Transform(Vec<T, N>& ioPoint, const SquareMat<T, N + 1>& inTransformMatrix)
{
  // If the point has one less dimension than the transform matrix, convert the point to one
  // more dimension by adding a 1 at the end, and then retrieve it from the result

  Vec<T, N + 1> point_and_1;
  for (std::size_t i = 0; i < N; ++i) { point_and_1[i] = ioPoint[i]; }
  point_and_1[N] = static_cast<T>(1);

  const auto transformed_point_and_1 = Transformed(point_and_1, inTransformMatrix);
  for (std::size_t i = 0; i < N; ++i) { ioPoint[i] = transformed_point_and_1[i]; }
}

template <typename T, std::size_t N>
void InverseTransform(Vec<T, N>& ioPoint, const Transformation<ValueType_t<T>, N>& inTransformation)
{
  ioPoint = inTransformation.InverseTransformedPoint(ioPoint);
}

template <typename T, std::size_t N>
constexpr auto BoundingAAHyperBox(const Vec<T, N>& inPoint)
{
  return AAHyperBox<T, N>(inPoint, inPoint);
}

template <typename T, std::size_t N>
constexpr auto BoundingAAHyperBoxTransformed(const Vec<T, N>& inPoint, const Transformation<T, N>& inTransformation)
{
  const auto transformed_point = inTransformation.TransformedPoint(inPoint);
  return BoundingAAHyperBox(transformed_point, transformed_point);
}

template <typename T, std::size_t N>
constexpr auto BoundingAAHyperBoxInverseTransformed(const Vec<T, N>& inPoint,
    const Transformation<T, N>& inTransformation)
{
  const auto inverse_transformed_point = inTransformation.InverseTransformedPoint(inPoint);
  return BoundingAAHyperBox(inverse_transformed_point, inverse_transformed_point);
}

}
