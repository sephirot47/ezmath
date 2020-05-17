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

template <typename T, std::size_t N>
template <typename... TArgs, typename>
constexpr Vec<T, N>::Vec(TArgs&&... inArgs) noexcept : mComponents { std::forward<TArgs>(inArgs)... }
{
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
constexpr Vec3<T> Cross(const Vec3<T>& inLHS, const Vec3<T>& inRHS)
{
  return Vec3<T> { inLHS[1] * inRHS[2] - inLHS[2] * inRHS[1],
    inLHS[2] * inRHS[0] - inLHS[0] * inRHS[2],
    inLHS[0] * inRHS[1] - inLHS[1] * inRHS[0] };
}

template <typename T>
constexpr auto Right()
{
  static_assert(T::NumComponents >= 1);
  T result = All<T>(static_cast<typename T::ValueType>(0));
  result[0] = static_cast<typename T::ValueType>(1);
  return result;
}

template <typename T>
constexpr auto Up()
{
  static_assert(T::NumComponents >= 2);
  T result = All<T>(static_cast<typename T::ValueType>(0));
  result[1] = static_cast<typename T::ValueType>(1);
  return result;
}

template <typename T>
constexpr auto Forward()
{
  static_assert(T::NumComponents >= 3);
  T result = All<T>(static_cast<typename T::ValueType>(0));
  result[2] = static_cast<typename T::ValueType>(-1);
  return result;
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
}
