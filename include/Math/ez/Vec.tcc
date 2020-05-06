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
  return !(*this >= inRHS);
}

template <typename T, std::size_t N>
constexpr bool Vec<T, N>::operator<=(const Vec& inRHS) const
{
  return !(*this > inRHS);
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
}
