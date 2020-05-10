#pragma once

#include "ez/CommonTypeTraits.h"
#include <type_traits>

namespace ez
{
// IsVec. Template specialization for Vec is in "Vec.h"
template <typename T>
struct IsVec final: std::false_type
{
};
template <typename T>
constexpr bool IsVec_v = IsVec<T>::value;

// IsMat. Template specialization for Mat is in "Mat.h"
template <typename T>
struct IsMat final: std::false_type
{
};
template <typename T>
inline constexpr bool IsMat_v = IsMat<T>::value;

// IsQuat. Template specialization for Quat is in "Quat.h"
template <typename T>
struct IsQuat final : std::false_type
{
};
template <typename T>
inline constexpr bool IsQuat_v = IsQuat<T>::value;

// IsAAHyperRectangle. Template specialization for AAHyperRectangle is in "AAHyperRectangle.h"
template <typename T>
struct IsAAHyperRectangle final: std::false_type
{
};
template <typename T>
constexpr bool IsAAHyperRectangle_v = IsAAHyperRectangle<T>::value;

// IsNumber
template <typename T>
constexpr auto IsNumber_v = std::is_arithmetic_v<T>;

// IsVecOrMat
template <typename T>
constexpr auto IsVecOrMat_v = IsVec_v<T> || IsMat_v<T>;

template <typename T>
std::enable_if_t<!IsNumber_v<T>, typename T::ValueType> _GetValueType()
{
  return typename T::ValueType {};
};

template <typename T>
std::enable_if_t<IsNumber_v<T>, T> _GetValueType()
{
  return T {};
};

template <typename T>
using ValueType_t = decltype(_GetValueType<T>());

template <typename T>
constexpr std::size_t GetNumDimensions()
{
  if constexpr (IsNumber_v<T>)
  {
    return 1;
  }
  else if constexpr (IsSpan_v<T>)
  {
    return GetNumDimensions<ValueType_t<T>>();
  }
  else
  {
    return T::NumDimensions;
  }
};

template <typename T>
static constexpr auto NumDimensions_v = GetNumDimensions<T>();

}