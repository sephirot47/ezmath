#pragma once

#include <type_traits>

namespace ez
{
// IsVec. Template specialization for Vec is in "Vec.h"
template <typename T>
struct IsVec final
{
  static constexpr bool value = false;
};
template <typename T>
constexpr bool IsVec_v = IsVec<T>::value;

// IsMat. Template specialization for Mat is in "Mat.h"
template <typename T>
struct IsMat final
{
  static constexpr bool value = false;
};
template <typename T>
inline constexpr bool IsMat_v = IsMat<T>::value;

// IsQuat. Template specialization for Quat is in "Quat.h"
template <typename T>
struct IsQuat
{
  static constexpr bool value = false;
};
template <typename T>
inline constexpr bool IsQuat_v = IsQuat<T>::value;

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
}