#pragma once

#include "ez/CommonTypeTraits.h"
#include <type_traits>

namespace ez
{
// IsVec. Template specialization for Vec is in "Vec.h"
template <typename T>
struct IsVec final : std::false_type
{
};
template <typename T>
constexpr bool IsVec_v = IsVec<T>::value;

// IsMat. Template specialization for Mat is in "Mat.h"
template <typename T>
struct IsMat final : std::false_type
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

// IsAAHyperBox. Template specialization for AAHyperBox is in "AAHyperBox.h"
template <typename T>
struct IsAAHyperBox final : std::false_type
{
};
template <typename T>
constexpr bool IsAAHyperBox_v = IsAAHyperBox<T>::value;

// IsNumber
template <typename T>
constexpr auto IsNumber_v = std::is_arithmetic_v<std::remove_cvref_t<T>>;

// IsVecOrMat
template <typename T>
constexpr auto IsVecOrMat_v = IsVec_v<T> || IsMat_v<T>;

template <typename T>
constexpr auto _GetNumComponents()
{
  if constexpr (IsNumber_v<T>)
  {
    return 1;
  }
  else if constexpr (IsVec_v<T>)
  {
    return T::NumComponents;
  }
  else if constexpr (IsQuat_v<T>)
  {
    return T::NumComponents;
  }
  else if constexpr (IsMat_v<T>)
  {
    return T::NumComponents;
  }
  else
  {
    return 0;
  }
}

template <typename... TArgs>
constexpr auto NumComponents_v = (_GetNumComponents<std::remove_cvref_t<TArgs>>() + ... + 0);

template <typename T>
auto _GetValueType()
{
  if constexpr (IsNumber_v<T>)
  {
    return T {};
  }
  else
  {
    return typename T::ValueType {};
  }
}

template <typename T>
using ValueType_t = decltype(_GetValueType<std::remove_cvref_t<T>>());

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