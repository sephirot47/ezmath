#pragma once

#include "ez/MathInitializerTokens.h"
#include "ez/MathTypeTraits.h"
#include "ez/MathForward.h"

namespace ez
{
using AngleRads = float;

template <typename T = float>
constexpr T Pi()
{
  return static_cast<T>(3.14159265358979323846);
}

template <typename T = float>
constexpr T QuarterPi()
{
  return Pi<T>() / 4;
}

template <typename T = float>
constexpr T HalfPi()
{
  return Pi<T>() / 2;
}

template <typename T = float>
constexpr T TwoPi()
{
  return Pi<T>() * 2;
}

template <typename T = float>
constexpr T FullCircleRads()
{
  return TwoPi<T>();
}

template <typename T = float>
constexpr T HalfCircleRads()
{
  return FullCircleRads<T>() / 2;
}

template <typename T = float>
constexpr T QuarterCircleRads()
{
  return FullCircleRads<T>() / 4;
}

template <typename T = float>
constexpr T Infinity()
{
  return std::numeric_limits<T>::infinity();
}

template <typename T>
constexpr auto All(const ValueType_t<T>& inAllValue)
{
  if constexpr (IsNumber_v<T>)
  {
    return inAllValue;
  }
  else
  {
    return T(MITAll<ValueType_t<T>> { static_cast<ValueType_t<T>>(inAllValue) });
  }
}

template <typename T>
constexpr T Zero()
{
  return All<T>(0);
}

template <typename T>
constexpr T One()
{
  return All<T>(1);
}

template <typename T>
constexpr auto Identity()
{
  if constexpr (IsMat_v<T>)
  {
    return T(MITMultiplicativeIdentity());
  }
  else if constexpr (IsQuat_v<T>)
  {
    return T(MITMultiplicativeIdentity());
  }
  else if constexpr (IsVec_v<T>)
  {
    return Zero<T>();
  }
  else if constexpr (IsNumber_v<T>)
  {
    return static_cast<T>(0);
  }
  else
  {
    static_assert(!std::is_same_v<T, T>, "Not implemented for this type.");
  }
}

}