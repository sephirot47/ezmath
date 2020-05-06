#pragma once

#include "ez/MathInitializerTokens.h"
#include "ez/MathTypeTraits.h"

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
constexpr T Diagonal(const ValueType_t<T>& inDiagonalValue)
{
  if constexpr (IsMat_v<T>)
  {
    static_assert(T::NumRows == T::NumCols, "Diagonal only supported for square matrices.");

    T diagonal_matrix = All<T>(static_cast<ValueType_t<T>>(0));
    for (std::size_t i = 0; i < T::NumRows; ++i) { diagonal_matrix[i][i] = inDiagonalValue; }
    return diagonal_matrix;
  }
  else
  {
    static_assert(!std::is_same_v<T, T>, "Not implemented for this type.");
  }
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

template <typename TColor>
constexpr TColor Black()
{
  using TValue = ValueType_t<TColor>;
  if constexpr (TColor::NumComponents == 4)
  {
    return TColor(static_cast<TValue>(0), static_cast<TValue>(0), static_cast<TValue>(0), static_cast<TValue>(1));
  }
  else
  {
    return TColor(static_cast<TValue>(0), static_cast<TValue>(0), static_cast<TValue>(0));
  }
}

template <typename TColor>
constexpr TColor Gray()
{
  using TValue = ValueType_t<TColor>;
  if constexpr (TColor::NumComponents == 4)
  {
    return TColor(static_cast<TValue>(0.5), static_cast<TValue>(0.5), static_cast<TValue>(0.5), static_cast<TValue>(1));
  }
  else
  {
    return TColor(static_cast<TValue>(0.5), static_cast<TValue>(0.5), static_cast<TValue>(0.5));
  }
}

template <typename TColor>
constexpr TColor White()
{
  using TValue = ValueType_t<TColor>;
  if constexpr (TColor::NumComponents == 4)
  {
    return TColor(static_cast<TValue>(1), static_cast<TValue>(1), static_cast<TValue>(1), static_cast<TValue>(1));
  }
  else
  {
    return TColor(static_cast<TValue>(1), static_cast<TValue>(1), static_cast<TValue>(1));
  }
}

template <typename TColor>
constexpr TColor Red()
{
  using TValue = ValueType_t<TColor>;
  if constexpr (TColor::NumComponents == 4)
  {
    return TColor(static_cast<TValue>(1), static_cast<TValue>(0), static_cast<TValue>(0), static_cast<TValue>(1));
  }
  else
  {
    return TColor(static_cast<TValue>(1), static_cast<TValue>(0), static_cast<TValue>(0));
  }
}

template <typename TColor>
constexpr TColor Green()
{
  using TValue = ValueType_t<TColor>;
  if constexpr (TColor::NumComponents == 4)
  {
    return TColor(static_cast<TValue>(0), static_cast<TValue>(1), static_cast<TValue>(0), static_cast<TValue>(1));
  }
  else
  {
    return TColor(static_cast<TValue>(0), static_cast<TValue>(1), static_cast<TValue>(0));
  }
}

template <typename TColor>
constexpr TColor Blue()
{
  using TValue = ValueType_t<TColor>;
  if constexpr (TColor::NumComponents == 4)
  {
    return TColor(static_cast<TValue>(0), static_cast<TValue>(0), static_cast<TValue>(1), static_cast<TValue>(1));
  }
  else
  {
    return TColor(static_cast<TValue>(0), static_cast<TValue>(0), static_cast<TValue>(1));
  }
}

template <typename TColor>
constexpr TColor Cyan()
{
  using TValue = ValueType_t<TColor>;
  if constexpr (TColor::NumComponents == 4)
  {
    return TColor(static_cast<TValue>(0), static_cast<TValue>(1), static_cast<TValue>(1), static_cast<TValue>(1));
  }
  else

    return TColor(static_cast<TValue>(0), static_cast<TValue>(1), static_cast<TValue>(1));
}

template <typename TColor>
constexpr TColor Magenta()
{
  using TValue = ValueType_t<TColor>;
  if constexpr (TColor::NumComponents == 4)
  {
    return TColor(static_cast<TValue>(1), static_cast<TValue>(0), static_cast<TValue>(1), static_cast<TValue>(1));
  }
  else
  {
    return TColor(static_cast<TValue>(1), static_cast<TValue>(0), static_cast<TValue>(1));
  }
}

template <typename TColor>
constexpr TColor Yellow()
{
  using TValue = ValueType_t<TColor>;
  if constexpr (TColor::NumComponents == 4)
  {
    return TColor(static_cast<TValue>(1), static_cast<TValue>(1), static_cast<TValue>(0), static_cast<TValue>(1));
  }
  else
  {
    return TColor(static_cast<TValue>(1), static_cast<TValue>(1), static_cast<TValue>(0));
  }
}

template <typename TColor>
constexpr TColor Brown()
{
  using TValue = ValueType_t<TColor>;
  if constexpr (TColor::NumComponents == 4)
  {
    return TColor(static_cast<TValue>(0.5), static_cast<TValue>(0.25), static_cast<TValue>(0), static_cast<TValue>(1));
  }
  else
  {
    return TColor(static_cast<TValue>(0.5), static_cast<TValue>(0.25), static_cast<TValue>(0));
  }
}

template <typename TColor>
constexpr TColor Orange()
{
  using TValue = ValueType_t<TColor>;
  if constexpr (TColor::NumComponents == 4)
  {
    return TColor(static_cast<TValue>(1), static_cast<TValue>(0.5), static_cast<TValue>(0), static_cast<TValue>(1));
  }
  else
  {
    return TColor(static_cast<TValue>(1), static_cast<TValue>(0.5), static_cast<TValue>(0));
  }
}

template <typename TColor>
constexpr TColor Purple()
{
  using TValue = ValueType_t<TColor>;
  if constexpr (TColor::NumComponents == 4)
  {
    return TColor(static_cast<TValue>(0.5), static_cast<TValue>(0), static_cast<TValue>(1), static_cast<TValue>(1));
  }
  else
  {
    return TColor(static_cast<TValue>(0.5), static_cast<TValue>(0), static_cast<TValue>(1));
  }
}

template <typename TColor>
constexpr TColor Pink()
{
  using TValue = ValueType_t<TColor>;
  if constexpr (TColor::NumComponents == 4)
  {
    return TColor(static_cast<TValue>(1), static_cast<TValue>(0.5), static_cast<TValue>(0.5), static_cast<TValue>(1));
  }
  else
  {
    return TColor(static_cast<TValue>(1), static_cast<TValue>(0.5), static_cast<TValue>(0.5));
  }
}

template <typename TColor>
constexpr TColor WithAlpha(const TColor& inColor, const typename TColor::ValueType inAlpha)
{
  TColor new_color = inColor;
  new_color[3] = inAlpha;
  return new_color;
}

template <typename TColor>
constexpr TColor WithValue(const TColor& inColor, const typename TColor::ValueType inValue)
{
  TColor new_color = inColor;
  for (std::size_t i = 0; i < 3; ++i) { new_color[i] *= inValue; }
  return new_color;
}

template <typename T>
constexpr auto Right()
{
  if constexpr (IsVec_v<T>)
  {
    static_assert(T::NumComponents >= 1);
    T result = All<T>(static_cast<typename T::ValueType>(0));
    result[0] = static_cast<typename T::ValueType>(1);
    return result;
  }
  else
  {
    static_assert(!std::is_same_v<T, T>, "Not implemented for this type.");
  }
}

template <typename T>
constexpr auto Up()
{
  if constexpr (IsVec_v<T>)
  {
    static_assert(T::NumComponents >= 2);
    T result = All<T>(static_cast<typename T::ValueType>(0));
    result[1] = static_cast<typename T::ValueType>(1);
    return result;
  }
  else
  {
    static_assert(!std::is_same_v<T, T>, "Not implemented for this type.");
  }
}

template <typename T>
constexpr auto Forward()
{
  if constexpr (IsVec_v<T>)
  {
    static_assert(T::NumComponents >= 3);
    T result = All<T>(static_cast<typename T::ValueType>(0));
    result[2] = static_cast<typename T::ValueType>(-1);
    return result;
  }
  else
  {
    static_assert(!std::is_same_v<T, T>, "Not implemented for this type.");
  }
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