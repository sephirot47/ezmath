#pragma once

#include "ez/MathInitializers.h"
#include "ez/MathTypeTraits.h"
#include <array>
#include <cmath>
#include <limits>

namespace ez
{
template <typename T, auto TGenerateSingleComponentFunction, typename... TExtraArgs>
constexpr T MathMultiComponentGenerated(TExtraArgs&&... inExtraArgs)
{
  if constexpr (IsNumber_v<T>)
  {
    return TGenerateSingleComponentFunction(std::forward<TExtraArgs>(inExtraArgs)...);
  }
  else
  {
    T result;
    constexpr auto SplitExtraArgs = (std::is_same_v<std::decay_t<T>, std::decay_t<TExtraArgs>> && ...);
    for (auto it = result.cbegin(); it != result.cend(); ++it)
    {
      const auto i = (it - result.cbegin());
      auto& v = result[i];
      if constexpr (SplitExtraArgs)
      {
        v = MathMultiComponentGenerated<std::decay_t<decltype(v)>, TGenerateSingleComponentFunction>(
            std::forward<decltype(inExtraArgs[i])>(inExtraArgs[i])...);
      }
      else
      {
        v = MathMultiComponentGenerated<std::decay_t<decltype(v)>, TGenerateSingleComponentFunction>(
            std::forward<TExtraArgs>(inExtraArgs)...);
      }
    }
    return result;
  }
}

template <typename T, auto TBaseCaseFunction, typename... TExtraArgs>
constexpr T MathMultiComponentApplied(const T& inValue, TExtraArgs&&... inExtraArgs)
{
  if constexpr (IsNumber_v<T>)
  {
    return TBaseCaseFunction(inValue, std::forward<TExtraArgs>(inExtraArgs)...);
  }
  else
  {
    auto result = inValue;
    constexpr auto SplitExtraArgs = (std::is_same_v<std::decay_t<T>, std::decay_t<TExtraArgs>> && ...);
    for (auto it = result.cbegin(); it != result.cend(); ++it)
    {
      const auto i = (it - result.cbegin());
      auto& v = result[i];
      if constexpr (SplitExtraArgs)
      {
        v = MathMultiComponentApplied<std::decay_t<decltype(v)>, TBaseCaseFunction>(v,
            std::forward<decltype(inExtraArgs[i])>(inExtraArgs[i])...);
      }
      else
      {
        v = MathMultiComponentApplied<std::decay_t<decltype(v)>, TBaseCaseFunction>(v,
            std::forward<TExtraArgs>(inExtraArgs)...);
      }
    }
    return result;
  }
}

template <typename T>
constexpr auto Abs(const T& inValue)
{
  if constexpr (IsNumber_v<T>)
  {
    return std::abs(inValue);
  }
  else
  {
    return MathMultiComponentApplied<T, Abs<ValueType_t<T>>>(inValue);
  }
}

template <typename T>
constexpr auto Round(const T& inValue)
{
  if constexpr (IsNumber_v<T>)
  {
    return std::round(inValue);
  }
  else
  {
    return MathMultiComponentApplied<T, Round<ValueType_t<T>>>(inValue);
  }
}

template <typename T>
constexpr auto Ceil(const T& inValue)
{
  if constexpr (IsNumber_v<T>)
  {
    return std::ceil(inValue);
  }
  else
  {
    return MathMultiComponentApplied<T, Ceil<ValueType_t<T>>>(inValue);
  }
}

template <typename T>
constexpr auto Floor(const T& inValue)
{
  if constexpr (IsNumber_v<T>)
  {
    return std::floor(inValue);
  }
  else
  {
    return MathMultiComponentApplied<T, Floor<ValueType_t<T>>>(inValue);
  }
}

template <typename T>
constexpr auto Fract(const T& inValue)
{
  if constexpr (IsNumber_v<T>)
  {
    return static_cast<T>(inValue - static_cast<int64_t>(inValue));
  }
  else
  {
    return MathMultiComponentApplied<T, Fract<ValueType_t<T>>>(inValue);
  }
}

template <typename T>
constexpr auto Sqrt(const T& inValue)
{
  if constexpr (IsNumber_v<T>)
  {
    return std::sqrt(inValue);
  }
  else
  {
    return MathMultiComponentApplied<T, Sqrt<ValueType_t<T>>>(inValue);
  }
}

template <typename T>
constexpr auto Pow(const T& inValue, const T& inExponent)
{
  if constexpr (IsNumber_v<T>)
  {
    return static_cast<T>(std::pow(inValue, inExponent));
  }
  else
  {
    return MathMultiComponentApplied<T, Pow<ValueType_t<T>>>(inValue, inExponent);
  }
}

template <typename T>
constexpr std::enable_if_t<std::is_integral_v<ValueType_t<T>>, T> Pow2Int(const T& inExponent)
{
  return (static_cast<T>(1) << inExponent);
}

template <typename T>
constexpr std::enable_if_t<std::is_integral_v<ValueType_t<T>>, T> Pow8Int(const T& inExponent)
{
  return (static_cast<T>(1) << (inExponent + inExponent + inExponent));
}

template <typename T>
constexpr auto Log2(const T& inValue)
{
  if constexpr (IsNumber_v<T>)
  {
    return static_cast<T>(std::log2(inValue));
  }
  else
  {
    return MathMultiComponentApplied<T, Log2<ValueType_t<T>>>(inValue);
  }
}

template <typename T>
constexpr auto Log10(const T& inValue)
{
  if constexpr (IsNumber_v<T>)
  {
    return static_cast<T>(std::log10(inValue));
  }
  else
  {
    return MathMultiComponentApplied<T, Log10<ValueType_t<T>>>(inValue);
  }
}

template <typename T>
constexpr auto Sin(const T& inValue)
{
  if constexpr (IsNumber_v<T>)
  {
    return std::sin(inValue);
  }
  else
  {
    return MathMultiComponentApplied<T, Sin<ValueType_t<T>>>(inValue);
  }
}

template <typename T>
constexpr auto Cos(const T& inValue)
{
  if constexpr (IsNumber_v<T>)
  {
    return std::cos(inValue);
  }
  else
  {
    return MathMultiComponentApplied<T, Cos<ValueType_t<T>>>(inValue);
  }
}

template <typename T>
constexpr auto Tan(const T& inValue)
{
  if constexpr (IsNumber_v<T>)
  {
    return std::tan(inValue);
  }
  else
  {
    return MathMultiComponentApplied<T, Tan<ValueType_t<T>>>(inValue);
  }
}

template <typename T>
constexpr auto ASin(const T& inValue)
{
  if constexpr (IsNumber_v<T>)
  {
    return std::asin(inValue);
  }
  else
  {
    return MathMultiComponentApplied<T, ASin<ValueType_t<T>>>(inValue);
  }
}

template <typename T>
constexpr auto ACos(const T& inValue)
{
  if constexpr (IsNumber_v<T>)
  {
    return std::acos(inValue);
  }
  else
  {
    return MathMultiComponentApplied<T, ACos<ValueType_t<T>>>(inValue);
  }
}

template <typename T>
constexpr auto ATan(const T& inValue)
{
  if constexpr (IsNumber_v<T>)
  {
    return std::atan(inValue);
  }
  else
  {
    return MathMultiComponentApplied<T, ATan<ValueType_t<T>>>(inValue);
  }
}

template <typename T>
constexpr auto Clamp(const T& inValue, const T& inMin, const T& inMax)
{
  if constexpr (IsNumber_v<T>)
  {
    return (inValue < inMin ? inMin : (inValue > inMax ? inMax : inValue));
  }
  else
  {
    return MathMultiComponentApplied<T, Clamp<ValueType_t<T>>>(inValue, inMin, inMax);
  }
}

template <typename T>
constexpr auto Clamp01(const T& inValue)
{
  return Clamp<T>(inValue, static_cast<T>(0), static_cast<T>(1));
}

namespace math_multicomponent_detail
{
  template <typename T>
  constexpr T MinValue()
  {
    return All<T>(std::numeric_limits<ValueType_t<T>>::lowest());
  }

  template <typename T>
  constexpr T MaxValue()
  {
    return All<T>(std::numeric_limits<ValueType_t<T>>::max());
  }
}

template <typename T>
constexpr auto Min(const T& inLHS, const T& inRHS)
{
  if constexpr (IsNumber_v<T>)
  {
    return std::min(inLHS, inRHS);
  }
  else
  {
    return MathMultiComponentApplied<T, Min<ValueType_t<T>>>(inLHS, inRHS);
  }
}

template <typename T>
constexpr auto Min(const T& inValue)
{
  if constexpr (IsNumber_v<T>)
  {
    return inValue;
  }
  else
  {
    auto min = math_multicomponent_detail::MaxValue<ValueType_t<T>>();
    for (const auto& component : inValue) { min = std::min(min, Min(component)); }
    return min;
  }
}

template <typename T>
constexpr auto MinUnsigned(const T& inLHS, const T& inRHS)
{
  if constexpr (IsNumber_v<T>)
  {
    return (Abs(inLHS) < Abs(inRHS)) ? inLHS : inRHS;
  }
  else
  {
    return MathMultiComponentApplied<T, MinUnsigned<ValueType_t<T>>>(inLHS, inRHS);
  }
}

template <typename T>
constexpr auto MinUnsigned(const T& inValue)
{
  if constexpr (IsNumber_v<T>)
  {
    return inValue;
  }
  else
  {
    auto min = math_multicomponent_detail::MaxValue<ValueType_t<T>>();
    for (const auto& component : inValue) { min = Abs(MinUnsigned(component)) < Abs(min) ? component : min; }
    return min;
  }
}

template <typename T>
constexpr auto MinIndex(const T& inValue)
{
  if constexpr (IsNumber_v<T>)
  {
    return 0;
  }
  else
  {
    auto min_index = 0ul;
    for (std::size_t i = 0ul; i < NumComponents_v<T>; ++i)
    {
      if (inValue[i] < inValue[min_index])
        min_index = i;
    }
    return min_index;
  }
}

template <typename T>
constexpr auto MinUnsignedIndex(const T& inValue)
{
  if constexpr (IsNumber_v<T>)
  {
    return 0;
  }
  else
  {
    auto min_index = 0ul;
    for (std::size_t i = 0ul; i < NumComponents_v<T>; ++i)
    {
      if (Abs(inValue[i]) < Abs(inValue[min_index]))
        min_index = i;
    }
    return min_index;
  }
}

template <typename T>
constexpr auto Max(const T& inLHS, const T& inRHS)
{
  if constexpr (IsNumber_v<T>)
  {
    return std::max(inLHS, inRHS);
  }
  else
  {
    return MathMultiComponentApplied<T, Max<ValueType_t<T>>>(inLHS, inRHS);
  }
}

template <typename T>
constexpr auto Max(const T& inValue)
{
  if constexpr (IsNumber_v<T>)
  {
    return inValue;
  }
  else
  {
    auto max = math_multicomponent_detail::MinValue<ValueType_t<T>>();
    for (const auto& component : inValue) { max = std::max(max, Max(component)); }
    return max;
  }
}

template <typename T>
constexpr auto MaxUnsigned(const T& inLHS, const T& inRHS)
{
  if constexpr (IsNumber_v<T>)
  {
    return (Abs(inLHS) > Abs(inRHS)) ? inLHS : inRHS;
  }
  else
  {
    return MathMultiComponentApplied<T, MaxUnsigned<ValueType_t<T>>>(inLHS, inRHS);
  }
}

template <typename T>
constexpr auto MaxUnsigned(const T& inValue)
{
  if constexpr (IsNumber_v<T>)
  {
    return inValue;
  }
  else
  {
    auto max = math_multicomponent_detail::MinValue<ValueType_t<T>>();
    for (const auto& component : inValue) { max = Abs(MaxUnsigned(component)) > Abs(max) ? component : max; }
    return max;
  }
}

template <typename T>
constexpr auto MaxIndex(const T& inValue)
{
  if constexpr (IsNumber_v<T>)
  {
    return 0;
  }
  else
  {
    auto max_index = 0ul;
    for (std::size_t i = 0ul; i < NumComponents_v<T>; ++i)
    {
      if (inValue[i] > inValue[max_index])
        max_index = i;
    }
    return max_index;
  }
}

template <typename T>
constexpr auto MaxUnsignedIndex(const T& inValue)
{
  if constexpr (IsNumber_v<T>)
  {
    return 0;
  }
  else
  {
    auto max_index = 0ul;
    for (std::size_t i = 0ul; i < NumComponents_v<T>; ++i)
    {
      if (Abs(inValue[i]) > Abs(inValue[max_index]))
        max_index = i;
    }
    return max_index;
  }
}

template <typename T>
constexpr T Min()
{
  return math_multicomponent_detail::MinValue<T>();
}

template <typename T>
constexpr T Max()
{
  return math_multicomponent_detail::MaxValue<T>();
}

template <typename T>
constexpr auto Sign(const T& inValue)
{
  if constexpr (IsNumber_v<T>)
  {
    return (inValue < 0 ? static_cast<T>(-1) : static_cast<T>(1));
  }
  else
  {
    return MathMultiComponentApplied<T, Sign<ValueType_t<T>>>(inValue);
  }
}

template <typename T>
constexpr bool IsBetween(const T& inValue, const T& inMin, const T& inMax)
{
  return (inValue >= inMin) && (inValue <= inMax);
}

template <typename T>
constexpr T
Map(const T& inValue, const T& inSourceBegin, const T& inSourceEnd, const T& inTargetBegin, const T& inTargetEnd)
{
  const auto source_range = (inSourceEnd - inSourceBegin);
  const auto target_range = (inTargetEnd - inTargetBegin);
  const auto source_progress = ((inValue - inSourceBegin) / source_range);
  return (source_progress * target_range) + inTargetBegin;
}

template <typename T>
constexpr T Map01ToNeg1Pos1(const T& inValue)
{
  return Map(inValue, All<T>(0), All<T>(1), All<T>(-1), All<T>(1));
}

template <typename T>
constexpr T MapNeg1Pos1To01(const T& inValue)
{
  return Map(inValue, All<T>(-1), All<T>(1), All<T>(0), All<T>(1));
}

template <typename T, typename TInterpolationFactor>
constexpr T Lerp(const T& inFrom, const T& inTo, const TInterpolationFactor& t)
{
  return inFrom + (inTo - inFrom) * t;
}

template <typename T>
constexpr T DegreeToRad(const T& inV)
{
  constexpr T Factor = (Pi<T>() / 180.0);
  return inV * Factor;
}

template <typename T>
constexpr T RadToDegree(const T& inV)
{
  constexpr T Factor = (180.0 / Pi<T>());
  return inV * Factor;
}

}