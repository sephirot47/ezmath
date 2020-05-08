#pragma once

#include "ez/MathInitializers.h"
#include "ez/MathTypeTraits.h"
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
    for (std::size_t i = 0; i < T::NumComponents; ++i)
    {
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
    for (std::size_t i = 0; i < T::NumComponents; ++i)
    {
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
constexpr auto Pow(const T& inValue, const T& inPower)
{
  if constexpr (IsNumber_v<T>)
  {
    return std::pow(inValue, inPower);
  }
  else
  {
    return MathMultiComponentApplied<T, Pow<ValueType_t<T>>>(inValue, inPower);
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
    auto min = std::numeric_limits<ValueType_t<T>>::max();
    for (const auto& component : inValue) { min = std::min(min, Min(component)); }
    return min;
  }
}

template <typename T>
constexpr auto Min()
{
  return All<T>(std::numeric_limits<ValueType_t<T>>::min());
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
    auto max = std::numeric_limits<ValueType_t<T>>::min();
    for (const auto& component : inValue) { max = std::max(max, Max(component)); }
    return max;
  }
}

template <typename T>
constexpr auto Max()
{
  return All<T>(std::numeric_limits<ValueType_t<T>>::max());
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