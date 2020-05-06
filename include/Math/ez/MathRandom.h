#pragma once

#include "ez/Math.h"
#include "ez/MathInitializers.h"
#include "ez/MathMultiComponent.h"

namespace ez
{
template <typename T>
constexpr auto RandomUnit()
{
  if constexpr (IsNumber_v<T>)
  {
    return static_cast<T>(static_cast<double>(rand()) / RAND_MAX);
  }
  else if constexpr (IsQuat_v<T>)
  {
    using ValueType = typename T::ValueType;
    return AngleAxis(RandomUnit<ValueType>() * FullCircleRads<ValueType>(), RandomUnit<Vec3<ValueType>>());
  }
  else
  {
    const auto result = MathMultiComponentGenerated<T, RandomUnit<ValueType_t<T>>>();
    const auto normalized_result = Normalized(result);
    return normalized_result;
  }
}

template <typename T>
// Min always included. Max excluded in integer types, included in real types.
constexpr auto Random(const T& inMin, const T& inMax)
{
  if constexpr (IsNumber_v<T>)
  {
    return static_cast<T>(RandomUnit<double>() * (inMax - inMin) + inMin);
  }
  else
  {
    return MathMultiComponentGenerated<T, Random<ValueType_t<T>>>(inMin, inMax);
  }
}

template <typename T>
constexpr auto RandomSign()
{
  if constexpr (IsNumber_v<T>)
  {
    return (rand() % 2 == 0) ? static_cast<T>(1) : static_cast<T>(-1);
  }
  else
  {
    return MathMultiComponentGenerated<T, RandomSign<ValueType_t<T>>>();
  }
}
}