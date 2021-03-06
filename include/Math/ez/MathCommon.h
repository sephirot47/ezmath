#pragma once

#include <ez/MathForward.h>
#include <ez/MathInitializers.h>
#include <ez/MathMultiComponent.h>
#include <ez/MathRandom.h>
#include <ez/MathTypeTraits.h>
#include <cmath>
#include <cstdint>
#include <tuple>

namespace ez
{
template <typename T>
constexpr auto Dot(const T& inLHS, const T& inRHS)
{
  if constexpr (IsNumber_v<T>)
  {
    return (inLHS * inRHS);
  }
  else
  {
    auto dot = static_cast<ValueType_t<T>>(0);
    for (std::size_t i = 0; i < NumComponents_v<T>; ++i) { dot += inLHS[i] * inRHS[i]; }
    return dot;
  }
}

template <typename T>
constexpr auto SqLength(const T& inV)
{
  return Dot(inV, inV);
}

template <typename T>
constexpr auto Length(const T& inV)
{
  return std::sqrt(SqLength(inV));
}
template <typename T>
constexpr auto Sq(const T& inV)
{
  return inV * inV;
}
template <typename TLHS,
    typename TRHS,
    typename = std::enable_if_t<(IsNumber_v<TLHS> || IsVec_v<TLHS>)&&(IsNumber_v<TRHS> || IsVec_v<TRHS>)>>
constexpr auto SqDistance(const TLHS& inLHS, const TRHS& inRHS)
{
  const auto diff = (inRHS - inLHS);
  return SqLength(diff);
}

template <typename TLHS, typename TRHS>
constexpr auto Distance(const TLHS& inLHS, const TRHS& inRHS)
{
  if constexpr (IsNumber_v<TLHS> && IsNumber_v<TRHS>)
  {
    return Abs(inLHS - inRHS);
  }
  else
  {
    return std::sqrt(SqDistance(inLHS, inRHS));
  }
}

template <typename T>
constexpr bool
IsVeryEqual(const T& inLHS, const T& inRHS, const T& inEpsilon = All<T>(static_cast<ValueType_t<T>>(1e-4)))
{
  return Abs(inLHS - inRHS) < inEpsilon;
}

template <typename T>
constexpr bool IsNormalized(const T& inV)
{
  using ValueType = typename T::ValueType;
  return IsVeryEqual(SqLength(inV), static_cast<ValueType>(1));
}

template <typename T>
[[nodiscard]] constexpr T Normalized(const T& inV)
{
  const auto length = Length(inV);
  EXPECTS(length != 0);
  return inV / length;
}

template <typename T>
[[nodiscard]] constexpr T NormalizedSafe(const T& inV)
{
  const auto sq_length = SqLength(inV);
  if (sq_length == 0.0f)
    return inV;
  const auto length = std::sqrt(sq_length);
  return inV / length;
}
}