#pragma once

#include "ez/MathForward.h"
#include "ez/Vec.h"
#include <cstdint>

namespace ez
{
template <typename T, std::size_t N>
Color<T, N> RGBToHSV(const Color<T, N>& inColorRGB);

template <typename T, std::size_t N>
Color<T, N> HSVToRGB(const Color<T, N>& inColorHSV);

template <typename TColor>
constexpr TColor Black();

template <typename TColor>
constexpr TColor Gray();

template <typename TColor>
constexpr TColor White();

template <typename TColor>
constexpr TColor Red();

template <typename TColor>
constexpr TColor Green();

template <typename TColor>
constexpr TColor Blue();

template <typename TColor>
constexpr TColor Cyan();

template <typename TColor>
constexpr TColor Magenta();

template <typename TColor>
constexpr TColor Yellow();

template <typename TColor>
constexpr TColor Brown();

template <typename TColor>
constexpr TColor Orange();

template <typename TColor>
constexpr TColor Purple();

template <typename TColor>
constexpr TColor Pink();

template <typename TColor>
constexpr TColor WithAlpha(const TColor& inColor, const typename TColor::ValueType inAlpha);

template <typename TColor>
constexpr TColor WithValue(const TColor& inColor, const typename TColor::ValueType inValue);
}

#include "ez/Color.tcc"