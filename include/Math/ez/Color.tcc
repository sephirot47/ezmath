#include "ez/Color.h"

namespace ez
{
template <typename T, std::size_t N>
Color<T, N> RGBToHSV(const Color<T, N>& inColorRGB)
{
  EXPECTS(inColorRGB[0] >= 0.0f && inColorRGB[0] <= 1.0f);
  EXPECTS(inColorRGB[1] >= 0.0f && inColorRGB[1] <= 1.0f);
  EXPECTS(inColorRGB[2] >= 0.0f && inColorRGB[2] <= 1.0f);

  auto result_hsv = inColorRGB;
  result_hsv[0] = result_hsv[1] = result_hsv[2] = static_cast<T>(0);

  const auto max_rgb_comp = Max(inColorRGB);
  const auto result_value = max_rgb_comp;
  if (result_value == static_cast<T>(0))
    return result_hsv;
  result_hsv[2] = result_value;

  const auto min_rgb_comp = Min(inColorRGB);
  const auto rgb_min_max_delta = (max_rgb_comp - min_rgb_comp);
  const auto result_saturation = (rgb_min_max_delta / max_rgb_comp);
  if (result_saturation == static_cast<T>(0))
    return result_hsv;
  result_hsv[1] = result_saturation;

  // Hue
  const auto& in_r = inColorRGB[0];
  const auto& in_g = inColorRGB[1];
  const auto& in_b = inColorRGB[2];

  auto result_hue = static_cast<T>(0);
  {
    if (in_r == max_rgb_comp)
      result_hue = (in_g - in_b) / rgb_min_max_delta;
    else if (in_g == max_rgb_comp)
      result_hue = static_cast<T>(2) + (in_b - in_r) / rgb_min_max_delta;
    else
      result_hue = static_cast<T>(4) + (in_r - in_g) / rgb_min_max_delta;
    result_hue /= 6.0f;
    result_hue = std::fmod(result_hue + 100.0f, 1.0f);
  }

  result_hsv[0] = result_hue;

  return result_hsv;
}

template <typename T, std::size_t N>
Color<T, N> HSVToRGB(const Color<T, N>& inColorHSV)
{
  EXPECTS(inColorHSV[0] >= 0.0f && inColorHSV[0] <= 1.0f);
  EXPECTS(inColorHSV[1] >= 0.0f && inColorHSV[1] <= 1.0f);
  EXPECTS(inColorHSV[2] >= 0.0f && inColorHSV[2] <= 1.0f);

  auto result_rgb = inColorHSV;

  const auto in_hue = inColorHSV[0];
  const auto hue_from_0_to_6 = in_hue * static_cast<T>(6);
  const auto in_saturation = inColorHSV[1];
  const auto in_value = inColorHSV[2];

  const auto hue_bucket = static_cast<int>(hue_from_0_to_6);
  const auto hue_remainder = (hue_from_0_to_6 - hue_bucket);
  const auto p = in_value * (static_cast<T>(1) - in_saturation);
  const auto q = in_value * (static_cast<T>(1) - (in_saturation * hue_remainder));
  const auto t = in_value * (static_cast<T>(1) - (in_saturation * (static_cast<T>(1) - hue_remainder)));

  switch (hue_bucket)
  {
  case 0:
    result_rgb[0] = in_value;
    result_rgb[1] = t;
    result_rgb[2] = p;
    break;

  case 1:
    result_rgb[0] = q;
    result_rgb[1] = in_value;
    result_rgb[2] = p;
    break;

  case 2:
    result_rgb[0] = p;
    result_rgb[1] = in_value;
    result_rgb[2] = t;
    break;

  case 3:
    result_rgb[0] = p;
    result_rgb[1] = q;
    result_rgb[2] = in_value;
    break;

  case 4:
    result_rgb[0] = t;
    result_rgb[1] = p;
    result_rgb[2] = in_value;
    break;

  case 5:
  default:
    result_rgb[0] = in_value;
    result_rgb[1] = p;
    result_rgb[2] = q;
    break;
  }
  return result_rgb;
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
}