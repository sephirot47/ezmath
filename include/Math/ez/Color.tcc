#include <ez/Color.h>

namespace ez
{
template <typename T, std::size_t N>
Color<T, N> RGBToHSV(const Color<T, N>& inColorRGB)
{
  EXPECTS(IsBetween(Part<0, 3>(inColorRGB), Zero<Color3<T>>(), One<Color3<T>>()));

  auto result_hsv = inColorRGB;
  Part<0, 3>(result_hsv) = Zero<Color3<T>>();

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
  EXPECTS(IsBetween(Part<0, 3>(inColorHSV), Zero<Color3<T>>(), One<Color3<T>>()));

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
  return WithPart<0, 3>(One<TColor>(),
      Color3<ValueType_t<TColor>>(static_cast<ValueType_t<TColor>>(0),
          static_cast<ValueType_t<TColor>>(0),
          static_cast<ValueType_t<TColor>>(0)));
}

template <typename TColor>
constexpr TColor Gray()
{
  return WithPart<0, 3>(One<TColor>(),
      Color3<ValueType_t<TColor>>(static_cast<ValueType_t<TColor>>(0.5),
          static_cast<ValueType_t<TColor>>(0.5),
          static_cast<ValueType_t<TColor>>(0.5)));
}

template <typename TColor>
constexpr TColor White()
{
  return WithPart<0, 3>(One<TColor>(),
      Color3<ValueType_t<TColor>>(static_cast<ValueType_t<TColor>>(1),
          static_cast<ValueType_t<TColor>>(1),
          static_cast<ValueType_t<TColor>>(1)));
}

template <typename TColor>
constexpr TColor Red()
{
  return WithPart<0, 3>(One<TColor>(),
      Color3<ValueType_t<TColor>>(static_cast<ValueType_t<TColor>>(1),
          static_cast<ValueType_t<TColor>>(0),
          static_cast<ValueType_t<TColor>>(0)));
}

template <typename TColor>
constexpr TColor Green()
{
  return WithPart<0, 3>(One<TColor>(),
      Color3<ValueType_t<TColor>>(static_cast<ValueType_t<TColor>>(0),
          static_cast<ValueType_t<TColor>>(1),
          static_cast<ValueType_t<TColor>>(0)));
}

template <typename TColor>
constexpr TColor Blue()
{
  return WithPart<0, 3>(One<TColor>(),
      Color3<ValueType_t<TColor>>(static_cast<ValueType_t<TColor>>(0),
          static_cast<ValueType_t<TColor>>(0),
          static_cast<ValueType_t<TColor>>(1)));
}

template <typename TColor>
constexpr TColor Cyan()
{
  return WithPart<0, 3>(One<TColor>(),
      Color3<ValueType_t<TColor>>(static_cast<ValueType_t<TColor>>(0),
          static_cast<ValueType_t<TColor>>(1),
          static_cast<ValueType_t<TColor>>(1)));
}

template <typename TColor>
constexpr TColor Magenta()
{
  return WithPart<0, 3>(One<TColor>(),
      Color3<ValueType_t<TColor>>(static_cast<ValueType_t<TColor>>(1),
          static_cast<ValueType_t<TColor>>(0),
          static_cast<ValueType_t<TColor>>(1)));
}

template <typename TColor>
constexpr TColor Yellow()
{
  return WithPart<0, 3>(One<TColor>(),
      Color3<ValueType_t<TColor>>(static_cast<ValueType_t<TColor>>(1),
          static_cast<ValueType_t<TColor>>(1),
          static_cast<ValueType_t<TColor>>(0)));
}

template <typename TColor>
constexpr TColor Brown()
{
  return WithPart<0, 3>(One<TColor>(),
      Color3<ValueType_t<TColor>>(static_cast<ValueType_t<TColor>>(0.5),
          static_cast<ValueType_t<TColor>>(0.25),
          static_cast<ValueType_t<TColor>>(0)));
}

template <typename TColor>
constexpr TColor Orange()
{
  return WithPart<0, 3>(One<TColor>(),
      Color3<ValueType_t<TColor>>(static_cast<ValueType_t<TColor>>(1),
          static_cast<ValueType_t<TColor>>(0.5),
          static_cast<ValueType_t<TColor>>(0)));
}

template <typename TColor>
constexpr TColor Purple()
{
  return WithPart<0, 3>(One<TColor>(),
      Color3<ValueType_t<TColor>>(static_cast<ValueType_t<TColor>>(0.5),
          static_cast<ValueType_t<TColor>>(0),
          static_cast<ValueType_t<TColor>>(1)));
}

template <typename TColor>
constexpr TColor Pink()
{
  return WithPart<0, 3>(One<TColor>(),
      Color3<ValueType_t<TColor>>(static_cast<ValueType_t<TColor>>(1),
          static_cast<ValueType_t<TColor>>(0.5),
          static_cast<ValueType_t<TColor>>(0.5)));
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
  return WithPart<0, 3>(inColor, Part<0, 3>(inColor) * inValue);
}
}