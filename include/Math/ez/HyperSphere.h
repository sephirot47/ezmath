#pragma once

#include "ez/IntersectMode.h"
#include "ez/MathCommon.h"
#include "ez/MathForward.h"
#include "ez/MathInitializers.h"
#include "ez/Vec.h"
#include <array>
#include <optional>

namespace ez
{
template <typename T, std::size_t N>
class HyperSphere final
{
public:
  using ValueType = T;
  static constexpr auto NumDimensions = N;

  HyperSphere() = default;
  HyperSphere(const Vec<T, N>& inCenter, const T& inRadius) : mCenter(inCenter), mRadius(inRadius) {}

  void SetCenter(const Vec<T, N>& inCenter) { mCenter = inCenter; }
  void SetRadius(const T& inRadius) { mRadius = inRadius; }

  const Vec<T, N>& GetCenter() const { return mCenter; }
  const T& GetRadius() const { return mRadius; }

private:
  Vec<T, N> mCenter = Zero<Vec<T, N>>();
  T mRadius = static_cast<T>(0);
};

// Intersection functions
template <EIntersectMode TIntersectMode, typename T, std::size_t N>
auto Intersect(const HyperSphere<T, N>& inLHS, const HyperSphere<T, N>& inRHS)
{
  static_assert(TIntersectMode == EIntersectMode::ONLY_CHECK, "Unsupported EIntersectMode.");
  return SqDistance(inLHS.GetCenter(), inRHS.GetCenter()) <= Sq(inLHS.GetRadius() + inRHS.GetRadius());
}

template <EIntersectMode TIntersectMode, typename T, std::size_t N>
auto Intersect(const HyperSphere<T, N>& inHyperSphere, const Ray<T, N>& inRay)
{
  static_assert(TIntersectMode == EIntersectMode::ALL_INTERSECTIONS || TIntersectMode == EIntersectMode::ONLY_CLOSEST
          || TIntersectMode == EIntersectMode::ONLY_CHECK,
      "Unsupported EIntersectMode");

  const auto ray_origin = inRay.GetOrigin();
  const auto ray_dir = inRay.GetDirection();
  const auto hyper_sphere_center = inHyperSphere.GetCenter();
  const auto hyper_sphere_radius = inHyperSphere.GetRadius();
  const auto o = (ray_origin - hyper_sphere_center);
  const auto d = ray_dir;
  const auto R = hyper_sphere_radius;

  const auto a = Dot(d, d);
  const auto b = static_cast<T>(2) * Dot(o, d);
  const auto c = SqLength(o) - Sq(R);

  const auto sqrt_number = (Sq(b) - static_cast<T>(4) * a * c);
  if (sqrt_number < 0)
  {
    if constexpr (TIntersectMode == EIntersectMode::ALL_INTERSECTIONS)
    {
      return std::array<std::optional<T>, 2> {};
    }
    else if constexpr (TIntersectMode == EIntersectMode::ONLY_CLOSEST)
    {
      return std::optional<T>();
    }
    else if constexpr (TIntersectMode == EIntersectMode::ONLY_CHECK)
    {
      return false;
    }
  }

  const auto sqrt_result = Sqrt(sqrt_number);
  const auto a2 = a * static_cast<T>(2);
  const auto k0 = (-b + sqrt_result) / a2;
  const auto k1 = (-b - sqrt_result) / a2;
  if constexpr (TIntersectMode == EIntersectMode::ALL_INTERSECTIONS)
  {
    std::array<std::optional<T>, 2> intersections;
    intersections[0] = (k0 >= 0) ? std::make_optional(k0) : std::optional<T>();
    intersections[1] = (k1 >= 0) ? std::make_optional(k1) : std::optional<T>();
    return intersections;
  }
  else if constexpr (TIntersectMode == EIntersectMode::ONLY_CLOSEST)
  {
    if (k0 < 0)
    {
      if (k1 < 0)
        return std::optional<T>();
      return std::make_optional(k1);
    }
    else
    {
      if (k1 < 0)
        return std::make_optional(k0);
      else
        return std::make_optional(Min(k0, k1));
    }
  }
  else if constexpr (TIntersectMode == EIntersectMode::ONLY_CHECK)
  {
    return k0 >= 0 || k1 >= 0;
  }
}

template <EIntersectMode TIntersectMode, typename T, std::size_t N>
auto Intersect(const Ray<T, N>& inRay, const HyperSphere<T, N>& inHyperSphere)
{
  return Intersect<TIntersectMode>(inHyperSphere, inRay);
}

template <typename T, std::size_t N>
bool Contains(const HyperSphere<T, N>& inHyperSphere, const Vec<T, N>& inPoint)
{
  return SqDistance(inPoint, inHyperSphere.GetCenter()) <= Sq(inHyperSphere.GetRadius());
}
}