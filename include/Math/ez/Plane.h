#pragma once

#include "ez/IntersectMode.h"
#include "ez/Macros.h"
#include "ez/MathForward.h"
#include "ez/MathInitializers.h"
#include "ez/Vec.h"
#include <optional>

namespace ez
{
template <typename T>
class Plane final
{
public:
  using ValueType = T;
  static constexpr auto NumDimensions = 3;

  Plane() = default;
  Plane(const Vec3<T>& inNormal, const T inDistanceFromOriginInNormalDirection);
  Plane(const Vec3<T>& inNormal, const Vec3<T>& inPlanePoint);
  Plane(const Plane&) = default;
  Plane& operator=(const Plane&) = default;
  Plane(Plane&&) = default;
  Plane& operator=(Plane&&) = default;
  ~Plane() = default;

  void SetNormal(const Vec3<T>& inNormal);
  void SetDistanceFromOrigin(const T inDistance);

  const Vec3<T>& GetNormal() const;
  T GetDistanceFromOrigin() const;
  Vec3<T> GetArbitraryPoint() const;

private:
  Vec3<T> mNormal = Forward<Vec3<T>>();      // A, B, C
  T mDistanceFromOrigin = static_cast<T>(0); // Distance from origin i-n the direction of the normal (D)
};

// Intersection functions
template <EIntersectMode TIntersectMode, typename T>
auto Intersect(const Ray<T, 3>& inRay, const Plane<T>& inPlane)
{
  static_assert(TIntersectMode == EIntersectMode::ALL_INTERSECTIONS || TIntersectMode == EIntersectMode::ONLY_CLOSEST
          || TIntersectMode == EIntersectMode::ONLY_CHECK,
      "Unsupported EIntersectMode.");

  const auto ray_dir_dot_plane_normal = Dot(inRay.GetDirection(), -inPlane.GetNormal());
  const auto is_very_parallel_to_plane = IsVeryEqual(ray_dir_dot_plane_normal, static_cast<T>(0));
  if (is_very_parallel_to_plane)
  {
    if constexpr (TIntersectMode == EIntersectMode::ONLY_CHECK)
    {
      return false;
    }
    else
    {
      return std::optional<T>();
    }
  }

  const auto& ray_dir_plane_normal_angle_cos = ray_dir_dot_plane_normal;
  const auto ray_origin_distance_to_plane = Distance(inRay.GetOrigin(), inPlane);
  const auto intersect_distance_from_ray_origin = (ray_origin_distance_to_plane / ray_dir_plane_normal_angle_cos);
  if (intersect_distance_from_ray_origin < static_cast<T>(0))
  {
    if constexpr (TIntersectMode == EIntersectMode::ONLY_CHECK)
    {
      return false;
    }
    else
    {
      return std::optional<T>();
    }
  }

  if constexpr (TIntersectMode == EIntersectMode::ONLY_CHECK)
  {
    return true;
  }
  else
  {
    return std::make_optional(intersect_distance_from_ray_origin);
  }
}
}

#include "ez/Plane.tcc"