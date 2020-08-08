#include "ez/MathCommon.h"
#include "ez/Ray.h"

namespace ez
{

template <typename T, std::size_t N>
Ray<T, N>::Ray(const Vec<T, N>& inOrigin, const Vec<T, N>& inDirection) : mOrigin(inOrigin)
{
  // Use setter to check direction is normalized
  SetDirection(inDirection);
}

template <typename T, std::size_t N>
void Ray<T, N>::SetDirection(const Vec<T, N>& inDirection)
{
  EXPECTS(IsNormalized(inDirection));
  mDirection = inDirection;
}

template <typename T, std::size_t N>
constexpr Vec3<T> Direction(const Ray<T, N>& inRay)
{
  return inRay.GetDirection();
}

template <typename T>
bool HasValue(const std::optional<T>& inOptional)
{
  return inOptional.has_value();
}

template <typename T>
std::optional<T> GetClosestIntersectionDistance(const std::array<std::optional<T>, 2>& inIntersectionDistances)
{
  if (inIntersectionDistances.at(0).has_value())
  {
    if (!inIntersectionDistances.at(1).has_value())
      return inIntersectionDistances.at(0);
    return std::make_optional(Min(*inIntersectionDistances.at(0), *inIntersectionDistances.at(1)));
  }
  return inIntersectionDistances.at(1);
}

template <typename T>
bool AddIntersectionDistance(std::array<std::optional<T>, 2>& ioIntersectionDistances, const T& inIntersectionDistance)
{
  if (!ioIntersectionDistances.at(0).has_value())
  {
    ioIntersectionDistances.at(0) = inIntersectionDistance;
    return true;
  }

  if (!ioIntersectionDistances.at(1).has_value())
    ioIntersectionDistances.at(1) = inIntersectionDistance;

  return false;
}

template <EIntersectMode TIntersectMode, typename T>
auto Intersect(const Ray<T, 3>& inRay, const Plane<T>& inPlane)
{
  static_assert(TIntersectMode == EIntersectMode::ALL_INTERSECTIONS || TIntersectMode == EIntersectMode::ONLY_CLOSEST
          || TIntersectMode == EIntersectMode::ONLY_CHECK,
      "Unsupported EIntersectMode.");

  const auto ray_dir_dot_plane_normal = Dot(inRay.GetDirection(), inPlane.GetNormal());
  const auto is_very_parallel_to_plane = IsVeryEqual(ray_dir_dot_plane_normal, static_cast<T>(0));
  if (is_very_parallel_to_plane)
  {
    if constexpr (TIntersectMode == EIntersectMode::ALL_INTERSECTIONS)
      return std::array { std::optional<T>() };
    else if constexpr (TIntersectMode == EIntersectMode::ONLY_CLOSEST)
      return std::optional<T>();
    else if constexpr (TIntersectMode == EIntersectMode::ONLY_CHECK)
      return false;
  }

  const auto dot_points_diff_normal = Dot(inPlane.GetArbitraryPoint() - inRay.GetOrigin(), inPlane.GetNormal());
  const auto intersect_distance_from_ray_origin = (dot_points_diff_normal / ray_dir_dot_plane_normal);
  const auto intersects = (intersect_distance_from_ray_origin >= static_cast<T>(0));
  if constexpr (TIntersectMode == EIntersectMode::ALL_INTERSECTIONS)
    return std::array { intersects ? std::make_optional(intersect_distance_from_ray_origin) : std::optional<T>() };
  else if constexpr (TIntersectMode == EIntersectMode::ONLY_CLOSEST)
    return intersects ? std::make_optional(intersect_distance_from_ray_origin) : std::optional<T>();
  else if constexpr (TIntersectMode == EIntersectMode::ONLY_CHECK)
    return intersects;
}

template <EIntersectMode TIntersectMode, typename T>
auto Intersect(const Plane<T>& inPlane, const Ray<T, 3>& inRay)
{
  return Intersect<TIntersectMode>(inRay, inPlane);
}

template <EIntersectMode TIntersectMode, typename T>
auto Intersect(const Ray<T, 3>& inRay, const AABox<T>& inAABox)
{
  static_assert(TIntersectMode == EIntersectMode::ALL_INTERSECTIONS || TIntersectMode == EIntersectMode::ONLY_CLOSEST
          || TIntersectMode == EIntersectMode::ONLY_CHECK,
      "Unsupported EIntersectMode.");

  const auto ray_direction_inverse = (static_cast<T>(1) / Direction(inRay));
  const auto tbot = ray_direction_inverse * (inAABox.GetMin() - inRay.GetOrigin());
  const auto ttop = ray_direction_inverse * (inAABox.GetMax() - inRay.GetOrigin());
  const auto tmin = Min(ttop, tbot);
  const auto tmax = Max(ttop, tbot);
  const auto enter = Max(tmin);
  const auto exit = Min(tmax);
  const auto intersects = ((exit >= 0) && (enter < exit));

  const auto IsValidIntersection = [](const T& in_intersection) {
    constexpr auto Epsilon = static_cast<T>(1e-7);
    return !std::isinf(in_intersection) && (in_intersection >= Epsilon);
  };

  if constexpr (TIntersectMode == EIntersectMode::ALL_INTERSECTIONS)
  {
    std::array<std::optional<T>, 2> intersection_distances;
    if (intersects)
    {
      const auto min_max = std::minmax(enter, exit);
      const auto min_max_arr = std::array { min_max.first, min_max.second };
      for (int i = 0; i < 2; ++i)
      {
        const auto& dist = min_max_arr.at(i);
        intersection_distances.at(i) = (IsValidIntersection(dist) ? std::make_optional(dist) : std::optional<T> {});
      }
    }
    return intersection_distances;
  }
  else if constexpr (TIntersectMode == EIntersectMode::ONLY_CLOSEST)
  {
    if (!intersects)
      return std::optional<T> {};
    const auto min_dist = Min(enter, exit);
    return (IsValidIntersection(min_dist) ? std::make_optional(min_dist) : std::optional<T> {});
  }
  else if constexpr (TIntersectMode == EIntersectMode::ONLY_CHECK)
  {
    return intersects;
  }
}

template <EIntersectMode TIntersectMode, typename T>
auto Intersect(const AABox<T>& inAABox, const Ray<T, 3>& inRay)
{
  return Intersect<TIntersectMode>(inRay, inAABox);
}

template <EIntersectMode TIntersectMode, typename T, std::size_t N>
auto Intersect(const HyperSphere<T, N>& inHyperSphere, const Ray<T, N>& inRay)
{
  static_assert(TIntersectMode == EIntersectMode::ALL_INTERSECTIONS || TIntersectMode == EIntersectMode::ONLY_CLOSEST
          || TIntersectMode == EIntersectMode::ONLY_CHECK,
      "Unsupported EIntersectMode");

  std::array<std::optional<T>, 2> intersections;

  const auto ray_origin = inRay.GetOrigin();
  const auto ray_dir = inRay.GetDirection();
  const auto hyper_sphere_center = Center(inHyperSphere);
  const auto hyper_sphere_radius = inHyperSphere.GetRadius();
  const auto ray_origin_local = (ray_origin - hyper_sphere_center);
  const auto o = ray_origin_local;
  const auto d = ray_dir;
  const auto R = hyper_sphere_radius;

  const auto a = Dot(d, d);
  const auto b = static_cast<T>(2) * Dot(o, d);
  const auto c = SqLength(o) - Sq(R);

  const auto sqrt_number = (Sq(b) - static_cast<T>(4) * a * c);
  if (sqrt_number >= 0)
  {
    const auto sqrt_result = Sqrt(sqrt_number);
    const auto a2 = a + a;
    intersections.at(0) = (-b + sqrt_result) / a2;
    intersections.at(1) = (-b - sqrt_result) / a2;

    for (auto& intersection : intersections)
    {
      if (*intersection < static_cast<T>(0))
      {
        intersection = std::nullopt;
        continue;
      }
    }
  }

  if constexpr (TIntersectMode == EIntersectMode::ALL_INTERSECTIONS)
    return intersections;
  else if constexpr (TIntersectMode == EIntersectMode::ONLY_CLOSEST)
    return GetClosestIntersectionDistance(intersections);
  else if constexpr (TIntersectMode == EIntersectMode::ONLY_CHECK)
    return std::any_of(intersections.cbegin(), intersections.cend(), &HasValue<T>);
}

template <EIntersectMode TIntersectMode, typename T, std::size_t N>
auto Intersect(const Ray<T, N>& inRay, const HyperSphere<T, N>& inHyperSphere)
{
  return Intersect<TIntersectMode>(inHyperSphere, inRay);
}

template <typename T>
auto IntersectCylinderWithoutCaps(const Vec3<T>& inRayOriginLocal,
    const Vec3<T>& inRayDirLocal,
    const Cylinder<T>& inCylinder,
    const T inCylinderSqRadius,
    const T inCylinderSqLength)
{
  std::array<std::optional<T>, 2> intersections;

  // Cylinder pipe without caps
  {
    const auto o = XY(inRayOriginLocal);
    const auto d = XY(inRayDirLocal);
    const auto R = inCylinder.GetRadius();

    const auto a = SqLength(d);
    const auto b = static_cast<T>(2) * Dot(o, d);
    const auto c = SqLength(o) - inCylinderSqRadius;

    const auto sqrt_number = (Sq(b) - static_cast<T>(4) * a * c);
    if (sqrt_number >= 0)
    {
      const auto sqrt_result = Sqrt(sqrt_number);
      const auto a2 = a + a;
      intersections.at(0) = (-b + sqrt_result) / a2;
      intersections.at(1) = (-b - sqrt_result) / a2;

      for (auto& intersection : intersections)
      {
        if (*intersection < static_cast<T>(0))
        {
          intersection = std::nullopt;
          continue;
        }

        const auto intersection_point_local_z = (inRayOriginLocal[2] + inRayDirLocal[2] * (*intersection));
        if (intersection_point_local_z < 0)
        {
          intersection = std::nullopt;
          continue;
        }

        if (Sq(intersection_point_local_z) > inCylinderSqLength)
        {
          intersection = std::nullopt;
          continue;
        }
      }
    }
  }

  return intersections;
}

template <typename T>
Ray3<T> GetRayInCylinderSpace(const Cylinder<T>& inCylinder, const Ray3<T>& inRay)
{
  const auto cylinder_orientation_inv = -Orientation(inCylinder);
  const auto ray_origin_local = (cylinder_orientation_inv * (inRay.GetOrigin() - inCylinder.GetDestiny()));
  const auto ray_dir_local = (cylinder_orientation_inv * inRay.GetDirection());
  return { ray_origin_local, ray_dir_local };
}

template <EIntersectMode TIntersectMode, typename T>
auto Intersect(const Cylinder<T>& inCylinder, const Ray<T, 3>& inRay)
{
  static_assert(TIntersectMode == EIntersectMode::ALL_INTERSECTIONS || TIntersectMode == EIntersectMode::ONLY_CLOSEST
          || TIntersectMode == EIntersectMode::ONLY_CHECK,
      "Unsupported EIntersectMode");

  const auto cylinder_sq_radius = Sq(inCylinder.GetRadius());
  const auto cylinder_sq_length = SqLength(inCylinder);
  const auto cylinder_length = Sqrt(cylinder_sq_length);
  const auto ray_local = GetRayInCylinderSpace(inCylinder, inRay);

  // Cylinder pipe without caps
  auto intersections = IntersectCylinderWithoutCaps(ray_local.GetOrigin(),
      Direction(ray_local),
      inCylinder,
      cylinder_sq_radius,
      cylinder_sq_length);

  // Caps
  for (int cap_i = 0; cap_i < 2; ++cap_i)
  {
    const auto already_have_all_intersections = std::all_of(intersections.cbegin(),
        intersections.cend(),
        [](const auto& in_intersection) { return in_intersection.has_value(); });

    if (already_have_all_intersections)
      break;

    const auto cap_plane_pos = Vec3<T> { 0, 0, cylinder_length * cap_i };
    const auto cap_plane = Plane<T> { Forward<Vec3<T>>(), cap_plane_pos };
    const auto cap_plane_intersection = IntersectClosest(ray_local, cap_plane);
    if (!cap_plane_intersection.has_value())
      continue;

    const auto cap_intersection_point = ray_local.GetPoint(*cap_plane_intersection);
    const auto intersection_sq_distance_to_cap_plane_pos = SqLength(XY(cap_intersection_point));
    if (intersection_sq_distance_to_cap_plane_pos > cylinder_sq_radius)
      continue;

    if (!AddIntersectionDistance(intersections, *cap_plane_intersection))
      break;
  }

  if constexpr (TIntersectMode == EIntersectMode::ALL_INTERSECTIONS)
    return intersections;
  else if constexpr (TIntersectMode == EIntersectMode::ONLY_CLOSEST)
    return GetClosestIntersectionDistance(intersections);
  else if constexpr (TIntersectMode == EIntersectMode::ONLY_CHECK)
    return std::any_of(intersections.cbegin(), intersections.cend(), &HasValue<T>);
}

template <EIntersectMode TIntersectMode, typename T>
auto Intersect(const Ray<T, 3>& inRay, const Cylinder<T>& inCylinder)
{
  return Intersect<TIntersectMode>(inCylinder, inRay);
}

template <EIntersectMode TIntersectMode, typename T>
auto Intersect(const Capsule<T>& inCapsule, const Ray<T, 3>& inRay)
{
  static_assert(TIntersectMode == EIntersectMode::ALL_INTERSECTIONS || TIntersectMode == EIntersectMode::ONLY_CLOSEST
          || TIntersectMode == EIntersectMode::ONLY_CHECK,
      "Unsupported EIntersectMode");

  const auto origin_sphere = Sphere<T> { inCapsule.GetOrigin(), inCapsule.GetRadius() };
  const auto destiny_sphere = Sphere<T> { inCapsule.GetDestiny(), inCapsule.GetRadius() };
  const auto cylinder = Cylinder<T> { inCapsule.GetOrigin(), inCapsule.GetDestiny(), inCapsule.GetRadius() };
  if constexpr (TIntersectMode == EIntersectMode::ONLY_CHECK)
  {
    return IntersectCheck(origin_sphere, inRay) || IntersectCheck(destiny_sphere, inRay)
        || IntersectCheck(cylinder, inRay);
  }
  else if constexpr (TIntersectMode == EIntersectMode::ALL_INTERSECTIONS
      || TIntersectMode == EIntersectMode::ONLY_CLOSEST)
  {
    // Capsule pipe without caps
    const auto ray_local = GetRayInCylinderSpace(cylinder, inRay);
    auto intersections = IntersectCylinderWithoutCaps(ray_local.GetOrigin(),
        Direction(ray_local),
        cylinder,
        Sq(cylinder.GetRadius()),
        SqLength(cylinder));

    for (const auto& sphere : { origin_sphere, destiny_sphere })
    {
      if (std::all_of(intersections.cbegin(), intersections.cend(), &HasValue<T>))
        return intersections;

      const auto sphere_intersections = Intersect<EIntersectMode::ALL_INTERSECTIONS>(inRay, sphere);
      for (const auto& sphere_intersection : sphere_intersections)
      {
        if (!sphere_intersection.has_value())
          continue;

        if (Contains(cylinder, inRay.GetPoint(*sphere_intersection)))
          continue;

        if (!AddIntersectionDistance(intersections, *sphere_intersection))
          break;
      }
    }

    if constexpr (TIntersectMode == EIntersectMode::ALL_INTERSECTIONS)
      return intersections;
    else if constexpr (TIntersectMode == EIntersectMode::ONLY_CLOSEST)
      return GetClosestIntersectionDistance(intersections);
  }
}

template <EIntersectMode TIntersectMode, typename T>
auto Intersect(const Ray<T, 3>& inRay, const Capsule<T>& inCapsule)
{
  return Intersect<TIntersectMode>(inCapsule, inRay);
}

// Transformation specialization
template <typename T, std::size_t N>
void Transform(Ray<T, N>& ioRayToTransform, const Transformation<ValueType_t<T>, N>& inTransformation)
{
  ioRayToTransform.SetOrigin(inTransformation.TransformedPoint(ioRayToTransform.GetOrigin()));
  ioRayToTransform.SetDirection(NormalizedSafe(inTransformation.TransformedDirection(ioRayToTransform.GetDirection())));
}

template <typename T, std::size_t N>
void Transform(Ray<T, N>& ioRayToTransform, const SquareMat<ValueType_t<T>, N>& inTransformMatrix)
{
  ioRayToTransform.SetOrigin(inTransformMatrix * ioRayToTransform.GetOrigin());
  ioRayToTransform.SetDirection(NormalizedSafe(inTransformMatrix * ioRayToTransform.GetDirection()));
}

template <typename T, std::size_t N>
void Transform(Ray<T, N>& ioRayToTransform, const SquareMat<T, N + 1>& inTransformMatrix)
{
  ioRayToTransform.SetOrigin(XYZ(inTransformMatrix * XYZ1(ioRayToTransform.GetOrigin())));
  ioRayToTransform.SetDirection(NormalizedSafe(XYZ(inTransformMatrix * XYZ0(ioRayToTransform.GetDirection()))));
}

template <typename T, std::size_t N>
void InverseTransform(Ray<T, N>& ioRayToTransform, const Transformation<ValueType_t<T>, N>& inTransformation)
{
  ioRayToTransform.SetOrigin(inTransformation.InverseTransformedPoint(ioRayToTransform.GetOrigin()));
  ioRayToTransform.SetDirection(
      NormalizedSafe(inTransformation.InverseTransformedDirection(ioRayToTransform.GetDirection())));
}
}