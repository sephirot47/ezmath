#include <ez/Line.h>
#include <ez/Macros.h>
#include <ez/Quat.h>

namespace ez
{

template <typename T, std::size_t N>
Line<T, N>::Line(const Vec<T, N>& inOrigin, const Vec<T, N>& inDirection)
    : mOrigin { inOrigin }, mDirection { inDirection }
{
  EXPECTS(IsNormalized(inDirection));
}

template <typename T, std::size_t N>
void Line<T, N>::SetDirection(const Vec<T, N>& inDirection)
{
  EXPECTS(IsNormalized(inDirection));
  mDirection = inDirection;
}

template <typename T, std::size_t N>
constexpr Vec<T, N> Direction(const Line<T, N>& inLine)
{
  return inLine.GetDirection();
}

template <typename T, std::size_t N>
constexpr RotationType_t<T, N> Orientation(const Line<T, N>& inLine)
{
  return FromTo(Forward<Vec3<T>>(), Direction(inLine));
}

namespace line_intersections_detail
{
  template <typename T>
  bool HasValue(const std::optional<T>& inOptional)
  {
    return inOptional.has_value();
  }

  template <typename T>
  std::optional<T> GetMinIntersectionDistance(const std::array<std::optional<T>, 2>& inIntersectionDistances)
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
  bool AddIntersectionDistance(std::array<std::optional<T>, 2>& ioIntersectionDistances,
      const T& inIntersectionDistance)
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

  template <typename T>
  Line3<T> GetLineInCylinderSpace(const Cylinder<T>& inCylinder, const Line3<T>& inLine)
  {
    const auto cylinder_orientation_inv = -Orientation(inCylinder);
    const auto line_origin_local = (cylinder_orientation_inv * (inLine.GetOrigin() - inCylinder.GetDestiny()));
    const auto line_dir_local = (cylinder_orientation_inv * inLine.GetDirection());
    return { line_origin_local, line_dir_local };
  }

  template <typename T>
  auto IntersectCylinderWithoutCaps(const Vec3<T>& inRayOriginLocal,
      const Vec3<T>& inRayDirLocal,
      const Cylinder<T>& inCylinder)
  {
    const auto line_dir_local_2d = XY(inRayDirLocal);
    const auto line_dir_local_2d_length = Length(line_dir_local_2d);
    if (IsVeryEqual(line_dir_local_2d_length, static_cast<T>(0)))
      return std::array<std::optional<T>, 2> {};

    const auto line_dir_local_2d_norm = (line_dir_local_2d / line_dir_local_2d_length);
    const auto line_2d = Ray2<T> { XY(inRayOriginLocal), line_dir_local_2d_norm };
    const auto cylinder_section_circle = Circle<T> { Zero<Vec2<T>>(), inCylinder.GetRadius() };

    auto intersections = IntersectAll(line_2d, cylinder_section_circle);
    for (auto& intersection : intersections)
    {
      if (!intersection.has_value())
        continue;

      (*intersection) /= line_dir_local_2d_length;
      const auto intersection_point_local_z = (inRayOriginLocal[2] + inRayDirLocal[2] * (*intersection));
      if (intersection_point_local_z < 0 || (Sq(intersection_point_local_z) > SqLength(inCylinder)))
        intersection = std::nullopt;
    }
    return intersections;
  }
}

template <EIntersectMode TIntersectMode, typename T>
auto Intersect(const Line3<T>& inLine, const Plane<T>& inPlane)
{
  static_assert(TIntersectMode == EIntersectMode::ALL_INTERSECTIONS || TIntersectMode == EIntersectMode::ONLY_CLOSEST
          || TIntersectMode == EIntersectMode::ONLY_CHECK,
      "Unsupported EIntersectMode.");

  const auto line_dir_dot_plane_normal = Dot(Direction(inLine), inPlane.GetNormal());
  const auto is_very_parallel_to_plane = IsVeryEqual(line_dir_dot_plane_normal, static_cast<T>(0));
  if (is_very_parallel_to_plane)
  {
    if constexpr (TIntersectMode == EIntersectMode::ALL_INTERSECTIONS)
      return std::array { std::optional<T>() };
    else if constexpr (TIntersectMode == EIntersectMode::ONLY_CLOSEST)
      return std::optional<T>();
    else if constexpr (TIntersectMode == EIntersectMode::ONLY_CHECK)
      return false;
  }

  const auto dot_points_diff_normal = Dot(inPlane.GetArbitraryPoint() - inLine.GetOrigin(), inPlane.GetNormal());
  const auto intersect_distance_from_line_origin = (dot_points_diff_normal / line_dir_dot_plane_normal);
  if constexpr (TIntersectMode == EIntersectMode::ALL_INTERSECTIONS)
    return std::array { std::make_optional(intersect_distance_from_line_origin) };
  else if constexpr (TIntersectMode == EIntersectMode::ONLY_CLOSEST)
    return std::make_optional(intersect_distance_from_line_origin);
  else if constexpr (TIntersectMode == EIntersectMode::ONLY_CHECK)
    return true;
}

template <EIntersectMode TIntersectMode, typename T>
auto Intersect(const Plane<T>& inPlane, const Line3<T>& inLine)
{
  return Intersect<TIntersectMode>(inLine, inPlane);
}

template <EIntersectMode TIntersectMode, typename T, std::size_t N>
auto Intersect(const Line<T, N>& inLine, const AAHyperBox<T, N>& inAAHyperBox)
{
  static_assert(TIntersectMode == EIntersectMode::ALL_INTERSECTIONS || TIntersectMode == EIntersectMode::ONLY_CHECK,
      "Unsupported EIntersectMode.");

  const auto line_direction_inverse = (static_cast<T>(1) / Direction(inLine));
  const auto tbot = line_direction_inverse * (inAAHyperBox.GetMin() - inLine.GetOrigin());
  const auto ttop = line_direction_inverse * (inAAHyperBox.GetMax() - inLine.GetOrigin());
  const auto tmin = Min(ttop, tbot);
  const auto tmax = Max(ttop, tbot);
  const auto enter = Max(tmin);
  const auto exit = Min(tmax);
  const auto intersects = ((exit >= 0) && (enter < exit));

  if constexpr (TIntersectMode == EIntersectMode::ALL_INTERSECTIONS)
  {
    std::array<std::optional<T>, 2> intersections;
    if (intersects)
      intersections = { std::make_optional(enter), std::make_optional(exit) };
    return intersections;
  }
  else if constexpr (TIntersectMode == EIntersectMode::ONLY_CHECK)
    return intersects;
}

template <EIntersectMode TIntersectMode, typename T, std::size_t N>
auto Intersect(const AAHyperBox<T, N>& inAAHyperBox, const Line<T, N>& inLine)
{
  return Intersect<TIntersectMode, T, N>(inLine, inAAHyperBox);
}

template <EIntersectMode TIntersectMode, typename T, std::size_t N>
auto Intersect(const HyperSphere<T, N>& inHyperSphere, const Line<T, N>& inLine)
{
  static_assert(TIntersectMode == EIntersectMode::ALL_INTERSECTIONS || TIntersectMode == EIntersectMode::ONLY_CHECK,
      "Unsupported EIntersectMode");

  const auto line_origin = inLine.GetOrigin();
  const auto line_dir = inLine.GetDirection();
  const auto hyper_sphere_center = Center(inHyperSphere);
  const auto hyper_sphere_radius = inHyperSphere.GetRadius();
  const auto line_origin_local = (line_origin - hyper_sphere_center);
  const auto o = line_origin_local;
  const auto d = line_dir;
  const auto R = hyper_sphere_radius;

  const auto a = Dot(d, d);
  const auto b = static_cast<T>(2) * Dot(o, d);
  const auto c = SqLength(o) - Sq(R);

  const auto sqrt_number = (Sq(b) - static_cast<T>(4) * a * c);
  if constexpr (TIntersectMode == EIntersectMode::ONLY_CHECK)
    return sqrt_number >= 0;
  else if constexpr (TIntersectMode == EIntersectMode::ALL_INTERSECTIONS)
  {
    std::array<std::optional<T>, 2> intersections;
    if (sqrt_number >= 0)
    {
      const auto sqrt_result = Sqrt(sqrt_number);
      const auto a2 = a + a;
      intersections.at(0) = (-b + sqrt_result) / a2;
      intersections.at(1) = (-b - sqrt_result) / a2;
    }
    return intersections;
  }
}

template <EIntersectMode TIntersectMode, typename T, std::size_t N>
auto Intersect(const Line<T, N>& inLine, const HyperSphere<T, N>& inHyperSphere)
{
  return Intersect<TIntersectMode>(inHyperSphere, inLine);
}

template <EIntersectMode TIntersectMode, typename T>
auto Intersect(const Cylinder<T>& inCylinder, const Line3<T>& inLine)
{
  static_assert(TIntersectMode == EIntersectMode::ALL_INTERSECTIONS || TIntersectMode == EIntersectMode::ONLY_CHECK,
      "Unsupported EIntersectMode");

  const auto cylinder_sq_radius = Sq(inCylinder.GetRadius());
  const auto cylinder_sq_length = SqLength(inCylinder);
  const auto cylinder_length = Sqrt(cylinder_sq_length);
  const auto line_local = line_intersections_detail::GetLineInCylinderSpace(inCylinder, inLine);

  // Cylinder pipe without caps
  auto intersections = line_intersections_detail::IntersectCylinderWithoutCaps(line_local.GetOrigin(),
      Direction(line_local),
      inCylinder);

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
    const auto cap_plane_intersection = IntersectClosest(line_local, cap_plane);
    if (!cap_plane_intersection.has_value())
      continue;

    const auto cap_intersection_point = line_local.GetPoint(*cap_plane_intersection);
    const auto intersection_sq_distance_to_cap_plane_pos = SqLength(XY(cap_intersection_point));
    if (intersection_sq_distance_to_cap_plane_pos > cylinder_sq_radius)
      continue;

    if (!line_intersections_detail::AddIntersectionDistance(intersections, *cap_plane_intersection))
      break;
  }

  if constexpr (TIntersectMode == EIntersectMode::ALL_INTERSECTIONS)
    return intersections;
  else if constexpr (TIntersectMode == EIntersectMode::ONLY_CHECK)
    return std::any_of(intersections.cbegin(), intersections.cend(), &line_intersections_detail::HasValue<T>);
}

template <EIntersectMode TIntersectMode, typename T>
auto Intersect(const Line3<T>& inLine, const Cylinder<T>& inCylinder)
{
  return Intersect<TIntersectMode>(inCylinder, inLine);
}

template <EIntersectMode TIntersectMode, typename T>
auto Intersect(const Capsule<T>& inCapsule, const Line3<T>& inLine)
{
  static_assert(TIntersectMode == EIntersectMode::ALL_INTERSECTIONS || TIntersectMode == EIntersectMode::ONLY_CHECK,
      "Unsupported EIntersectMode");

  const auto origin_sphere = Sphere<T> { inCapsule.GetOrigin(), inCapsule.GetRadius() };
  const auto destiny_sphere = Sphere<T> { inCapsule.GetDestiny(), inCapsule.GetRadius() };
  const auto cylinder = Cylinder<T> { inCapsule.GetOrigin(), inCapsule.GetDestiny(), inCapsule.GetRadius() };
  if constexpr (TIntersectMode == EIntersectMode::ONLY_CHECK)
  {
    return IntersectCheck(origin_sphere, inLine) || IntersectCheck(destiny_sphere, inLine)
        || IntersectCheck(cylinder, inLine);
  }
  else if constexpr (TIntersectMode == EIntersectMode::ALL_INTERSECTIONS)
  {
    // Capsule pipe without caps
    const auto line_local = line_intersections_detail::GetLineInCylinderSpace(cylinder, inLine);
    auto intersections = line_intersections_detail::IntersectCylinderWithoutCaps(line_local.GetOrigin(),
        Direction(line_local),
        cylinder);

    for (const auto& sphere : { origin_sphere, destiny_sphere })
    {
      if (std::all_of(intersections.cbegin(), intersections.cend(), &line_intersections_detail::HasValue<T>))
        return intersections;

      const auto sphere_intersections = Intersect<EIntersectMode::ALL_INTERSECTIONS>(inLine, sphere);
      for (const auto& sphere_intersection : sphere_intersections)
      {
        if (!sphere_intersection.has_value())
          continue;

        if (Contains(cylinder, inLine.GetPoint(*sphere_intersection)))
          continue;

        if (!line_intersections_detail::AddIntersectionDistance(intersections, *sphere_intersection))
          break;
      }
    }

    return intersections;
  }
}

template <EIntersectMode TIntersectMode, typename T>
auto Intersect(const Line3<T>& inLine, const Capsule<T>& inCapsule)
{
  return Intersect<TIntersectMode>(inCapsule, inLine);
}

}