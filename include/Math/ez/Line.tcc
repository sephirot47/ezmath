#include <ez/BinaryIndex.h>
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

template <typename T>
bool IsOnPositiveSide(const Line2<T>& inLine, const Vec2<T>& inPoint)
{
  return Dot(Perpendicular(Direction(inLine)), (inPoint - inLine.GetOrigin())) > 0;
}

namespace line_detail
{
  template <typename T>
  bool HasValue(const std::optional<T>& inOptional)
  {
    return inOptional.has_value();
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
  bool AddIntersectionDistance(std::array<std::optional<T>, 2>& ioIntersectionDistances,
      const std::optional<T>& inIntersectionDistance)
  {
    if (!inIntersectionDistance.has_value())
      return false;
    return AddIntersectionDistance(ioIntersectionDistances, *inIntersectionDistance);
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
  auto IntersectCylinderWithoutCaps(const Vec3<T>& inLineOriginLocal,
      const Vec3<T>& inLineDirLocal,
      const Cylinder<T>& inCylinder)
  {
    const auto line_dir_local_2d = XY(inLineDirLocal);
    const auto line_dir_local_2d_length = Length(line_dir_local_2d);
    if (IsVeryEqual(line_dir_local_2d_length, static_cast<T>(0)))
      return std::array<std::optional<T>, 2> {};

    const auto line_dir_local_2d_norm = (line_dir_local_2d / line_dir_local_2d_length);
    const auto line_2d = Line2<T> { XY(inLineOriginLocal), line_dir_local_2d_norm };
    const auto cylinder_section_circle = Circle<T> { Zero<Vec2<T>>(), inCylinder.GetRadius() };

    auto intersections = IntersectAll(line_2d, cylinder_section_circle);
    for (auto& intersection : intersections)
    {
      if (!intersection.has_value())
        continue;

      (*intersection) /= line_dir_local_2d_length;
      const auto intersection_point_local_z = (inLineOriginLocal[2] + inLineDirLocal[2] * (*intersection));
      if (intersection_point_local_z < 0 || (Sq(intersection_point_local_z) > SqLength(inCylinder)))
        intersection = std::nullopt;
    }
    return intersections;
  }

  template <typename T, std::size_t N, typename TLineLikePrimitive>
  T GetT(const TLineLikePrimitive& inLineLike, const Vec<T, N>& inPoint)
  {
    const auto line_dir = Direction(inLineLike);
    for (std::size_t i = 0; i < NumComponents_v<decltype(inPoint)>; ++i)
    {
      if (!IsVeryEqual(line_dir[i], static_cast<T>(0)))
        return (inPoint[i] - inLineLike.GetOrigin()[i]) / line_dir[i];
    }
    return static_cast<T>(0);
  }

  template <typename TFrom, typename TTo, typename T>
  void ChangeIntersectionOrigin(const TFrom& inFrom, const TTo& inTo, T& ioIntersection)
  {
    const auto from_direction = Direction(inFrom);
    const auto to_direction = Direction(inTo);
    for (std::size_t i = 0; i < NumComponents_v<decltype(Direction(std::declval<TFrom>()))>; ++i)
    {
      if (IsVeryEqual(to_direction[i], static_cast<T>(0)))
        continue;

      const auto point_coord = inFrom.GetOrigin()[i] + from_direction[i] * ioIntersection;
      ioIntersection = ((point_coord - inTo.GetOrigin()[i]) / to_direction[i]);
      return;
    }
  }

  template <typename TFrom, typename TTo, typename T>
  void ChangeIntersectionOrigin(const TFrom& inFrom, const TTo& inTo, std::optional<T>& ioIntersection)
  {
    if (ioIntersection)
      ChangeIntersectionOrigin(inFrom, inTo, *ioIntersection);
  }

  template <typename TFrom, typename TTo, typename T, std::size_t N>
  void ChangeIntersectionOrigin(const TFrom& inFrom, const TTo& inTo, std::array<std::optional<T>, N>& ioIntersections)
  {
    for (auto& intersection : ioIntersections) ChangeIntersectionOrigin(inFrom, inTo, intersection);
  }

  template <typename TFrom, typename TTo>
  void ChangeIntersectionOrigin(const TFrom& inFrom, const TTo& inTo, bool& ioIntersection)
  {
  }

  template <typename T>
  std::optional<T> GetMinIntersectionDistance(const std::array<std::optional<T>, 1>& inIntersectionDistances)
  {
    return inIntersectionDistances.front();
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
}

template <typename T>
constexpr T ClosestPointT(const Line2<T>& inLineLHS, const Line2<T>& inLineRHS)
{
  const auto intersection = IntersectClosest(inLineLHS, inLineRHS);
  if (intersection.has_value())
    return *intersection;

  // Lines are parallel. Pick a consistent closest point even when commuting parameters.
  auto line_lhs_less_than_line_rhs = false;
  for (std::size_t i = 0; i < 2; ++i)
  {
    if (inLineLHS.GetOrigin()[i] < inLineRHS.GetOrigin()[i])
      return ClosestPointT(inLineLHS, inLineRHS.GetOrigin());
    else if (inLineLHS.GetOrigin()[i] > inLineRHS.GetOrigin()[i])
      return static_cast<T>(0);
  }
  return static_cast<T>(0);
}

template <typename T>
constexpr T ClosestPointT(const Line3<T>& inLineLHS, const Line3<T>& inLineRHS)
{
  return static_cast<T>(0); // TODO
}

template <typename T, std::size_t N>
constexpr T ClosestPointT(const Line<T, N>& inLine, const Vec<T, N>& inPoint)
{
  const auto line_dir = Direction(inLine);
  const auto point_origin_vector = (inPoint - inLine.GetOrigin());
  return Dot(point_origin_vector, line_dir);
}

template <typename T, std::size_t N>
constexpr T ClosestPointT(const Line<T, N>& inLine, const Ray<T, N>& inRay)
{
  const auto ray_line = Line<T, N> { inRay.GetOrigin(), Direction(inRay) };
  const auto ray_line_closest_point_t = ClosestPointT(ray_line, inLine);
  const auto ray_line_closest_point_t_clamped = Max(ray_line_closest_point_t, static_cast<T>(0));
  const auto ray_closest_point = inRay.GetPoint(ray_line_closest_point_t_clamped);
  return ClosestPointT(inLine, ray_closest_point);
}

template <typename T, std::size_t N>
constexpr T ClosestPointT(const Line<T, N>& inLine, const Segment<T, N>& inSegment)
{
  const auto segment_sq_length = SqLength(inSegment);
  if (IsVeryEqual(segment_sq_length, static_cast<T>(0)))
    return ClosestPointT(inLine, inSegment.GetOrigin());

  const auto segment_line_closest_point_t = ClosestPointT(inSegment.GetLine(), inLine);
  const auto segment_length = Sqrt(segment_sq_length);
  const auto segment_line_closest_point_t_clamped
      = Clamp(segment_line_closest_point_t, static_cast<T>(0), segment_length);
  const auto segment_closest_point = inSegment.GetPoint(segment_line_closest_point_t_clamped);
  return ClosestPointT(inLine, segment_closest_point);
}

template <typename T, std::size_t N>
constexpr T ClosestPointT(const Line<T, N>& inLine, const AAHyperBox<T, N>& inAAHyperBox)
{
  return line_detail::GetT(inLine, ClosestPoint(inLine, inAAHyperBox));
}

template <typename T, std::size_t N>
constexpr T ClosestPointT(const Line<T, N>& inLine, const HyperBox<T, N>& inHyperBox)
{
  return line_detail::GetT(inLine, ClosestPoint(inLine, inHyperBox));
}

template <typename T, std::size_t N>
constexpr Vec<T, N> ClosestPoint(const Line<T, N>& inLine, const AAHyperBox<T, N>& inAAHyperBox)
{
  const auto aa_hyper_box_extents = inAAHyperBox.GetSize() / static_cast<T>(2);
  const auto hyper_box = HyperBox<T, N> { Center(inAAHyperBox), aa_hyper_box_extents, RotationTypeIdentity<T, N>() };
  return ClosestPoint(inLine, hyper_box);
}

template <typename T, std::size_t N>
constexpr Vec<T, N> ClosestPoint(const Line<T, N>& inLine, const HyperBox<T, N>& inHyperBox)
{
  auto min_sq_distance_from_line_to_hyper_box_point = Max<T>();
  auto closest_hyper_box_point_to_line = Max<Vec<T, N>>();
  const auto hyper_box_center = Center(inHyperBox);
  const auto hyper_box_orientation = Orientation(inHyperBox);
  for (std::size_t i = 0; i < HyperBox<T, N>::NumPoints; ++i)
  {
    if constexpr (N == 2)
    {
      const auto hyper_box_point = (hyper_box_center + inHyperBox.GetExtents() * (MakeBinaryIndex<N, T>(i) * 2 - 1));
      const auto hyper_box_point_rotated = Rotated(hyper_box_point, hyper_box_orientation);
      const auto sq_distance_from_line_to_hyper_box_point = SqDistance(inLine, hyper_box_point_rotated);
      if (sq_distance_from_line_to_hyper_box_point < min_sq_distance_from_line_to_hyper_box_point)
      {
        min_sq_distance_from_line_to_hyper_box_point = sq_distance_from_line_to_hyper_box_point;
        closest_hyper_box_point_to_line = hyper_box_point_rotated;
      }
    }
    else if constexpr (N == 3)
    {
      // TODO. Do same with edges.
    }
  }
  return closest_hyper_box_point_to_line;
}

template <typename T, std::size_t N, typename TPrimitive>
constexpr Vec<T, N> ClosestPoint(const Line<T, N>& inLine, const TPrimitive& inPrimitive)
{
  const auto t = ClosestPointT(inLine, inPrimitive);
  return inLine.GetPoint(t);
}

template <typename T, std::size_t N, typename TPrimitive>
constexpr T SqDistance(const Line<T, N>& inLine, const TPrimitive& inPrimitive)
{
  const auto closest_point_on_line = ClosestPoint(inLine, inPrimitive);
  const auto closest_point_on_primitive = ClosestPoint(inPrimitive, closest_point_on_line);
  return SqDistance(closest_point_on_line, closest_point_on_primitive);
}

template <EIntersectMode TIntersectMode, typename T>
auto Intersect(const Line2<T>& inLineLHS, const Line2<T>& inLineRHS)
{
  static_assert(TIntersectMode == EIntersectMode::ALL_INTERSECTIONS || TIntersectMode == EIntersectMode::ONLY_CLOSEST
          || TIntersectMode == EIntersectMode::ONLY_CHECK,
      "Unsupported EIntersectMode.");

  const auto lhs_dir = Direction(inLineLHS);
  const auto rhs_dir = Direction(inLineRHS);
  const auto denominator = (lhs_dir[0] * rhs_dir[1] - lhs_dir[1] * rhs_dir[0]);
  if (IsVeryEqual(denominator, static_cast<T>(0)))
  {
    if constexpr (TIntersectMode == EIntersectMode::ALL_INTERSECTIONS)
      return std::array { std::optional<T>() };
    else if constexpr (TIntersectMode == EIntersectMode::ONLY_CLOSEST)
      return std::optional<T>();
    else if constexpr (TIntersectMode == EIntersectMode::ONLY_CHECK)
      return false;
  }

  if constexpr (TIntersectMode == EIntersectMode::ALL_INTERSECTIONS || TIntersectMode == EIntersectMode::ONLY_CLOSEST)
  {
    const auto& lhs_ori = inLineLHS.GetOrigin();
    const auto& rhs_ori = inLineRHS.GetOrigin();
    const auto numerator = (rhs_dir[0] * (lhs_ori[1] - rhs_ori[1]) + rhs_dir[1] * (rhs_ori[0] - lhs_ori[0]));
    const auto t = (numerator / denominator);
    if constexpr (TIntersectMode == EIntersectMode::ALL_INTERSECTIONS)
      return std::array { std::make_optional(t) };
    else if constexpr (TIntersectMode == EIntersectMode::ONLY_CLOSEST)
      return std::make_optional(t);
  }
  else if constexpr (TIntersectMode == EIntersectMode::ONLY_CHECK)
    return true;
}

template <EIntersectMode TIntersectMode, typename T, std::size_t N>
auto Intersect(const Line<T, N>& inLine, const Vec<T, N>& inPoint)
{
  const auto intersection = Intersect<TIntersectMode>(inPoint, inLine);
  if constexpr (TIntersectMode == EIntersectMode::ALL_INTERSECTIONS)
    return std::array { intersection[0].has_value() ? line_detail::GetT(inLine, inPoint) : std::optional<T> {} };
  else if constexpr (TIntersectMode == EIntersectMode::ONLY_CLOSEST)
    return intersection.has_value() ? line_detail::GetT(inLine, inPoint) : std::optional<T> {};
  else if constexpr (TIntersectMode == EIntersectMode::ONLY_CHECK)
    return intersection;
}

template <EIntersectMode TIntersectMode, typename T, std::size_t N>
auto Intersect(const Line<T, N>& inLine, const Ray<T, N>& inRay)
{
  auto intersection = Intersect<TIntersectMode, T>(inRay, inLine);
  line_detail::ChangeIntersectionOrigin(inRay, inLine, intersection);
  return intersection;
}

template <EIntersectMode TIntersectMode, typename T, std::size_t N>
auto Intersect(const Line<T, N>& inLine, const Segment<T, N>& inSegment)
{
  auto intersection = Intersect<TIntersectMode, T>(inSegment, inLine);
  line_detail::ChangeIntersectionOrigin(inSegment, inLine, intersection);
  return intersection;
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

template <EIntersectMode TIntersectMode, typename T, std::size_t N>
auto Intersect(const Line<T, N>& inLine, const AAHyperBox<T, N>& inAAHyperBox)
{
  static_assert(TIntersectMode == EIntersectMode::ALL_INTERSECTIONS || TIntersectMode == EIntersectMode::ONLY_CLOSEST
          || TIntersectMode == EIntersectMode::ONLY_CHECK,
      "Unsupported EIntersectMode.");

  const auto line_direction_inverse = (static_cast<T>(1) / Direction(inLine));
  const auto tbot = line_direction_inverse * (inAAHyperBox.GetMin() - inLine.GetOrigin());
  const auto ttop = line_direction_inverse * (inAAHyperBox.GetMax() - inLine.GetOrigin());
  const auto tmin = Min(ttop, tbot);
  const auto tmax = Max(ttop, tbot);
  const auto enter = Max(tmin);
  const auto exit = Min(tmax);
  const auto intersects = (enter < exit);

  if constexpr (TIntersectMode == EIntersectMode::ALL_INTERSECTIONS)
  {
    std::array<std::optional<T>, 2> intersections;
    if (intersects)
      intersections = { std::make_optional(enter), std::make_optional(exit) };
    return intersections;
  }
  else if constexpr (TIntersectMode == EIntersectMode::ONLY_CLOSEST)
  {
    return intersects ? std::make_optional(Min(enter, exit)) : std::optional<T> {};
  }
  else if constexpr (TIntersectMode == EIntersectMode::ONLY_CHECK)
    return intersects;
}

template <EIntersectMode TIntersectMode, typename T, std::size_t N>
auto Intersect(const Line<T, N>& inLine, const HyperBox<T, N>& inHyperBox)
{
  const auto hyper_box_orientation_inv = -Orientation(inHyperBox);
  return Intersect<TIntersectMode>(Rotated(inLine, hyper_box_orientation_inv),
      MakeAAHyperBoxFromCenterSize(Rotated(Center(inHyperBox), hyper_box_orientation_inv), inHyperBox.GetSize()));
}

template <EIntersectMode TIntersectMode, typename T, std::size_t N>
auto Intersect(const Line<T, N>& inLine, const HyperSphere<T, N>& inHyperSphere)
{
  static_assert(TIntersectMode == EIntersectMode::ALL_INTERSECTIONS || TIntersectMode == EIntersectMode::ONLY_CLOSEST
          || TIntersectMode == EIntersectMode::ONLY_CHECK,
      "Unsupported EIntersectMode.");

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
  if constexpr (TIntersectMode == EIntersectMode::ALL_INTERSECTIONS || TIntersectMode == EIntersectMode::ONLY_CLOSEST)
  {
    std::array<std::optional<T>, 2> intersections;
    if (sqrt_number >= 0)
    {
      const auto sqrt_result = Sqrt(sqrt_number);
      const auto a2 = a + a;
      intersections.at(0) = (-b + sqrt_result) / a2;
      intersections.at(1) = (-b - sqrt_result) / a2;
    }

    if constexpr (TIntersectMode == EIntersectMode::ALL_INTERSECTIONS)
      return intersections;
    else if constexpr (TIntersectMode == EIntersectMode::ONLY_CLOSEST)
      return line_detail::GetMinIntersectionDistance(intersections);
  }
  else if constexpr (TIntersectMode == EIntersectMode::ONLY_CHECK)
    return sqrt_number >= 0;
}

template <EIntersectMode TIntersectMode, typename T>
auto Intersect(const Line3<T>& inLine, const Cylinder<T>& inCylinder)
{
  static_assert(TIntersectMode == EIntersectMode::ALL_INTERSECTIONS || TIntersectMode == EIntersectMode::ONLY_CLOSEST
          || TIntersectMode == EIntersectMode::ONLY_CHECK,
      "Unsupported EIntersectMode.");

  const auto cylinder_sq_radius = Sq(inCylinder.GetRadius());
  const auto cylinder_sq_length = SqLength(inCylinder);
  const auto cylinder_length = Sqrt(cylinder_sq_length);
  const auto line_local = line_detail::GetLineInCylinderSpace(inCylinder, inLine);

  // Cylinder pipe without caps
  auto intersections
      = line_detail::IntersectCylinderWithoutCaps(line_local.GetOrigin(), Direction(line_local), inCylinder);

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

    if (!line_detail::AddIntersectionDistance(intersections, *cap_plane_intersection))
      break;
  }

  if constexpr (TIntersectMode == EIntersectMode::ALL_INTERSECTIONS)
    return intersections;
  else if constexpr (TIntersectMode == EIntersectMode::ONLY_CLOSEST)
    return line_detail::GetMinIntersectionDistance(intersections);
  else if constexpr (TIntersectMode == EIntersectMode::ONLY_CHECK)
    return std::any_of(intersections.cbegin(), intersections.cend(), &line_detail::HasValue<T>);
}

template <EIntersectMode TIntersectMode, typename T, std::size_t N>
auto Intersect(const Line<T, N>& inLine, const Capsule<T, N>& inCapsule)
{
  static_assert(TIntersectMode == EIntersectMode::ALL_INTERSECTIONS || TIntersectMode == EIntersectMode::ONLY_CLOSEST
          || TIntersectMode == EIntersectMode::ONLY_CHECK,
      "Unsupported EIntersectMode.");

  const auto origin_hypersphere = HyperSphere<T, N> { inCapsule.GetOrigin(), inCapsule.GetRadius() };
  const auto destiny_hypersphere = HyperSphere<T, N> { inCapsule.GetDestiny(), inCapsule.GetRadius() };

  std::array<std::optional<T>, 2> intersections;
  std::conditional_t<N == 3, Cylinder<T>, HyperBox<T, N>> mid_section_primitive;
  if constexpr (N == 3)
  {
    // Middle cylinder
    const auto mid_cylinder = Cylinder<T> { inCapsule.GetOrigin(), inCapsule.GetDestiny(), inCapsule.GetRadius() };
    mid_section_primitive = mid_cylinder;

    if constexpr (TIntersectMode == EIntersectMode::ALL_INTERSECTIONS || TIntersectMode == EIntersectMode::ONLY_CLOSEST)
    {
      const auto line_local_mid_section = line_detail::GetLineInCylinderSpace(mid_section_primitive, inLine);
      intersections = line_detail::IntersectCylinderWithoutCaps(line_local_mid_section.GetOrigin(),
          Direction(line_local_mid_section),
          mid_section_primitive);
    }
  }
  else if constexpr (N == 2)
  {
    // Middle rect
    const auto capsule_length = Length(inCapsule);
    const auto capsule_direction = Direction(inCapsule);
    const auto capsule_perpendicular_direction = Perpendicular(capsule_direction);
    const auto capsule_length_vec = (capsule_direction * capsule_length);
    const auto capsule_radius_extent = (capsule_perpendicular_direction * inCapsule.GetRadius());
    const auto mid_rect_origin = inCapsule.GetOrigin() - capsule_radius_extent;
    const auto mid_rect_center = Center(inCapsule);
    const auto mid_rect_extents = Vec2<T> { capsule_length / static_cast<T>(2), inCapsule.GetRadius() };
    const auto mid_rect = HyperBox<T, N> { mid_rect_center, mid_rect_extents, Orientation(inCapsule) };
    if constexpr (TIntersectMode == EIntersectMode::ALL_INTERSECTIONS || TIntersectMode == EIntersectMode::ONLY_CLOSEST)
    {
      for (int segmenti = 0; segmenti < 2; ++segmenti)
      {
        const auto offset = (segmenti == 0 ? Zero<Vec2<T>>() : capsule_radius_extent * static_cast<T>(2));
        const auto mid_rect_segment_origin = mid_rect_origin + offset;
        const auto mid_rect_segment_destiny = mid_rect_segment_origin + capsule_length_vec;
        const auto mid_rect_segment = Segment2<T> { mid_rect_segment_origin, mid_rect_segment_destiny };
        const auto mid_rect_segment_intersection = IntersectClosest(inLine, mid_rect_segment);
        line_detail::AddIntersectionDistance(intersections, *mid_rect_segment_intersection);
      }
    }
    mid_section_primitive = mid_rect;
  }

  if constexpr (TIntersectMode == EIntersectMode::ONLY_CHECK)
  {
    return IntersectCheck(origin_hypersphere, inLine) || IntersectCheck(destiny_hypersphere, inLine)
        || IntersectCheck(mid_section_primitive, inLine);
  }

  if constexpr (TIntersectMode == EIntersectMode::ALL_INTERSECTIONS || TIntersectMode == EIntersectMode::ONLY_CLOSEST)
  {
    // HyperSphere caps
    for (const auto& hypersphere : { origin_hypersphere, destiny_hypersphere })
    {
      if (std::all_of(intersections.cbegin(), intersections.cend(), &line_detail::HasValue<T>))
        break;

      const auto hypersphere_intersections = Intersect<EIntersectMode::ALL_INTERSECTIONS>(inLine, hypersphere);
      for (const auto& hypersphere_intersection : hypersphere_intersections)
      {
        if (!hypersphere_intersection.has_value())
          continue;

        if (Contains(mid_section_primitive, inLine.GetPoint(*hypersphere_intersection)))
          continue;

        if (!line_detail::AddIntersectionDistance(intersections, *hypersphere_intersection))
          break;
      }
    }

    if constexpr (TIntersectMode == EIntersectMode::ALL_INTERSECTIONS)
      return intersections;
    else if constexpr (TIntersectMode == EIntersectMode::ONLY_CLOSEST)
      return line_detail::GetMinIntersectionDistance(intersections);
  }
}

template <EIntersectMode TIntersectMode, typename T, std::size_t N>
auto Intersect(const Line<T, N>& inLine, const Triangle<T, N>& inTriangle)
{
  static_assert(TIntersectMode == EIntersectMode::ALL_INTERSECTIONS || TIntersectMode == EIntersectMode::ONLY_CLOSEST
          || TIntersectMode == EIntersectMode::ONLY_CHECK,
      "Unsupported EIntersectMode.");

  if constexpr (N == 2)
  {
    if constexpr (TIntersectMode == EIntersectMode::ALL_INTERSECTIONS || TIntersectMode == EIntersectMode::ONLY_CLOSEST)
    {
      std::array<std::optional<T>, 2> intersections;
      line_detail::AddIntersectionDistance(intersections,
          IntersectClosest(inLine, Segment2<T> { inTriangle[0], inTriangle[1] }));
      line_detail::AddIntersectionDistance(intersections,
          IntersectClosest(inLine, Segment2<T> { inTriangle[1], inTriangle[2] }));
      line_detail::AddIntersectionDistance(intersections,
          IntersectClosest(inLine, Segment2<T> { inTriangle[2], inTriangle[0] }));

      if constexpr (TIntersectMode == EIntersectMode::ALL_INTERSECTIONS)
        return intersections;
      else if constexpr (TIntersectMode == EIntersectMode::ONLY_CLOSEST)
        return line_detail::GetMinIntersectionDistance(intersections);
    }
    else if constexpr (TIntersectMode == EIntersectMode::ONLY_CHECK)
    {
      const auto side01 = IsOnPositiveSide(inLine, inTriangle[0]);
      const auto side12 = IsOnPositiveSide(inLine, inTriangle[1]);
      const auto side20 = IsOnPositiveSide(inLine, inTriangle[2]);
      const auto intersects = !(side01 == side12 && side12 == side20);
      return intersects;
    }
  }
}

template <typename T, std::size_t N>
bool Contains(const Line<T, N>& inLine, const Vec<T, N>& inPoint)
{
  constexpr auto Epsilon = static_cast<T>(1e-7);
  return (SqDistance(inLine, inPoint) < Epsilon);
}

template <typename T, std::size_t N, typename TPrimitive>
bool Contains(const Line<T, N>& inLine, const TPrimitive& inPrimitive)
{
  return false;
}

template <typename T, std::size_t N>
constexpr Line<T, N> Translated(const Line<T, N>& inLine, const Vec<T, N>& inTranslation)
{
  return Line<T, N> { inLine.GetOrigin() + inTranslation, inLine.GetDirection() };
}

template <typename T, std::size_t N>
constexpr Line<T, N> Rotated(const Line<T, N>& inLine, const RotationType_t<T, N>& inRotation)
{
  return Line<T, N> { Rotated(inLine.GetOrigin(), inRotation), Rotated(inLine.GetDirection(), inRotation) };
}
}