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

}