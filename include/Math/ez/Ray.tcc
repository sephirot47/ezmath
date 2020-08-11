#include "ez/Line.h"
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
constexpr Vec<T, N> Direction(const Ray<T, N>& inRay)
{
  return inRay.GetDirection();
}

template <EIntersectMode TIntersectMode, typename T>
auto Intersect(const Ray3<T>& inRay, const Plane<T>& inPlane)
{
  static_assert(TIntersectMode == EIntersectMode::ALL_INTERSECTIONS || TIntersectMode == EIntersectMode::ONLY_CLOSEST
          || TIntersectMode == EIntersectMode::ONLY_CHECK,
      "Unsupported EIntersectMode.");

  const auto ray_line = Line3<T> { inRay.GetOrigin(), Direction(inRay) };
  const auto intersection = Intersect<EIntersectMode::ONLY_CLOSEST, T>(ray_line, inPlane);

  const auto intersects = (intersection.has_value() && (*intersection >= static_cast<T>(0)));
  if constexpr (TIntersectMode == EIntersectMode::ALL_INTERSECTIONS)
    return std::array { intersects ? std::make_optional(*intersection) : std::optional<T>() };
  else if constexpr (TIntersectMode == EIntersectMode::ONLY_CLOSEST)
    return (intersects ? std::make_optional(*intersection) : std::optional<T>());
  else if constexpr (TIntersectMode == EIntersectMode::ONLY_CHECK)
    return intersects;
}

template <EIntersectMode TIntersectMode, typename T>
auto Intersect(const Plane<T>& inPlane, const Ray3<T>& inRay)
{
  return Intersect<TIntersectMode>(inRay, inPlane);
}

template <EIntersectMode TIntersectMode, typename T, std::size_t N>
auto Intersect(const Ray<T, N>& inRay, const AAHyperBox<T, N>& inAAHyperBox)
{
  static_assert(TIntersectMode == EIntersectMode::ALL_INTERSECTIONS || TIntersectMode == EIntersectMode::ONLY_CLOSEST
          || TIntersectMode == EIntersectMode::ONLY_CHECK,
      "Unsupported EIntersectMode.");

  const auto ray_line = Line<T, N> { inRay.GetOrigin(), Direction(inRay) };
  auto intersections = IntersectAll(ray_line, inAAHyperBox);

  for (auto& intersection : intersections)
  {
    if (!intersection.has_value())
      continue;

    constexpr auto Epsilon = static_cast<T>(1e-7);
    if (*intersection < Epsilon)
    {
      intersection = std::nullopt;
      continue;
    }
  }

  if constexpr (TIntersectMode == EIntersectMode::ALL_INTERSECTIONS)
    return intersections;
  else if constexpr (TIntersectMode == EIntersectMode::ONLY_CLOSEST)
    return line_intersections_detail::GetMinIntersectionDistance(intersections);
  else if constexpr (TIntersectMode == EIntersectMode::ONLY_CHECK)
    return false;
}

template <EIntersectMode TIntersectMode, typename T, std::size_t N>
auto Intersect(const AAHyperBox<T, N>& inAAHyperBox, const Ray<T, N>& inRay)
{
  return Intersect<TIntersectMode, T, N>(inRay, inAAHyperBox);
}

template <EIntersectMode TIntersectMode, typename T, std::size_t N>
auto Intersect(const Ray<T, N>& inRay, const AAHyperCube<T, N>& inAAHyperCube)
{
  return Intersect<TIntersectMode, T, N>(inRay, MakeAAHyperBoxFromAAHyperCube(inAAHyperCube));
}

template <EIntersectMode TIntersectMode, typename T, std::size_t N>
auto Intersect(const AAHyperCube<T, N>& inAAHyperCube, const Ray<T, N>& inRay)
{
  return Intersect<TIntersectMode, T, N>(inRay, inAAHyperCube);
}

template <EIntersectMode TIntersectMode, typename T, std::size_t N>
auto Intersect(const HyperSphere<T, N>& inHyperSphere, const Ray<T, N>& inRay)
{
  static_assert(TIntersectMode == EIntersectMode::ALL_INTERSECTIONS || TIntersectMode == EIntersectMode::ONLY_CLOSEST
          || TIntersectMode == EIntersectMode::ONLY_CHECK,
      "Unsupported EIntersectMode");

  const auto ray_line = Line<T, N> { inRay.GetOrigin(), Direction(inRay) };
  auto intersections = IntersectAll(ray_line, inHyperSphere);
  for (auto& intersection : intersections)
  {
    if (!intersection.has_value())
      continue;

    constexpr auto Epsilon = static_cast<T>(1e-7);
    if (*intersection < Epsilon)
    {
      intersection = std::nullopt;
      continue;
    }

    if constexpr (TIntersectMode == EIntersectMode::ONLY_CHECK)
      return true;
  }

  if constexpr (TIntersectMode == EIntersectMode::ALL_INTERSECTIONS)
    return intersections;
  else if constexpr (TIntersectMode == EIntersectMode::ONLY_CLOSEST)
    return line_intersections_detail::GetMinIntersectionDistance(intersections);
  else if constexpr (TIntersectMode == EIntersectMode::ONLY_CHECK)
    return std::any_of(intersections.cbegin(), intersections.cend(), &line_intersections_detail::HasValue<T>);
}

template <EIntersectMode TIntersectMode, typename T, std::size_t N>
auto Intersect(const Ray<T, N>& inRay, const HyperSphere<T, N>& inHyperSphere)
{
  return Intersect<TIntersectMode>(inHyperSphere, inRay);
}

template <EIntersectMode TIntersectMode, typename T>
auto Intersect(const Cylinder<T>& inCylinder, const Ray3<T>& inRay)
{
  static_assert(TIntersectMode == EIntersectMode::ALL_INTERSECTIONS || TIntersectMode == EIntersectMode::ONLY_CLOSEST
          || TIntersectMode == EIntersectMode::ONLY_CHECK,
      "Unsupported EIntersectMode");

  const auto ray_line = Line3<T> { inRay.GetOrigin(), Direction(inRay) };
  auto intersections = IntersectAll(ray_line, inCylinder);
  for (auto& intersection : intersections)
  {
    if (!intersection.has_value())
      continue;

    constexpr auto Epsilon = static_cast<T>(1e-7);
    if (*intersection < Epsilon)
    {
      intersection = std::nullopt;
      continue;
    }

    if constexpr (TIntersectMode == EIntersectMode::ONLY_CHECK)
      return true;
  }

  if constexpr (TIntersectMode == EIntersectMode::ALL_INTERSECTIONS)
    return intersections;
  else if constexpr (TIntersectMode == EIntersectMode::ONLY_CLOSEST)
    return line_intersections_detail::GetMinIntersectionDistance(intersections);
  else if constexpr (TIntersectMode == EIntersectMode::ONLY_CHECK)
    return std::any_of(intersections.cbegin(), intersections.cend(), &line_intersections_detail::HasValue<T>);
}

template <EIntersectMode TIntersectMode, typename T>
auto Intersect(const Ray3<T>& inRay, const Cylinder<T>& inCylinder)
{
  return Intersect<TIntersectMode>(inCylinder, inRay);
}

template <EIntersectMode TIntersectMode, typename T>
auto Intersect(const Capsule<T>& inCapsule, const Ray3<T>& inRay)
{
  static_assert(TIntersectMode == EIntersectMode::ALL_INTERSECTIONS || TIntersectMode == EIntersectMode::ONLY_CLOSEST
          || TIntersectMode == EIntersectMode::ONLY_CHECK,
      "Unsupported EIntersectMode");

  const auto ray_line = Line3<T> { inRay.GetOrigin(), Direction(inRay) };
  auto intersections = IntersectAll(ray_line, inCapsule);
  for (auto& intersection : intersections)
  {
    if (!intersection.has_value())
      continue;

    constexpr auto Epsilon = static_cast<T>(1e-7);
    if (*intersection < Epsilon)
    {
      intersection = std::nullopt;
      continue;
    }

    if constexpr (TIntersectMode == EIntersectMode::ONLY_CHECK)
      return true;
  }

  if constexpr (TIntersectMode == EIntersectMode::ALL_INTERSECTIONS)
    return intersections;
  else if constexpr (TIntersectMode == EIntersectMode::ONLY_CLOSEST)
    return line_intersections_detail::GetMinIntersectionDistance(intersections);
  else if constexpr (TIntersectMode == EIntersectMode::ONLY_CHECK)
    return std::any_of(intersections.cbegin(), intersections.cend(), &line_intersections_detail::HasValue<T>);
}

template <EIntersectMode TIntersectMode, typename T>
auto Intersect(const Ray3<T>& inRay, const Capsule<T>& inCapsule)
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