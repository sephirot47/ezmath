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

template <typename T>
bool IsOnPositiveSide(const Ray2<T>& inRay, const Vec2<T>& inPoint)
{
  return Dot(Perpendicular(Direction(inRay)), (inPoint - inRay.GetOrigin())) > 0;
}

// Intersect
template <EIntersectMode TIntersectMode, typename T, std::size_t N>
auto Intersect(const Ray<T, N>& inRay, const Vec<T, N>& inPoint)
{
  const auto intersection = Intersect<TIntersectMode>(inPoint, inRay);
  if constexpr (TIntersectMode == EIntersectMode::ALL_INTERSECTIONS)
    return std::array { intersection[0].has_value() ? line_detail::GetT(inRay, inPoint) : std::optional<T> {} };
  else if constexpr (TIntersectMode == EIntersectMode::ONLY_CLOSEST)
    return intersection.has_value() ? line_detail::GetT(inRay, inPoint) : std::optional<T> {};
  else if constexpr (TIntersectMode == EIntersectMode::ONLY_CHECK)
    return intersection;
}

template <EIntersectMode TIntersectMode, typename T, typename TPrimitive, std::size_t N>
auto Intersect(const Ray<T, N>& inRay, const TPrimitive& inPrimitive)
{
  static_assert(TIntersectMode == EIntersectMode::ALL_INTERSECTIONS || TIntersectMode == EIntersectMode::ONLY_CLOSEST
          || TIntersectMode == EIntersectMode::ONLY_CHECK,
      "Unsupported EIntersectMode.");

  constexpr auto Epsilon = static_cast<T>(1e-7);
  auto intersections = IntersectAll(inRay.GetLine(), inPrimitive);

  for (auto& intersection : intersections)
  {
    if (!intersection.has_value())
      continue;

    const auto point_inside_ray_range = (*intersection >= Epsilon);
    if constexpr (TIntersectMode == EIntersectMode::ONLY_CHECK)
    {
      if (point_inside_ray_range)
        return true;
    }
    else if (!point_inside_ray_range)
      intersection = std::nullopt;
  }

  if constexpr (TIntersectMode == EIntersectMode::ALL_INTERSECTIONS)
    return intersections;
  else if constexpr (TIntersectMode == EIntersectMode::ONLY_CLOSEST)
    return line_detail::GetMinIntersectionDistance(intersections);
  else if constexpr (TIntersectMode == EIntersectMode::ONLY_CHECK)
    return Contains(inPrimitive, inRay.GetOrigin());
}

// Contains
template <typename T, std::size_t N>
bool Contains(const Ray<T, N>& inRay, const Vec<T, N>& inPoint)
{
  constexpr auto Epsilon = static_cast<T>(1e-7);
  return (SqDistance(inRay, inPoint) < Epsilon);
}

template <typename T, std::size_t N, typename TPrimitive>
bool Contains(const Ray<T, N>& inRay, const TPrimitive& inPrimitive)
{
  return false;
}

template <typename T, std::size_t N, typename TPrimitive>
auto ClosestPointT(const Ray<T, N>& inRay, const TPrimitive& inPrimitive)
{
  const auto line_closest_point_t = ClosestPointT(inRay.GetLine(), inPrimitive);
  return Max(line_closest_point_t, static_cast<T>(0));
}

template <typename T, std::size_t N, typename TPrimitive>
auto ClosestPoint(const Ray<T, N>& inRay, const TPrimitive& inPrimitive)
{
  return inRay.GetPoint(ClosestPointT(inRay, inPrimitive));
}

template <typename T, std::size_t N>
auto SqDistance(const Ray<T, N>& inRay, const Vec<T, N>& inPoint)
{
  const auto closest_point_t = ClosestPointT(inRay, inPoint);
  if (closest_point_t < static_cast<T>(0))
    return SqDistance(inRay.GetOrigin(), inPoint);
  return SqDistance(inRay.GetPoint(closest_point_t), inPoint);
}

template <typename T, std::size_t N, typename TPrimitive>
auto SqDistance(const Ray<T, N>& inRay, const TPrimitive& inPrimitive)
{
  return SqDistance(inRay, ClosestPoint(inRay, inPrimitive));
}

template <typename T, std::size_t N>
constexpr Ray<T, N> Translated(const Ray<T, N>& inRay, const Vec<T, N>& inTranslation)
{
  return Ray<T, N> { inRay.GetOrigin() + inTranslation, inRay.GetDirection() };
}

template <typename T, std::size_t N>
constexpr Ray<T, N> Rotated(const Ray<T, N>& inRay, const RotationType_t<T, N>& inRotation)
{
  return Ray<T, N> { Rotated(inRay.GetOrigin(), inRotation), Rotated(inRay.GetDirection(), inRotation) };
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