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

namespace ray_intersections_detail
{
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

template <EIntersectMode TIntersectMode, typename T, typename TPrimitive, std::size_t N>
auto Intersect(const Ray<T, N>& inRay, const TPrimitive& inPrimitive)
{
  static_assert(TIntersectMode == EIntersectMode::ALL_INTERSECTIONS || TIntersectMode == EIntersectMode::ONLY_CLOSEST
          || TIntersectMode == EIntersectMode::ONLY_CHECK,
      "Unsupported EIntersectMode.");

  constexpr auto Epsilon = static_cast<T>(1e-7);
  const auto ray_line = Line<T, N> { inRay.GetOrigin(), Direction(inRay) };
  auto intersections = IntersectAll(ray_line, inPrimitive);

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
    return ray_intersections_detail::GetMinIntersectionDistance(intersections);
  else if constexpr (TIntersectMode == EIntersectMode::ONLY_CHECK)
    return Contains(inPrimitive, inRay.GetOrigin());
}

template <EIntersectMode TIntersectMode, typename T, typename TPrimitive, std::size_t N>
auto Intersect(const TPrimitive& inPrimitive, const Ray<T, N>& inRay)
{
  return Intersect<TIntersectMode, T, TPrimitive, N>(inRay, inPrimitive);
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