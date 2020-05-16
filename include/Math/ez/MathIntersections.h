#pragma once

#include "MathIntersections.h"
#include "ez/AACube.h"
#include "ez/AARect.h"
#include "ez/Geometry.h"
#include "ez/HyperSphere.h"
#include "ez/Plane.h"
#include "ez/Ray.h"
#include "ez/Triangle.h"
#include "ez/Vec.h"
#include <functional>
#include <type_traits>

using namespace std::placeholders;

namespace ez
{
// SAT Normals
template <typename T>
constexpr auto GetSATNormals(const AACube<T>&)
{
  return std::array { Right<Vec3<T>>(), Up<Vec3<T>>(), Forward<Vec3<T>>() };
}

template <typename T>
constexpr auto GetSATNormals(const AARect<T>&)
{
  return std::array { Right<Vec3<T>>(), Up<Vec3<T>>() };
}

template <typename T>
auto GetSATNormals(const Triangle3<T>& inTriangle)
{
  return std::array { Normal(inTriangle) };
}

template <typename T>
constexpr auto GetSATNormals(const Triangle2<T>&)
{
  return std::array<Vec2<T>, 0> {};
}

// SAT Edges
template <typename T>
constexpr auto GetSATEdges(const AACube<T>&)
{
  return std::array { Right<Vec3<T>>(), Up<Vec3<T>>(), Forward<Vec3<T>>() };
}

template <typename T>
constexpr auto GetSATEdges(const AARect<T>&)
{
  return std::array { Right<Vec2<T>>(), Up<Vec2<T>>() };
}

template <typename T, std::size_t N>
auto GetSATEdges(const Triangle<T, N>& inTriangle)
{
  return std::array { (inTriangle[0] - inTriangle[1]),
    (inTriangle[0] - inTriangle[2]),
    (inTriangle[1] - inTriangle[2]) };
}

template <typename TLHSConvexObject, typename TRHSConvexObject>
bool DoProjectionsOverlap(const TLHSConvexObject& inLHS,
    const TRHSConvexObject& inRHS,
    const typename decltype(GetSATNormals(inLHS))::value_type& inProjectionAxis)
{
  using VecType = typename decltype(GetSATNormals(inLHS))::value_type;
  using ValueType = ValueType_t<VecType>;

  auto min_lhs_projection = Max<ValueType>();
  auto max_lhs_projection = Min<ValueType>();
  for (const auto& lhs_point : inLHS)
  {
    const auto lhs_projection = Dot(lhs_point, inProjectionAxis);
    min_lhs_projection = Min(min_lhs_projection, lhs_projection);
    max_lhs_projection = Max(max_lhs_projection, lhs_projection);
  }

  auto min_rhs_projection = Max<ValueType>();
  auto max_rhs_projection = Min<ValueType>();
  for (const auto& rhs_point : inRHS)
  {
    const auto rhs_projection = Dot(rhs_point, inProjectionAxis);
    min_rhs_projection = Min(min_rhs_projection, rhs_projection);
    max_rhs_projection = Max(max_rhs_projection, rhs_projection);
  }

  const auto there_is_no_projection_overlap
      = (min_lhs_projection > max_rhs_projection) || (min_rhs_projection > max_lhs_projection);
  const auto there_is_projection_overlap = !there_is_no_projection_overlap;
  return there_is_projection_overlap;
}

template <typename TLHSConvexObject, typename TRHSConvexObject>
bool Intersect(const TLHSConvexObject& inLHS, const TRHSConvexObject& inRHS)
{
  const auto ProjectionsOverlap = std::bind(DoProjectionsOverlap<TLHSConvexObject, TRHSConvexObject>, inLHS, inRHS, _1);
  const auto AxisSeparatesObjects = [&](const auto& inAxis) { return !ProjectionsOverlap(inAxis); };

  // Test LHS normals
  const auto lhs_normals = GetSATNormals(inLHS);
  if (std::any_of(lhs_normals.cbegin(), lhs_normals.cend(), AxisSeparatesObjects))
    return false;

  // Test RHS normals
  const auto rhs_normals = GetSATNormals(inRHS);
  if (std::any_of(rhs_normals.cbegin(), rhs_normals.cend(), AxisSeparatesObjects))
    return false;

  // Test combined axes (LHS-to-RHS cross product of edges)
  const auto lhs_edges = GetSATEdges(inLHS);
  const auto rhs_edges = GetSATEdges(inRHS);
  for (const auto& lhs_edge : lhs_edges)
  {
    for (const auto& rhs_edge : rhs_edges)
    {
      const auto cross_axis = Cross(lhs_edge, rhs_edge);
      if (AxisSeparatesObjects(cross_axis))
        return false;
    }
  }

  return true;
}

template <typename T>
std::optional<T> Intersect(const Ray3<T>& inRay, const Plane<T>& inPlane)
{
  const auto ray_dir_dot_plane_normal = Dot(inRay.GetDirection(), -inPlane.GetNormal());
  const auto is_very_parallel_to_plane = IsVeryEqual(ray_dir_dot_plane_normal, static_cast<T>(0));
  if (is_very_parallel_to_plane)
    return std::nullopt;

  const auto& ray_dir_plane_normal_angle_cos = ray_dir_dot_plane_normal;
  const auto ray_origin_distance_to_plane = Distance(inRay.GetOrigin(), inPlane);
  const auto intersect_distance_from_ray_origin = (ray_origin_distance_to_plane / ray_dir_plane_normal_angle_cos);
  if (intersect_distance_from_ray_origin < static_cast<T>(0))
    return std::nullopt;

  return intersect_distance_from_ray_origin;
}

template <typename T>
std::optional<T> Intersect(const Ray3<T>& inRay, const Triangle3<T>& inTriangle)
{
  const auto ray_plane_intersection_distance = Intersect(inRay, GetPlane(inTriangle));
  if (!ray_plane_intersection_distance)
    return std::nullopt;

  const auto ray_plane_intersection_point = inRay.GetPoint(*ray_plane_intersection_distance);
  const auto barycentric_coordinates = BarycentricCoordinates(inTriangle, ray_plane_intersection_point);
  const auto intersection_point_is_in_triangle = IsBetween(barycentric_coordinates, Zero<Vec3<T>>(), One<Vec3<T>>());
  if (!intersection_point_is_in_triangle)
    return std::nullopt;

  return ray_plane_intersection_distance;
}

template <typename T, std::size_t N>
bool Intersect(const HyperSphere<T, N>& inLHS, const HyperSphere<T, N>& inRHS)
{
  return SqDistance(inLHS.GetCenter(), inRHS.GetCenter()) <= Sq(inLHS.GetRadius() + inRHS.GetRadius());
}

template <typename T, std::size_t N>
bool Contains(const HyperSphere<T, N>& inHyperSphere, const Vec<T, N>& inPoint)
{
  return SqDistance(inPoint, inHyperSphere.GetCenter()) <= Sq(inHyperSphere.GetRadius());
}

template <typename T>
auto Intersect(const Triangle3<T>& inTriangle, const Ray3<T>& inRay)
{
  return Intersect(inRay, inTriangle);
}

template <typename T>
std::array<std::optional<T>, 2> Intersect(const Ray3<T>& inRay, const AACube<T>& inAACube)
{
  constexpr auto CubeFaceNormals = std::array {
    // Order important for loop below.
    Left<Vec3<T>>(),
    Down<Vec3<T>>(),
    Forward<Vec3<T>>(),
    Right<Vec3<T>>(),
    Up<Vec3<T>>(),
    Back<Vec3<T>>(),
  };

  const auto aacube_size = inAACube.GetSize();

  auto intersections_found = 0;
  std::array<std::optional<T>, 2> intersection_distances;
  for (std::size_t i = 0; i < CubeFaceNormals.size(); ++i) // For each cube face
  {
    // Check whether it intersects the plane
    const auto& aacube_face_normal = CubeFaceNormals[i];
    const auto& aacube_face_point = (i < 3) ? inAACube.GetMin() : inAACube.GetMax();
    const auto aacube_face_plane = Planef(aacube_face_normal, aacube_face_point);
    if (const auto intersection_distance = Intersect(inRay, aacube_face_plane))
    {
      // If it intersects the plane, then check if the intersection point is inside the cube
      const auto intersection_point = inRay.GetPoint(*intersection_distance);

      auto aacube_face_normal_projector = One<Vec3<T>>();
      aacube_face_normal_projector[i % 3] = static_cast<T>(0);

      const auto reprojected_intersection_point = aacube_face_normal_projector * intersection_point;
      const auto reprojected_cube_face_min = aacube_face_normal_projector * inAACube.GetMin();
      const auto reprojected_cube_face_max = aacube_face_normal_projector * inAACube.GetMax();

      const auto intersection_point_contained_in_aacube_face
          = IsBetween(reprojected_intersection_point, reprojected_cube_face_min, reprojected_cube_face_max);

      if (intersection_point_contained_in_aacube_face)
      {
        intersection_distances[intersections_found] = intersection_distance;

        ++intersections_found;
        if (intersections_found == 2)
          break;
      }
    }
  }

  return intersection_distances;
}

template <typename T>
auto Intersect(const AACube<T>& inLHS, const AACube<T>& inRHS)
{
  return inLHS.GetMin() <= inRHS.GetMax() || inRHS.GetMin() <= inLHS.GetMax();
}

template <typename T>
auto Intersect(const AACube<T>& inAACube, const Ray3<T>& inRay)
{
  return Intersect(inRay, inAACube);
}

template <typename T, std::size_t N, typename TRHSConvexObject>
bool Intersect(const Vec<T, N>& inPoint, const TRHSConvexObject& inRHS)
{
  return Contains(inRHS, inPoint);
}

template <typename T, std::size_t N, typename TLHSConvexObject>
bool Intersect(const TLHSConvexObject& inLHS, const Vec<T, N>& inPoint)
{
  return Intersect(inPoint, inLHS);
}

template <typename T, std::size_t N>
bool Intersect(const Plane<T>& inPlane, const Ray<T, 3>& inRay)
{
  return Intersect(inPlane, inRay).has_value();
}

template <typename T, std::size_t N>
bool Contains(const AAHyperRectangle<T, N>& inAAHyperRectangle, const Vec<T, N>& inPoint)
{
  return inPoint >= inAAHyperRectangle.GetMin() && inPoint <= inAAHyperRectangle.GetMax();
}

template <typename TContainer, typename TContainee>
std::enable_if_t<!IsVec_v<TContainee>, bool> Contains(const TContainer& inContainer, const TContainee& inContainee)
{
  return std::all_of(inContainee.cbegin(), inContainee.cend(), [&inContainer](const auto& inContaineeElement) {
    return Contains(inContainer, inContaineeElement);
  });
}

}