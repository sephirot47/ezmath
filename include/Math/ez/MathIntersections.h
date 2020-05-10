#pragma once

#include "MathIntersections.h"
#include "ez/AACube.h"
#include "ez/AARect.h"
#include "ez/Geometry.h"
#include "ez/Triangle.h"
#include "ez/Vec.h"
#include <functional>
#include <type_traits>

using namespace std::placeholders;

namespace ez
{
// SAT Normals
template <typename T>
auto GetSATNormals(const AACube<T>&)
{
  return std::array { Right<Vec3<T>>(), Up<Vec3<T>>(), Forward<Vec3<T>>() };
}

template <typename T>
auto GetSATNormals(const AARect<T>&)
{
  return std::array { Right<Vec3<T>>(), Up<Vec3<T>>() };
}

template <typename T>
auto GetSATNormals(const Triangle3<T>& inTriangle)
{
  return std::array { Normal(inTriangle) };
}

template <typename T>
auto GetSATNormals(const Triangle2<T>&)
{
  return std::array<Vec2<T>, 0> {};
}

// SAT Edges
template <typename T>
auto GetSATEdges(const AACube<T>&)
{
  return std::array { Right<Vec3<T>>(), Up<Vec3<T>>(), Forward<Vec3<T>>() };
}

template <typename T>
auto GetSATEdges(const AARect<T>&)
{
  return std::array { Right<Vec2<T>>(), Up<Vec2<T>>() };
}

template <typename T, std::size_t N>
auto GetSATEdges(const Triangle<T, N>& inTriangle)
{
  // TODO: Does it need to be normalized???
  return std::array { NormalizedSafe(inTriangle[0] - inTriangle[1]),
    NormalizedSafe(inTriangle[0] - inTriangle[2]),
    NormalizedSafe(inTriangle[1] - inTriangle[2]) };
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