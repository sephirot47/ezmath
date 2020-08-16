#pragma once

#include "MathIntersections.h"
#include "ez/IntersectMode.h"
#include "ez/Vec.h"
#include <functional>
#include <type_traits>

using namespace std::placeholders;

namespace ez
{
// Convex intersection functions (SAT)
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

template <EIntersectMode TIntersectMode, typename TLHSConvexObject, typename TRHSConvexObject>
bool IntersectCheckSAT(const TLHSConvexObject& inLHS, const TRHSConvexObject& inRHS)
{
  static_assert(TIntersectMode == EIntersectMode::ONLY_CHECK, "Unsupported EIntersectMode");

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

template <EIntersectMode TIntersectMode, typename T, std::size_t N, typename TRHSConvexObject>
auto Intersect(const Vec<T, N>& inPoint, const TRHSConvexObject& inRHS)
{
  static_assert(TIntersectMode == EIntersectMode::ONLY_CHECK, "Unsupported EIntersectMode");
  return Contains(inRHS, inPoint);
}

template <EIntersectMode TIntersectMode, typename T, std::size_t N, typename TLHSConvexObject>
auto Intersect(const TLHSConvexObject& inLHS, const Vec<T, N>& inPoint)
{
  static_assert(TIntersectMode == EIntersectMode::ONLY_CHECK, "Unsupported EIntersectMode");
  return Intersect<TIntersectMode>(inPoint, inLHS);
}

// Generic contains
template <typename TContainer, typename TContainee>
std::enable_if_t<!IsVec_v<TContainee>, bool> Contains(const TContainer& inContainer, const TContainee& inContainee)
{
  return std::all_of(inContainee.cbegin(), inContainee.cend(), [&inContainer](const auto& inContaineeElement) {
    return Contains(inContainer, inContaineeElement);
  });
}

// Intersect aliases
template <typename TLHS, typename TRHS>
auto IntersectAll(const TLHS& inLHS, const TRHS& inRHS)
{
  return Intersect<EIntersectMode::ALL_INTERSECTIONS>(inLHS, inRHS);
}

template <typename TLHS, typename TRHS>
auto IntersectClosest(const TLHS& inLHS, const TRHS& inRHS)
{
  return Intersect<EIntersectMode::ONLY_CLOSEST>(inLHS, inRHS);
}

template <typename TLHS, typename TRHS>
bool IntersectCheck(const TLHS& inLHS, const TRHS& inRHS)
{
  return Intersect<EIntersectMode::ONLY_CHECK>(inLHS, inRHS);
}

}