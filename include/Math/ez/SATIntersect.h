#pragma once

#include "ez/IntersectMode.h"
#include "ez/Vec.h"
#include <functional>
#include <type_traits>

using namespace std::placeholders;

namespace ez
{
// Convex intersection functions (using SAT)
template <typename TLHSPoints, typename TRHSPoints, typename T, std::size_t N>
std::array<std::array<T, 2>, 2>
GetProjectionsIntervals(const TLHSPoints& inLHSPoints, const TRHSPoints& inRHSPoints, const Vec<T, N>& inProjectionAxis)
{
  auto min_lhs_projection = Max<T>();
  auto max_lhs_projection = Min<T>();
  for (const auto& lhs_point : inLHSPoints)
  {
    const auto lhs_projection = Dot(lhs_point, inProjectionAxis);
    min_lhs_projection = Min(min_lhs_projection, lhs_projection);
    max_lhs_projection = Max(max_lhs_projection, lhs_projection);
  }

  auto min_rhs_projection = Max<T>();
  auto max_rhs_projection = Min<T>();
  for (const auto& rhs_point : inRHSPoints)
  {
    const auto rhs_projection = Dot(rhs_point, inProjectionAxis);
    min_rhs_projection = Min(min_rhs_projection, rhs_projection);
    max_rhs_projection = Max(max_rhs_projection, rhs_projection);
  }

  return std::array { std::array { min_lhs_projection, max_lhs_projection },
    std::array { min_rhs_projection, max_rhs_projection } };
}

template <typename TLHSPoints, typename TRHSPoints, typename T, std::size_t N>
bool DoProjectionsOverlap(const TLHSPoints& inLHSPoints,
    const TRHSPoints& inRHSPoints,
    const Vec<T, N>& inProjectionAxis)
{
  const auto projections_intervals = GetProjectionsIntervals(inLHSPoints, inRHSPoints, inProjectionAxis);

  const auto& min_lhs_projection = projections_intervals[0][0];
  const auto& max_lhs_projection = projections_intervals[0][1];
  const auto& min_rhs_projection = projections_intervals[1][0];
  const auto& max_rhs_projection = projections_intervals[1][1];

  const auto there_is_no_projection_overlap
      = (min_lhs_projection > max_rhs_projection) || (min_rhs_projection > max_lhs_projection);
  const auto there_is_projection_overlap = !there_is_no_projection_overlap;
  return there_is_projection_overlap;
}

template <typename TLHSPrimitive, typename TRHSPrimitive>
auto IntersectMaxDistanceSAT(const TLHSPrimitive& inLHSPrimitive, const TRHSPrimitive& inRHSPrimitive)
{
  using VecType = decltype(GetSATPoints(std::declval<TLHSPrimitive>()))::value_type;
  using T = ValueType_t<VecType>;
  static constexpr auto N = NumComponents_v<VecType>;

  const auto lhs_points = GetSATPoints(inLHSPrimitive);
  const auto rhs_points = GetSATPoints(inRHSPrimitive);
  const auto GetAxisMinDistance = [&](const Vec<T, N>& in_projection_axis) {
    const auto projections_intervals = GetProjectionsIntervals(lhs_points, rhs_points, in_projection_axis);
    const auto& min_lhs_projection = projections_intervals[0][0];
    const auto& max_lhs_projection = projections_intervals[0][1];
    const auto& min_rhs_projection = projections_intervals[1][0];
    const auto& max_rhs_projection = projections_intervals[1][1];

    const auto there_is_no_projection_overlap
        = (min_lhs_projection > max_rhs_projection) || (min_rhs_projection > max_lhs_projection);
    const auto there_is_projection_overlap = !there_is_no_projection_overlap;
    if (there_is_projection_overlap)
      return static_cast<T>(0);

    const auto min_distance
        = Min(Abs(max_rhs_projection - min_lhs_projection), Abs(max_lhs_projection - min_rhs_projection));
    return min_distance;
  };

  auto max_distance = Min<T>();
  {
    // LHS normals
    const auto lhs_normals = GetSATNormals(inLHSPrimitive);
    for (const auto& lhs_normal : lhs_normals)
    { max_distance = Max(GetAxisMinDistance(NormalizedSafe(lhs_normal)), max_distance); }

    // RHS normals
    const auto rhs_normals = GetSATNormals(inRHSPrimitive);
    for (const auto& rhs_normal : rhs_normals)
    { max_distance = Max(GetAxisMinDistance(NormalizedSafe(rhs_normal)), max_distance); }

    if constexpr (N == 3)
    {
      // Test combined axes (LHS-to-RHS cross product of edges)
      const auto lhs_edges = GetSATEdges(inLHSPrimitive);
      const auto rhs_edges = GetSATEdges(inRHSPrimitive);
      for (const auto& lhs_edge : lhs_edges)
      {
        for (const auto& rhs_edge : rhs_edges)
        {
          const auto cross_axis = Cross(lhs_edge, rhs_edge);
          max_distance = Max(GetAxisMinDistance(cross_axis), max_distance);
        }
      }
    }
  }
  return max_distance;
}

template <typename TLHSPrimitive, typename TRHSPrimitive>
bool IntersectCheckSAT(const TLHSPrimitive& inLHSPrimitive, const TRHSPrimitive& inRHSPrimitive)
{
  using VecType = decltype(GetSATPoints(std::declval<TLHSPrimitive>()))::value_type;
  static constexpr auto N = NumComponents_v<VecType>;

  const auto lhs_points = GetSATPoints(inLHSPrimitive);
  const auto rhs_points = GetSATPoints(inRHSPrimitive);
  const auto AxisSeparatesObjects
      = [&](const auto& inAxis) { return !DoProjectionsOverlap(lhs_points, rhs_points, inAxis); };

  // Test LHS normals
  const auto lhs_normals = GetSATNormals(inLHSPrimitive);
  if (std::any_of(lhs_normals.cbegin(), lhs_normals.cend(), AxisSeparatesObjects))
    return false;

  // Test RHS normals
  const auto rhs_normals = GetSATNormals(inRHSPrimitive);
  if (std::any_of(rhs_normals.cbegin(), rhs_normals.cend(), AxisSeparatesObjects))
    return false;

  // Test combined axes (LHS-to-RHS cross product of edges)
  if constexpr (N == 3)
  {
    const auto lhs_edges = GetSATEdges(inLHSPrimitive);
    const auto rhs_edges = GetSATEdges(inRHSPrimitive);
    for (const auto& lhs_edge : lhs_edges)
    {
      for (const auto& rhs_edge : rhs_edges)
      {
        const auto cross_axis = Cross(lhs_edge, rhs_edge);
        if (AxisSeparatesObjects(cross_axis))
          return false;
      }
    }
  }
  return true;
}
}