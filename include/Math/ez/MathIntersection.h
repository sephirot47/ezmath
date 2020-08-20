#pragma once

#include <ez/IntersectMode.h>

namespace ez
{
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
