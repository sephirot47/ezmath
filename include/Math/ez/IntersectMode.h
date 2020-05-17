#pragma once

#include "ez/Flags.h"

namespace ez
{
enum class EIntersectMode
{
  ALL_INTERSECTIONS, // Get all intersections. Returns an std::array or std::vector of the intersections.
  ONLY_CLOSEST,      // Stop at the first intersection. Returns an std::optional of the intersection.
  ONLY_CHECK,        // Only returns a boolean telling whether they intersect or not.
};
DECLARE_FLAGS(EIntersectMode);
}