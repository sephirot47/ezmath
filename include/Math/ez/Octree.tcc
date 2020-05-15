#include "Math.h"
#include "Octree.h"
#include <algorithm>
#include <numeric>

namespace ez
{
template <typename TPrimitive>
Octree<TPrimitive>::Octree(const Span<TPrimitive>& inPrimitives,
    const std::size_t inLeafNodesMaxCapacity,
    const std::size_t inMaxDepth)
{
  *this = OctreeBuilder<TPrimitive>::Build(inPrimitives, inLeafNodesMaxCapacity, inMaxDepth);
}

template <typename TPrimitive>
template <EOctreeIntersectFlags TIntersectionMode>
std::vector<typename Octree<TPrimitive>::Intersection> Octree<TPrimitive>::Intersect(const Ray3<ValueType>& inRay,
    const ValueType inMaxDistance) const
{
  EXPECTS(mPrimitivesPool);

  constexpr auto StopAtFirstIntersection
      = IsFlagOn(TIntersectionMode, EOctreeIntersectFlags::STOP_AT_FIRST_INTERSECTION);
  constexpr auto SortResults = IsFlagOn(TIntersectionMode, EOctreeIntersectFlags::SORT_RESULTS);

  std::vector<Intersection> intersections;
  IntersectRecursive<TIntersectionMode>(inRay, inMaxDistance, *mPrimitivesPool, intersections);

  // Sort if needed
  if constexpr (SortResults)
  {
    std::sort(intersections.begin(), intersections.end(), [](const auto& inLHS, const auto& inRHS) {
      return inLHS.mDistance < inRHS.mDistance;
    });
  }

  if constexpr (StopAtFirstIntersection)
    ENSURES(intersections.size() <= 1);

  return intersections;
}

template <typename TPrimitive>
template <EOctreeIntersectFlags TIntersectionMode>
void Octree<TPrimitive>::IntersectRecursive(const Ray3<ValueType>& inRay,
    const ValueType inMaxDistance,
    const std::vector<TPrimitive>& inPrimitivesPool,
    std::vector<Intersection>& ioIntersections) const
{
  constexpr auto StopAtFirstIntersection
      = IsFlagOn(TIntersectionMode, EOctreeIntersectFlags::STOP_AT_FIRST_INTERSECTION);

  const auto aacube_size = mAACube.GetSize();
  const auto aacube_half_size = (aacube_size / static_cast<ValueType>(2));

  // If STOP_AT_FIRST_INTERSECTION, check whether this AACube needs to be checked because of max distance or not
  if constexpr (StopAtFirstIntersection)
  {
    const auto max_cube_half_size = Max(aacube_half_size);
    const auto max_distance_and_max_cube_half_size = inMaxDistance + max_cube_half_size;
    const auto sq_max_distance_and_max_cube_half_size
        = max_distance_and_max_cube_half_size * max_distance_and_max_cube_half_size;
    const auto sq_dist_ray_origin_to_aacube_center = SqDistance(inRay.GetOrigin(), mAACube.GetCenter());
    if (sq_dist_ray_origin_to_aacube_center > sq_max_distance_and_max_cube_half_size)
      return;
  }

  if (IsLeaf())
  {
    // Base case, linear search through its contained primitives
    for (const auto& primitive_index : mPrimitivesIndices)
    {
      const auto& primitive = inPrimitivesPool.at(primitive_index);
      const auto PushResultIfNeeded = [&](const auto& inIntersectionResultDistance) {
        if (inIntersectionResultDistance && *inIntersectionResultDistance < inMaxDistance)
        {
          Intersection intersection;
          intersection.mDistance = *inIntersectionResultDistance;
          intersection.mPrimitiveIndex = primitive_index;
          ioIntersections.push_back(std::move(intersection));
        }
      };

      const auto intersection_result = ::ez::Intersect(inRay, primitive);
      if constexpr (IsArray_v<decltype(intersection_result)>)
        std::for_each(intersection_result.begin(), intersection_result.end(), PushResultIfNeeded);
      else
        PushResultIfNeeded(intersection_result);

      if constexpr (StopAtFirstIntersection)
      {
        if (ioIntersections.size() == 1)
          break;
      }
    }
  }
  else
  {
    // Recursive case, check which children octrees the ray intersects
    std::array<std::optional<ChildSequentialIndex>, 4> child_octree_indices_to_explore;

    // Determine entry intersection in the octree by looking at external planes.
    // This will give us the first child octree to explore.
    for (int external_plane_i = 0; external_plane_i < 6; ++external_plane_i)
    {
      const auto& external_plane_normal = ExternalOctreePlaneNormals.at(external_plane_i);
      if (Dot(inRay.GetDirection(), external_plane_normal) > 0) // Only consider planes from which it can enter
        continue;

      const auto remapped_external_plane_normal = Max(external_plane_normal, Zero<Vec3<ValueType>>());
      const auto external_plane_point = mAACube.GetMin() + remapped_external_plane_normal * aacube_size;
      const auto external_plane = Plane<ValueType>(external_plane_normal, external_plane_point);
      const auto ray_plane_intersection_distance = ::ez::Intersect(inRay, external_plane);
      if (!ray_plane_intersection_distance)
        continue;

      const auto external_plane_id = static_cast<EExternalOctreePlaneId>(external_plane_i);
      const auto first_child_octree_id_to_explore
          = GetNextChildOctreeIndexToExplore(external_plane_id, inRay.GetPoint(*ray_plane_intersection_distance));
      if (!first_child_octree_id_to_explore) // We have hit the plane but not the octree, keep looking
        continue;

      child_octree_indices_to_explore[0] = first_child_octree_id_to_explore;
      break; // We can only enter to the octree from one of its faces, so as soon as we find it, break
    }

    // Determine which octree children to explore by looking at the intersection with the 3 internal planes
    auto internal_plane_intersection_distances = std::array {
      std::make_tuple(std::optional<ChildSequentialIndex>(), Infinity<ValueType>()), // MID_X
      std::make_tuple(std::optional<ChildSequentialIndex>(), Infinity<ValueType>()), // MID_Y
      std::make_tuple(std::optional<ChildSequentialIndex>(), Infinity<ValueType>()), // MID_Z
    };

    for (int internal_plane_i = 0; internal_plane_i < 3; ++internal_plane_i)
    {
      const auto& internal_plane_normal = InternalOctreePlaneNormals.at(internal_plane_i);
      const auto internal_plane_point = mAACube.GetCenter();
      const auto internal_plane = Plane<ValueType>(internal_plane_normal, internal_plane_point);
      const auto ray_plane_intersection_distance = ::ez::Intersect(inRay, internal_plane);
      if (!ray_plane_intersection_distance)
        continue;

      const auto internal_plane_id = static_cast<EInternalOctreePlaneId>(internal_plane_i);
      const auto next_child_octree_id_to_explore = GetNextChildOctreeIndexToExplore(internal_plane_id,
          inRay.GetDirection(),
          inRay.GetPoint(*ray_plane_intersection_distance));
      if (!next_child_octree_id_to_explore)
        continue;

      internal_plane_intersection_distances[internal_plane_i]
          = std::make_tuple(next_child_octree_id_to_explore, *ray_plane_intersection_distance);
    }

    // If we want to stop at first hit, order child octrees based on intersection distance
    // so that we explore them in order and we can early out asap
    if constexpr (StopAtFirstIntersection)
    {
      std::sort(internal_plane_intersection_distances.begin(),
          internal_plane_intersection_distances.end(),
          [](auto& inLHS, auto& inRHS) { return std::get<1>(inLHS) < std::get<1>(inRHS); });
    }

    child_octree_indices_to_explore[1] = std::get<0>(internal_plane_intersection_distances[0]);
    child_octree_indices_to_explore[2] = std::get<0>(internal_plane_intersection_distances[1]);
    child_octree_indices_to_explore[3] = std::get<0>(internal_plane_intersection_distances[2]);

    // Recursive calls to explore up to 4 children
    for (const auto child_octree_index : child_octree_indices_to_explore)
    {
      if (!child_octree_index)
        continue;

      const auto child_octree_to_explore = GetChildOctree(*child_octree_index);
      if (!child_octree_to_explore)
        continue;

      child_octree_to_explore->template IntersectRecursive<TIntersectionMode>(inRay,
          inMaxDistance,
          inPrimitivesPool,
          ioIntersections);
      if constexpr (StopAtFirstIntersection)
      {
        if (ioIntersections.size() == 1)
          break;
      }
    }
  }
}

template <typename TPrimitive>
const std::vector<TPrimitive>& Octree<TPrimitive>::GetPrimitivesPool() const
{
  EXPECTS(mPrimitivesPool);
  return *mPrimitivesPool;
}

template <typename TPrimitive>
Octree<TPrimitive>::AACubeType Octree<TPrimitive>::GetChildAACube(const Octree::ChildMultiIndex01 inChildIndex) const
{
  const auto child_aacube_size = (mAACube.GetSize() / static_cast<ValueType>(2));
  const auto child_aacube_size_indexed = (child_aacube_size * Vec3<ValueType>(inChildIndex));
  const auto child_aacube_min = mAACube.GetMin() + child_aacube_size_indexed;
  const auto child_aacube_max = child_aacube_min + child_aacube_size;
  return AACube<ValueType>(child_aacube_min, child_aacube_max);
}

template <typename TPrimitive>
Octree<TPrimitive>::AACubeType Octree<TPrimitive>::GetChildAACube(
    const typename Octree<TPrimitive>::ChildSequentialIndex inInternalIndex) const
{
  return GetChildAACube(MakeBinaryIndex<3>(inInternalIndex));
}

template <typename TPrimitive>
Octree<TPrimitive>* Octree<TPrimitive>::GetChildOctree(const Octree::ChildMultiIndex01 inChildIndex)
{
  return mChildren[MakeSequentialIndex(inChildIndex)];
}

template <typename TPrimitive>
const Octree<TPrimitive>* Octree<TPrimitive>::GetChildOctree(const Octree::ChildMultiIndex01 inChildIndex) const
{
  return const_cast<Octree&>(*this).GetChildOctree(inChildIndex);
}

template <typename TPrimitive>
Octree<TPrimitive>* Octree<TPrimitive>::GetChildOctree(
    const typename Octree<TPrimitive>::ChildSequentialIndex inInternalIndex)
{
  EXPECTS(inInternalIndex < mChildren.size());
  return mChildren.at(inInternalIndex).get();
}

template <typename TPrimitive>
const Octree<TPrimitive>* Octree<TPrimitive>::GetChildOctree(
    const Octree<TPrimitive>::ChildSequentialIndex inInternalIndex) const
{
  return const_cast<Octree&>(*this).GetChildOctree(inInternalIndex);
}

template <typename TPrimitive>
bool Octree<TPrimitive>::IsLeaf() const
{
  return std::all_of(mChildren.cbegin(), mChildren.cend(), [](const auto& child) { return child == nullptr; });
}

template <typename TPrimitive>
template <bool IsConst>
Octree<TPrimitive>::GIterator<IsConst>::GIterator(OctreeType& ioOctree, const ChildSequentialIndex inBeginIndex)
    : mOctree { ioOctree }, mCurrentIndex(inBeginIndex)
{
  if (mCurrentIndex < 8 && !mOctree.GetChildOctree(mCurrentIndex)) // Advance to next valid index (end() if there isnt)
    ++(*this);
}

template <typename TPrimitive>
template <bool IsConst>
typename Octree<TPrimitive>::template GIterator<IsConst>& Octree<TPrimitive>::GIterator<IsConst>::operator++()
{
  EXPECTS(mCurrentIndex < 8);

  while (true)
  {
    ++mCurrentIndex;

    if (mCurrentIndex == 8)
      break;

    if (mOctree.GetChildOctree(mCurrentIndex) != nullptr)
      break;
  }

  ENSURES(mCurrentIndex <= 8);

  return *this;
}

template <typename TPrimitive>
std::optional<typename Octree<TPrimitive>::ChildSequentialIndex> Octree<TPrimitive>::GetNextChildOctreeIndexToExplore(
    const EExternalOctreePlaneId& inExternalOctreePlaneId,
    const Vec3<ValueType>& inIntersectionPoint) const
{
  // For external planes, we assume we are entering (not exiting) the Octree
  const auto plane_i = static_cast<int>(inExternalOctreePlaneId);
  const auto plane_coordinate_i = (plane_i / 2);
  const auto min_to_point_scaled = (inIntersectionPoint - mAACube.GetMin()) / mAACube.GetSize();
  auto min_to_point_scaled_safe = min_to_point_scaled;
  min_to_point_scaled_safe[plane_coordinate_i] = static_cast<ValueType>(0.5); // To avoid IsBetween deal with epsilons

  if (!IsBetween(min_to_point_scaled_safe, Zero<Vec3<ValueType>>(), One<Vec3<ValueType>>()))
    return std::nullopt; // Intersection point is outside octree cube. No next child octree to explore.

  auto binary_index = BinaryIndex<3>(Round(min_to_point_scaled_safe));
  binary_index[plane_coordinate_i] = (plane_i % 2);
  return MakeSequentialIndex(binary_index);
}

template <typename TPrimitive>
std::optional<typename Octree<TPrimitive>::ChildSequentialIndex> Octree<TPrimitive>::GetNextChildOctreeIndexToExplore(
    const EInternalOctreePlaneId& inInternalOctreePlaneId,
    const Vec3<ValueType>& inRayDirection,
    const Vec3<ValueType>& inIntersectionPoint) const
{
  const auto plane_i = static_cast<int>(inInternalOctreePlaneId);
  const auto min_to_point_scaled = (inIntersectionPoint - mAACube.GetMin()) / mAACube.GetSize();
  auto min_to_point_scaled_safe = min_to_point_scaled;
  min_to_point_scaled_safe[plane_i] = static_cast<ValueType>(0.5); // To avoid IsBetween deal with epsilons

  if (!IsBetween(min_to_point_scaled_safe, Zero<Vec3<ValueType>>(), One<Vec3<ValueType>>()))
    return std::nullopt; // Intersection point is outside octree cube. No next child octree to explore.

  auto binary_index = BinaryIndex<3>(Round(min_to_point_scaled_safe));
  binary_index[plane_i] = (inRayDirection[plane_i] < 0 ? 0 : 1);
  return MakeSequentialIndex(binary_index);
}

template <typename TPrimitive>
template <bool IsConst>
typename Octree<TPrimitive>::template GIterator<IsConst>::OctreeType&
    Octree<TPrimitive>::GIterator<IsConst>::operator*()
{
  const auto current_octree = mOctree.GetChildOctree(mCurrentIndex);
  EXPECTS(current_octree);
  return *current_octree;
}

template <typename TPrimitive>
Octree<TPrimitive> OctreeBuilder<TPrimitive>::Build(const Span<TPrimitive>& inPrimitives,
    const std::size_t inLeafNodesMaxCapacity,
    const std::size_t inMaxDepth)
{
  const auto bounding_aa_cube = BoundingAACube(inPrimitives);
  return BuildRecursive(bounding_aa_cube,
      inPrimitives,
      MakeSpan<typename Octree<TPrimitive>::PrimitiveIndex>({}),
      inLeafNodesMaxCapacity,
      inMaxDepth,
      0);
}

template <typename TPrimitive>
Octree<TPrimitive> OctreeBuilder<TPrimitive>::BuildRecursive(
    const typename Octree<TPrimitive>::AACubeType& inBoundingAACube,
    const Span<TPrimitive>& inPrimitivesPool,
    const Span<typename Octree<TPrimitive>::PrimitiveIndex>& inParentPrimitivesIndices,
    const std::size_t inLeafNodesMaxCapacity,
    const std::size_t inMaxDepth,
    const std::size_t inCurrentDepth)
{
  if (inCurrentDepth > inMaxDepth)
    return {};

  // Create octree
  Octree<TPrimitive> octree;
  octree.mAACube = inBoundingAACube;

  if (inCurrentDepth == 0)
  {
    // Octree global primitives pool
    octree.mPrimitivesPool = std::make_optional<std::vector<TPrimitive>>();
    octree.mPrimitivesPool->reserve(inPrimitivesPool.GetNumberOfElements());
    std::copy(inPrimitivesPool.cbegin(), inPrimitivesPool.cend(), std::back_inserter(*octree.mPrimitivesPool));

    // Octree primitives indices 0,1,2,...,N
    octree.mPrimitivesIndices.resize(octree.mPrimitivesPool->size());
    std::iota(octree.mPrimitivesIndices.begin(), octree.mPrimitivesIndices.end(), 0);
  }
  else
  {
    // Copy only primitive indices that intersect this Octree bounding cube
    octree.mPrimitivesIndices.reserve(inParentPrimitivesIndices.GetNumberOfElements() / 8);
    std::copy_if(inParentPrimitivesIndices.cbegin(),
        inParentPrimitivesIndices.cend(),
        std::back_inserter(octree.mPrimitivesIndices),
        [&](const auto& inPrimitiveIndex) {
          return Intersect(inBoundingAACube, inPrimitivesPool.at(inPrimitiveIndex));
        });
  }

  if (octree.mPrimitivesIndices.size() <= inLeafNodesMaxCapacity)
    return octree;

  for (std::size_t i = 0; i < 8; ++i)
  {
    const auto child_bounding_aa_cube = octree.GetChildAACube(i);
    auto built_child = BuildRecursive(child_bounding_aa_cube,
        inPrimitivesPool,
        MakeSpan(octree.mPrimitivesIndices),
        inLeafNodesMaxCapacity,
        inMaxDepth,
        inCurrentDepth + 1);

    if (!built_child.IsEmpty())
      octree.mChildren[i] = std::make_unique<Octree<TPrimitive>>(std::move(built_child));
  }

  return octree;
}
}