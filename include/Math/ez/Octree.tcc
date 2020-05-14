#include "Math.h"
#include "Octree.h"
#include <algorithm>

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
std::vector<typename Octree<TPrimitive>::ValueType> Octree<TPrimitive>::IntersectAll(const Ray3<ValueType>& inRay) const
{
  std::vector<ValueType> intersections;
  IntersectAllRecursive(inRay, intersections);
  return intersections;
}

template <typename TPrimitive>
void Octree<TPrimitive>::IntersectAllRecursive(const Ray3<ValueType>& inRay,
    std::vector<ValueType>& ioIntersections) const
{
  if (IsLeaf())
  {
    for (const auto& primitive : mPrimitives)
    {
      const auto intersection_result = Intersect(primitive, inRay);
      if constexpr (IsArray_v<decltype(intersection_result)>)
      {
        for (const auto& intersection_subresult : intersection_result)
        {
          if (intersection_subresult)
            ioIntersections.push_back(*intersection_subresult);
        }
      }
      else
      {
        if (intersection_result)
          ioIntersections.push_back(*intersection_result);
      }
    }
  }
  else
  {
    const auto aacube_size = mAACube.GetSize();
    const auto aacube_half_size = aacube_size / static_cast<ValueType>(2);

    // Entry intersection point
    std::array<std::optional<ChildSequentialIndex>, 4> child_octree_indices_to_explore_in_order;

    // Determine entry intersection in the octree by looking at external planes
    for (int external_plane_i = 0; external_plane_i < 6; ++external_plane_i)
    {
      const auto& external_plane_normal = OctreePlaneNormals.at(external_plane_i);
      if (Dot(inRay.GetDirection(), external_plane_normal) > 0) // Only consider planes from which it can enter
        continue;

      const auto remapped_external_plane_normal = Max(external_plane_normal, Zero<Vec3<ValueType>>());
      const auto external_plane_point = mAACube.GetMin() + remapped_external_plane_normal * aacube_size;
      const auto external_plane = Plane<ValueType>(external_plane_normal, external_plane_point);
      const auto ray_plane_intersection_distance = Intersect(inRay, external_plane);
      if (!ray_plane_intersection_distance)
        continue;

      const auto octree_plane_id = static_cast<EOctreePlaneId>(external_plane_i);
      const auto first_child_octree_id_to_explore = GetNextChildOctreeIndexToExplore(octree_plane_id,
          inRay.GetDirection(),
          inRay.GetPoint(*ray_plane_intersection_distance));
      if (!first_child_octree_id_to_explore) // We have hit the plane but not the octree, keep looking
        continue;

      child_octree_indices_to_explore_in_order[0] = first_child_octree_id_to_explore;
      break; // We can only enter to the octree from one of its faces
    }

    // Determine which octree children to explore by looking at the intersection with the 3 inner planes
    auto mid_plane_intersection_distances = std::array {
      std::make_tuple(std::optional<ChildSequentialIndex>(), Infinity<ValueType>()), // MID_X
      std::make_tuple(std::optional<ChildSequentialIndex>(), Infinity<ValueType>()), // MID_Y
      std::make_tuple(std::optional<ChildSequentialIndex>(), Infinity<ValueType>()), // MID_Z
    };

    for (int mid_plane_i = 0; mid_plane_i < 3; ++mid_plane_i)
    {
      const auto mid_plane_id = static_cast<EOctreePlaneId>(mid_plane_i + 6);
      const auto& mid_plane_normal = OctreePlaneNormals.at(static_cast<int>(mid_plane_id));
      const auto mid_plane_point = mAACube.GetCenter();
      const auto mid_plane = Plane<ValueType>(mid_plane_normal, mid_plane_point);
      const auto ray_plane_intersection_distance = Intersect(inRay, mid_plane);
      if (!ray_plane_intersection_distance)
        continue;

      const auto next_child_octree_id_to_explore = GetNextChildOctreeIndexToExplore(mid_plane_id,
          inRay.GetDirection(),
          inRay.GetPoint(*ray_plane_intersection_distance));
      if (!next_child_octree_id_to_explore)
        continue;

      mid_plane_intersection_distances[mid_plane_i]
          = std::make_tuple(next_child_octree_id_to_explore, *ray_plane_intersection_distance);
    }

    /*
    std::sort(mid_plane_intersection_distances.begin(),
        mid_plane_intersection_distances.end(),
        [](auto& inLHS, auto& inRHS) { return std::get<1>(inLHS) < std::get<1>(inRHS); });
    */

    child_octree_indices_to_explore_in_order[1] = std::get<0>(mid_plane_intersection_distances[0]);
    child_octree_indices_to_explore_in_order[2] = std::get<0>(mid_plane_intersection_distances[1]);
    child_octree_indices_to_explore_in_order[3] = std::get<0>(mid_plane_intersection_distances[2]);

    // Recursive calls to explore up to 4 children
    for (const auto child_octree_index : child_octree_indices_to_explore_in_order)
    {
      if (!child_octree_index)
        continue; // break;

      const auto child_octree_to_explore = GetChildOctree(*child_octree_index);
      if (!child_octree_to_explore)
        continue;

      child_octree_to_explore->IntersectAllRecursive(inRay, ioIntersections);
    }
  }
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
    const EOctreePlaneId& inOctreePlaneId,
    const Vec3<ValueType>& inRayDirection,
    const Vec3<ValueType>& inIntersectionPoint) const
{
  const auto is_external_plane = IsExternalPlane(inOctreePlaneId);
  const auto plane_i = static_cast<int>(inOctreePlaneId);
  const auto plane_coordinate_i = (is_external_plane ? (plane_i / 2) : (plane_i % 3));
  const auto min_to_point = (inIntersectionPoint - mAACube.GetMin());
  const auto aacube_size = mAACube.GetSize();
  const auto min_to_point_scaled = (min_to_point / aacube_size);
  auto min_to_point_scaled_safe = min_to_point_scaled;
  min_to_point_scaled_safe[plane_coordinate_i] = static_cast<ValueType>(0.5); // To avoid IsBetween deal with epsilons

  if (!IsBetween(min_to_point_scaled_safe, Zero<Vec3<ValueType>>(), One<Vec3<ValueType>>()))
    return std::nullopt; // Intersection point is outside octree cube. No next child octree to explore.

  // Compute binary index of the next child we should explore
  // We assume we are entering (not exiting) the Octree for planes LEFT, RIGHT, BOTTOM, TOP, FRONT and BACK
  auto binary_index = BinaryIndex<3>(Round(min_to_point_scaled_safe));
  if (is_external_plane)
    binary_index[plane_coordinate_i] = (plane_i % 2);
  else
    binary_index[plane_coordinate_i] = (inRayDirection[plane_coordinate_i] < 0 ? 0 : 1);
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
  return BuildRecursive(bounding_aa_cube, inPrimitives, inLeafNodesMaxCapacity, inMaxDepth, 0);
}

template <typename TPrimitive>
Octree<TPrimitive> OctreeBuilder<TPrimitive>::BuildRecursive(
    const typename Octree<TPrimitive>::AACubeType& inAABoundingCube,
    const Span<TPrimitive>& inPrimitives,
    const std::size_t inLeafNodesMaxCapacity,
    const std::size_t inMaxDepth,
    const std::size_t inCurrentDepth)
{
  Octree<TPrimitive> octree;
  octree.mAACube = inAABoundingCube;
  octree.mPrimitives.reserve(inPrimitives.GetNumberOfElements() / 8);
  std::copy_if(inPrimitives.cbegin(),
      inPrimitives.cend(),
      std::back_inserter(octree.mPrimitives),
      [&](const auto& inPrimitive) { return Intersect(inAABoundingCube, inPrimitive); });

  if (octree.mPrimitives.size() > inLeafNodesMaxCapacity && inCurrentDepth < inMaxDepth)
  {
    for (std::size_t i = 0; i < 8; ++i)
    {
      const auto child_bounding_aa_cube = octree.GetChildAACube(i);
      auto built_child = BuildRecursive(child_bounding_aa_cube,
          MakeSpan(octree.mPrimitives),
          inLeafNodesMaxCapacity,
          inMaxDepth,
          inCurrentDepth + 1);

      if (!built_child.IsEmpty())
      {
        octree.mChildren[i] = std::make_unique<Octree<TPrimitive>>(std::move(built_child));
      }
    }
  }

  return octree;
}
}