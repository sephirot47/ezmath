#include "ez/HyperSphere.h"
#include "ez/Math.h"
#include "ez/Octree.h"
#include "ez/Plane.h"
#include <algorithm>
#include <numeric>
#include <stack>
#include <tuple>
#include <utility>

namespace ez
{

template <typename TPrimitive>
Octree<TPrimitive>::Octree(const AABoxf& inAABox)
{
  mAABox = inAABox;
  mPrimitivesPool = std::make_optional<std::vector<TPrimitive>>();
}

template <typename TPrimitive>
Octree<TPrimitive>::Octree(const Span<TPrimitive>& inPrimitives,
    const std::size_t inLeafNodesMaxCapacity,
    const std::size_t inMaxDepth)
{
  *this = OctreeBuilder<TPrimitive>::Build(inPrimitives, inLeafNodesMaxCapacity, inMaxDepth);
}

template <typename TPrimitive>
const std::vector<TPrimitive>& Octree<TPrimitive>::GetPrimitivesPool() const
{
  EXPECTS(mPrimitivesPool);
  return *mPrimitivesPool;
}

template <typename TPrimitive>
Octree<TPrimitive>::AABoxType Octree<TPrimitive>::GetChildAABox(
    const typename Octree<TPrimitive>::ChildSequentialIndex inInternalIndex) const
{
  using ValueType = ValueType_t<TPrimitive>;
  const auto child_multi_index = MakeBinaryIndex<3, ValueType>(inInternalIndex);
  const auto child_aabox_size = (mAABox.GetSize() / static_cast<ValueType>(2));
  const auto child_aabox_size_indexed = (child_aabox_size * child_multi_index);
  const auto child_aabox_min = mAABox.GetMin() + child_aabox_size_indexed;
  const auto child_aabox_max = child_aabox_min + child_aabox_size;
  return AABox<ValueType>(child_aabox_min, child_aabox_max);
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
bool Octree<TPrimitive>::AddPrimitive(const TPrimitive& inPrimitive,
    const std::size_t inLeafNodesMaxCapacity,
    const std::size_t inMaxDepth)
{
  // Adapt octree size if needed(and children's size as well)
  if (!Contains(mAABox, inPrimitive))
  {
    mAABox.Wrap(BoundingAABox(inPrimitive));

    std::stack<std::tuple<Octree*, Octree*, std::size_t>> octree_stack;
    octree_stack.emplace(this, nullptr, 0ul);
    while (!octree_stack.empty())
    {
      const auto& octree_stack_top = octree_stack.top();
      const auto octree = std::get<0>(octree_stack_top);
      const auto parent_octree = std::get<1>(octree_stack_top);
      const auto octree_index_in_parent = std::get<2>(octree_stack_top);
      octree_stack.pop();

      for (std::size_t i = 0; i < 8; ++i) // Enqueue children if any
      {
        if (const auto octree_child = octree->GetChildOctree(i))
          octree_stack.emplace(octree_child, octree, i);
      }

      if (!parent_octree)
        continue;

      octree->mAABox = parent_octree->GetChildAABox(octree_index_in_parent); // Change size
    }
  }

  const auto old_primitives_pool_size = mPrimitivesPool->size();
  const auto new_primitive_index_if_added = mPrimitivesPool->size();
  const auto new_primitive_added = AddPrimitiveRecursive(inPrimitive,
      inLeafNodesMaxCapacity,
      inMaxDepth,
      0,
      new_primitive_index_if_added,
      *mPrimitivesPool,
      true);

  if (new_primitive_added)
    assert(mPrimitivesPool->size() == old_primitives_pool_size + 1);
  else
    assert(mPrimitivesPool->size() == old_primitives_pool_size);

  return new_primitive_added;
}

template <typename TPrimitive>
bool Octree<TPrimitive>::AddPrimitiveRecursive(const TPrimitive& inPrimitive,
    const std::size_t inLeafNodesMaxCapacity,
    const std::size_t inMaxDepth,
    const std::size_t inCurrentDepth,
    const std::size_t inNewPrimitiveIndexIfAdded,
    std::vector<TPrimitive>& ioPrimitivesPool,
    const bool inAddPrimitiveToPool)
{
  if (inCurrentDepth > inMaxDepth)
    return false;

  if (!IntersectCheck(inPrimitive, mAABox))
    return false;

  if (mPrimitivesIndices.size() < inLeafNodesMaxCapacity)
  {
    mPrimitivesIndices.push_back(inNewPrimitiveIndexIfAdded);
    if (inAddPrimitiveToPool)
      ioPrimitivesPool.push_back(inPrimitive);
    return true;
  }

  // Not enough room in this level. We have to go one level deeper.
  const auto AddPrimitiveToChildren
      = [&](const TPrimitive& inPrimitive, const std::size_t inPrimitiveIndex, const bool inAddPrimitiveToPool) {
          auto added_to_some_child = false;
          for (std::size_t i = 0; i < 8; ++i)
          {
            const auto child_octree_aabox = GetChildAABox(i);
            if (!IntersectCheck(inPrimitive, child_octree_aabox))
              continue;

            auto child_octree = GetChildOctree(i);
            if (!child_octree)
            {
              mChildren.at(i) = std::move(std::make_unique<Octree>(child_octree_aabox));
              child_octree = mChildren.at(i).get();
            }

            const auto add_primitive_to_pool = (inAddPrimitiveToPool && !added_to_some_child);
            const auto added_to_child = child_octree->AddPrimitiveRecursive(inPrimitive,
                inLeafNodesMaxCapacity,
                inMaxDepth,
                inCurrentDepth + 1,
                inPrimitiveIndex,
                ioPrimitivesPool,
                add_primitive_to_pool);

            added_to_some_child |= added_to_child;
          }
          return added_to_some_child;
        };

  // Save a copy of the primitives indices of this node before to reallocate them afterwards
  const auto old_primitive_indices_copy_to_be_reallocated_in_children = mPrimitivesIndices;
  mPrimitivesIndices.clear(); // Empty this node because it will no longer be a leaft

  // Also, this node will stop being a leaf, so readd its indices in its children.
  for (const auto& old_primitive_index_to_be_reallocated_in_children :
      old_primitive_indices_copy_to_be_reallocated_in_children)
  {
    const auto& old_primitive_to_be_reallocated_in_children
        = ioPrimitivesPool.at(old_primitive_index_to_be_reallocated_in_children);
    AddPrimitiveToChildren(old_primitive_to_be_reallocated_in_children,
        old_primitive_index_to_be_reallocated_in_children,
        false);
  }

  // Add input primitive
  const auto input_primitive_added_to_some_child
      = AddPrimitiveToChildren(inPrimitive, inNewPrimitiveIndexIfAdded, inAddPrimitiveToPool);

  return input_primitive_added_to_some_child;
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
  const auto min_to_point_scaled = (inIntersectionPoint - mAABox.GetMin()) / mAABox.GetSize();
  auto min_to_point_scaled_safe = min_to_point_scaled;
  min_to_point_scaled_safe[plane_coordinate_i] = static_cast<ValueType>(0.5); // To avoid IsBetween deal with epsilons

  if (!IsBetween(min_to_point_scaled_safe, Zero<Vec3<ValueType>>(), One<Vec3<ValueType>>()))
    return std::nullopt; // Intersection point is outside octree box. No next child octree to explore.

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
  const auto min_to_point_scaled = (inIntersectionPoint - mAABox.GetMin()) / mAABox.GetSize();
  auto min_to_point_scaled_safe = min_to_point_scaled;
  min_to_point_scaled_safe[plane_i] = static_cast<ValueType>(0.5); // To avoid IsBetween deal with epsilons

  if (!IsBetween(min_to_point_scaled_safe, Zero<Vec3<ValueType>>(), One<Vec3<ValueType>>()))
    return std::nullopt; // Intersection point is outside octree box. No next child octree to explore.

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
  const auto bounding_aa_box = BoundingAABox(inPrimitives);
  return BuildRecursive(bounding_aa_box,
      inPrimitives,
      MakeSpan<typename Octree<TPrimitive>::PrimitiveIndex>({}),
      inLeafNodesMaxCapacity,
      inMaxDepth,
      0);
}

template <typename TPrimitive>
Octree<TPrimitive> OctreeBuilder<TPrimitive>::BuildRecursive(
    const typename Octree<TPrimitive>::AABoxType& inBoundingAABox,
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
  octree.mAABox = inBoundingAABox;

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
    // Copy only primitive indices that intersect this Octree bounding box
    octree.mPrimitivesIndices.reserve(inParentPrimitivesIndices.GetNumberOfElements() / 8);
    std::copy_if(inParentPrimitivesIndices.cbegin(),
        inParentPrimitivesIndices.cend(),
        std::back_inserter(octree.mPrimitivesIndices),
        [&](const auto& inPrimitiveIndex) {
          const auto& inPrimitive = inPrimitivesPool.at(inPrimitiveIndex);
          return IntersectCheck(inBoundingAABox, inPrimitive);
        });
  }

  if (octree.mPrimitivesIndices.size() <= inLeafNodesMaxCapacity)
    return octree;

  for (std::size_t i = 0; i < 8; ++i)
  {
    const auto child_bounding_aa_box = octree.GetChildAABox(i);
    auto built_child = BuildRecursive(child_bounding_aa_box,
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

template <typename TPrimitive>
struct IntersectHelperStruct final
{
  using ValueType = ValueType_t<TPrimitive>;
  const Ray3<ValueType> mRay;
  const ValueType mMaxDistance;

  IntersectHelperStruct(const Ray3<ValueType>& inRay, const ValueType& inMaxDistance)
      : mRay { inRay }, mMaxDistance { inMaxDistance }
  {
  }

  template <EIntersectMode TIntersectMode>
  auto Intersect(const Octree<TPrimitive>& inTopOctree)
  {
    static_assert(TIntersectMode == EIntersectMode::ALL_INTERSECTIONS || TIntersectMode == EIntersectMode::ONLY_CLOSEST
            || TIntersectMode == EIntersectMode::ONLY_CHECK,
        "Unsupported EIntersectMode");
    EXPECTS(inTopOctree.mPrimitivesPool);

    using IntersectionType = typename Octree<TPrimitive>::Intersection;
    std::vector<IntersectionType> intersections;

    if constexpr (TIntersectMode == EIntersectMode::ONLY_CHECK)
    {
      return IntersectRecursive<TIntersectMode>(inTopOctree, *inTopOctree.mPrimitivesPool, intersections);
    }
    else
    {
      IntersectRecursive<TIntersectMode>(inTopOctree, *inTopOctree.mPrimitivesPool, intersections);

      if constexpr (TIntersectMode == EIntersectMode::ALL_INTERSECTIONS)
      {
        return intersections;
      }
      else if constexpr (TIntersectMode == EIntersectMode::ONLY_CLOSEST)
      {
        assert(intersections.size() <= 1);
        return intersections.size() == 1 ? std::make_optional(intersections[0]) : std::nullopt;
      }
    }
  }

  template <EIntersectMode TIntersectMode, typename TIntersectionDistances>
  auto TreatIntersectionResult(const Octree<TPrimitive>& inOctree,
      const Octree<TPrimitive>::PrimitiveIndex inPrimitiveIndex,
      const TIntersectionDistances& inIntersectionDistances,
      std::vector<typename Octree<TPrimitive>::Intersection>& ioIntersections)
  {
    assert(TIntersectMode == EIntersectMode::ALL_INTERSECTIONS || TIntersectMode == EIntersectMode::ONLY_CLOSEST);

    if constexpr (IsArray_v<TIntersectionDistances>)
    {
      for (const auto& intersection_distance : inIntersectionDistances)
      { TreatIntersectionResult<TIntersectMode>(inOctree, inPrimitiveIndex, intersection_distance, ioIntersections); }
    }
    else
    {
      const auto& inIntersectionDistance = inIntersectionDistances;
      if (!inIntersectionDistance || *inIntersectionDistance > mMaxDistance)
        return; // Do not consider intersections further than the maximum distance

      if constexpr (TIntersectMode == EIntersectMode::ALL_INTERSECTIONS)
      {
        const auto intersection_point = mRay.GetPoint(*inIntersectionDistance);
        ioIntersections.emplace_back(*inIntersectionDistance, inPrimitiveIndex);
      }
      else if constexpr (TIntersectMode == EIntersectMode::ONLY_CLOSEST)
      {
        if (ioIntersections.empty() || *inIntersectionDistance < ioIntersections.front().mDistance)
        {
          // Only save the closest intersection out of all primitives
          ioIntersections.emplace_back(*inIntersectionDistance, inPrimitiveIndex);
          ioIntersections.resize(0);
        }
      }
    }
  }

  template <EIntersectMode TIntersectMode>
  auto IntersectRecursive(const Octree<TPrimitive>& inOctree,
      const std::vector<TPrimitive>& inPrimitivesPool,
      std::vector<typename Octree<TPrimitive>::Intersection>& ioIntersections)
  {
    using OctreeType = Octree<TPrimitive>;
    using ChildSequentialIndexType = typename OctreeType::ChildSequentialIndex;

    const auto aabox_size = inOctree.mAABox.GetSize();
    const auto aabox_half_size = (aabox_size / static_cast<ValueType>(2));

    // Check whether this AABox needs to be checked because of max distance or not
    if (mMaxDistance != Infinity<ValueType>())
    {
      const auto max_box_half_size = Max(aabox_half_size);
      const auto aabox_sphere = Sphere<ValueType>(inOctree.mAABox.GetCenter(), max_box_half_size);
      const auto ray_max_distance_sphere = Sphere<ValueType>(mRay.GetOrigin(), mMaxDistance);
      if (!::ez::IntersectCheck(aabox_sphere, ray_max_distance_sphere))
      {
        if constexpr (TIntersectMode == EIntersectMode::ONLY_CHECK)
        {
          return false;
        }
        else
        {
          return;
        }
      }
    }

    if (inOctree.IsLeaf())
    {
      // Base case, linear search through its contained primitives
      for (const auto& primitive_index : inOctree.mPrimitivesIndices)
      {
        const auto& primitive = inPrimitivesPool.at(primitive_index);

        const auto primitive_intersections = ::ez::Intersect<TIntersectMode>(mRay, primitive);
        // TODO: Put this if/else below into a separate function, as done with TreatIntersectionResult
        if constexpr (IsArray_v<decltype(primitive_intersections)>)
        {
          if constexpr (TIntersectMode == EIntersectMode::ONLY_CHECK)
          {
            if (std::any_of(primitive_intersections.cbegin(),
                    primitive_intersections.cend(),
                    [&](const auto& inHasIntersected) { return inHasIntersected; }))
              return true;
          }
          else
          {
            for (const auto& primitive_intersection : primitive_intersections)
            {
              TreatIntersectionResult<TIntersectMode>(inOctree,
                  primitive_index,
                  primitive_intersection,
                  ioIntersections);
            }
          }
        }
        else
        {
          const auto& primitive_intersection = primitive_intersections;
          if constexpr (TIntersectMode == EIntersectMode::ONLY_CHECK)
          {
            if (primitive_intersection)
              return true;
          }
          else
          {
            TreatIntersectionResult<TIntersectMode>(inOctree, primitive_index, primitive_intersection, ioIntersections);
          }
        }
      }
    }
    else
    {
      // If the ray origin is inside the octree, forward the intersection to its children
      if (Contains(inOctree.mAABox, mRay.GetOrigin()))
      {
        if constexpr (TIntersectMode == EIntersectMode::ONLY_CHECK)
        {
          for (const auto& child_octree : inOctree)
          {
            if (IntersectRecursive<TIntersectMode>(child_octree, inPrimitivesPool, ioIntersections))
              return true;
          }
          return false;
        }
        else if constexpr (TIntersectMode == EIntersectMode::ONLY_CLOSEST)
        {
          for (const auto& child_octree : inOctree)
          { IntersectRecursive<TIntersectMode>(child_octree, inPrimitivesPool, ioIntersections); }
          return;
        }
        else if constexpr (TIntersectMode == EIntersectMode::ALL_INTERSECTIONS)
        {
          for (const auto& child_octree : inOctree)
          { IntersectRecursive<TIntersectMode>(child_octree, inPrimitivesPool, ioIntersections); }
          return;
        }
      }

      // Recursive case, check which children octrees the ray intersects
      std::array<std::optional<typename OctreeType::ChildSequentialIndex>, 4> child_octree_indices_to_explore;

      // Determine entry intersection in the octree by looking at external planes.
      // This will give us the first child octree to explore.
      for (int external_plane_i = 0; external_plane_i < 6; ++external_plane_i)
      {
        const auto& external_plane_normal = OctreeType::ExternalOctreePlaneNormals.at(external_plane_i);
        if (Dot(mRay.GetDirection(), external_plane_normal) > 0) // Only consider planes from which it can enter
          continue;

        const auto remapped_external_plane_normal = Max(external_plane_normal, Zero<Vec3<ValueType>>());
        const auto external_plane_point = inOctree.mAABox.GetMin() + remapped_external_plane_normal * aabox_size;
        const auto external_plane = Plane<ValueType>(external_plane_normal, external_plane_point);
        const auto ray_plane_intersection_distance = ::ez::IntersectClosest(mRay, external_plane);
        if (!ray_plane_intersection_distance)
          continue;

        if (*ray_plane_intersection_distance > mMaxDistance)
          continue;

        const auto external_plane_id = static_cast<typename OctreeType::EExternalOctreePlaneId>(external_plane_i);
        const auto first_child_octree_id_to_explore = inOctree.GetNextChildOctreeIndexToExplore(external_plane_id,
            mRay.GetPoint(*ray_plane_intersection_distance));
        if (!first_child_octree_id_to_explore) // We have hit the plane but not the octree, keep looking
          continue;

        child_octree_indices_to_explore[0] = first_child_octree_id_to_explore;
        break; // We can only enter to the octree from one of its faces, so as soon as we find it, break
      }

      // Determine which octree children to explore by looking at the intersection with the 3 internal planes
      auto internal_plane_intersection_distances = std::array {
        std::make_tuple(std::optional<ChildSequentialIndexType>(), Infinity<ValueType>()), // MID_X
        std::make_tuple(std::optional<ChildSequentialIndexType>(), Infinity<ValueType>()), // MID_Y
        std::make_tuple(std::optional<ChildSequentialIndexType>(), Infinity<ValueType>()), // MID_Z
      };

      for (int internal_plane_i = 0; internal_plane_i < 3; ++internal_plane_i)
      {
        const auto& internal_plane_normal = OctreeType::InternalOctreePlaneNormals.at(internal_plane_i);
        const auto internal_plane_point = inOctree.mAABox.GetCenter();
        const auto internal_plane = Plane<ValueType>(internal_plane_normal, internal_plane_point);
        const auto ray_plane_intersection_distance = ::ez::IntersectClosest(mRay, internal_plane);
        if (!ray_plane_intersection_distance)
          continue;

        if (*ray_plane_intersection_distance > mMaxDistance)
          continue;

        const auto internal_plane_id = static_cast<typename OctreeType::EInternalOctreePlaneId>(internal_plane_i);
        const auto next_child_octree_id_to_explore = inOctree.GetNextChildOctreeIndexToExplore(internal_plane_id,
            mRay.GetDirection(),
            mRay.GetPoint(*ray_plane_intersection_distance));
        if (!next_child_octree_id_to_explore)
          continue;

        internal_plane_intersection_distances[internal_plane_i]
            = std::make_tuple(next_child_octree_id_to_explore, *ray_plane_intersection_distance);
      }

      // If we want to stop at first hit, order child octrees based on intersection distance
      // so that we explore them in order and we can early out asap
      if constexpr (TIntersectMode == EIntersectMode::ONLY_CLOSEST)
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

        const auto child_octree_to_explore = inOctree.GetChildOctree(*child_octree_index);
        if (!child_octree_to_explore)
          continue;

        if constexpr (TIntersectMode == EIntersectMode::ONLY_CHECK)
        {
          if (IntersectRecursive<TIntersectMode, TPrimitive>(*child_octree_to_explore,
                  inPrimitivesPool,
                  ioIntersections))
          {
            return true;
          }
        }
        else
        {
          IntersectRecursive<TIntersectMode>(*child_octree_to_explore, inPrimitivesPool, ioIntersections);
          if constexpr (TIntersectMode == EIntersectMode::ONLY_CLOSEST)
          {
            if (ioIntersections.size() >= 1)
              break;
          }
        }
      }
    }

    if constexpr (TIntersectMode == EIntersectMode::ONLY_CHECK)
    {
      return false;
    }
  }
};

template <EIntersectMode TIntersectMode, typename TPrimitive>
auto Intersect(const Octree<TPrimitive>& inTopOctree,
    const Ray3<ValueType_t<TPrimitive>>& inRay,
    const ValueType_t<TPrimitive> inMaxDistance = Infinity<ValueType_t<TPrimitive>>())
{
  IntersectHelperStruct<TPrimitive> intersecter { inRay, inMaxDistance };
  return intersecter.template Intersect<TIntersectMode>(inTopOctree);
}

template <EIntersectMode TIntersectMode, typename TPrimitive>
auto Intersect(const Ray3<ValueType_t<TPrimitive>>& inRay,
    const Octree<TPrimitive>& inTopOctree,
    const ValueType_t<TPrimitive> inMaxDistance = Infinity<ValueType_t<TPrimitive>>())
{
  return Intersect<TIntersectMode, TPrimitive>(inTopOctree, inRay, inMaxDistance);
}

}