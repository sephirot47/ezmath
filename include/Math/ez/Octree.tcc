#include "ez/Math.h"
#include "ez/Octree.h"
#include "ez/Sphere.h"
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
const std::vector<TPrimitive>& Octree<TPrimitive>::GetPrimitivesPool() const
{
  EXPECTS(mPrimitivesPool);
  return *mPrimitivesPool;
}

template <typename TPrimitive>
Octree<TPrimitive>::AACubeType Octree<TPrimitive>::GetChildAACube(
    const typename Octree<TPrimitive>::ChildSequentialIndex inInternalIndex) const
{
  using ValueType = ValueType_t<TPrimitive>;
  const auto child_multi_index = MakeBinaryIndex<3, ValueType>(inInternalIndex);
  const auto child_aacube_size = (mAACube.GetSize() / static_cast<ValueType>(2));
  const auto child_aacube_size_indexed = (child_aacube_size * child_multi_index);
  const auto child_aacube_min = mAACube.GetMin() + child_aacube_size_indexed;
  const auto child_aacube_max = child_aacube_min + child_aacube_size;
  return AACube<ValueType>(child_aacube_min, child_aacube_max);
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

struct IntersectHelperStruct
{
  template <EIntersectMode TIntersectMode, typename TPrimitive>
  static auto Intersect(const Octree<TPrimitive>& inTopOctree,
      const Ray3<ValueType_t<TPrimitive>>& inRay,
      const ValueType_t<TPrimitive> inMaxDistance)
  {
    static_assert(TIntersectMode == EIntersectMode::ALL_INTERSECTIONS || TIntersectMode == EIntersectMode::ONLY_CLOSEST
            || TIntersectMode == EIntersectMode::ONLY_CHECK,
        "Unsupported EIntersectMode");
    EXPECTS(inTopOctree.mPrimitivesPool);

    using IntersectionType = typename Octree<TPrimitive>::Intersection;
    std::vector<IntersectionType> intersections;

    if constexpr (TIntersectMode == EIntersectMode::ONLY_CHECK)
    {
      return IntersectRecursive<TIntersectMode, TPrimitive>(inTopOctree,
          inRay,
          inMaxDistance,
          *inTopOctree.mPrimitivesPool,
          intersections);
    }
    else
    {
      IntersectRecursive<TIntersectMode, TPrimitive>(inTopOctree,
          inRay,
          inMaxDistance,
          *inTopOctree.mPrimitivesPool,
          intersections);

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

  template <EIntersectMode TIntersectMode, typename TPrimitive>
  static auto IntersectRecursive(const Octree<TPrimitive>& inOctree,
      const Ray3<ValueType_t<TPrimitive>>& inRay,
      const ValueType_t<TPrimitive> inMaxDistance,
      const std::vector<TPrimitive>& inPrimitivesPool,
      std::vector<typename Octree<TPrimitive>::Intersection>& ioIntersections)
  {
    using OctreeType = Octree<TPrimitive>;
    using IntersectionType = typename Octree<TPrimitive>::Intersection;
    using ChildSequentialIndexType = typename OctreeType::ChildSequentialIndex;
    using ValueType = ValueType_t<TPrimitive>;

    const auto aacube_size = inOctree.mAACube.GetSize();
    const auto aacube_half_size = (aacube_size / static_cast<ValueType>(2));

    // Check whether this AACube needs to be checked because of max distance or not
    if (inMaxDistance != Infinity<ValueType>())
    {
      const auto max_cube_half_size = Max(aacube_half_size);
      const auto aacube_sphere = Sphere<ValueType>(inOctree.mAACube.GetCenter(), max_cube_half_size);
      const auto ray_max_distance_sphere = Sphere<ValueType>(inRay.GetOrigin(), inMaxDistance);
      if (!::ez::IntersectCheck(aacube_sphere, ray_max_distance_sphere))
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
      auto closest_intersection_distance = Infinity<ValueType>();
      for (const auto& primitive_index : inOctree.mPrimitivesIndices)
      {
        const auto& primitive = inPrimitivesPool.at(primitive_index);
        const auto TreatIntersectionResult = [&](const auto& inIntersectionDistance) {
          assert(TIntersectMode == EIntersectMode::ALL_INTERSECTIONS || TIntersectMode == EIntersectMode::ONLY_CLOSEST);

          if (!inIntersectionDistance)
            return;

          if (*inIntersectionDistance > inMaxDistance)
            return; // Do not consider intersections further than the maximum distance

          // Only consider intersections of this primitive inside this cube. To avoid duplicates.
          {
            const auto intersection_point = inRay.GetPoint(*inIntersectionDistance);
            if (!Contains(inOctree.mAACube, intersection_point))
              return;
          }

          typename Octree<TPrimitive>::Intersection intersection;
          intersection.mDistance = *inIntersectionDistance;
          intersection.mPrimitiveIndex = primitive_index;

          if constexpr (TIntersectMode == EIntersectMode::ALL_INTERSECTIONS)
          {
            ioIntersections.push_back(std::move(intersection));
          }
          else if constexpr (TIntersectMode == EIntersectMode::ONLY_CLOSEST)
          {
            if (intersection.mDistance < closest_intersection_distance)
            {
              // Only save the closest intersection amongst all primitives
              ioIntersections.resize(0);
              closest_intersection_distance = intersection.mDistance;
              ioIntersections.push_back(std::move(intersection));
            }
          }
        };

        const auto primitive_intersections = ::ez::Intersect<TIntersectMode>(inRay, primitive);
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
            std::for_each(primitive_intersections.cbegin(), primitive_intersections.cend(), TreatIntersectionResult);
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
            TreatIntersectionResult(primitive_intersection);
          }
        }
      }
    }
    else
    {
      // Recursive case, check which children octrees the ray intersects
      std::array<std::optional<typename OctreeType::ChildSequentialIndex>, 4> child_octree_indices_to_explore;

      // Determine entry intersection in the octree by looking at external planes.
      // This will give us the first child octree to explore.
      for (int external_plane_i = 0; external_plane_i < 6; ++external_plane_i)
      {
        const auto& external_plane_normal = OctreeType::ExternalOctreePlaneNormals.at(external_plane_i);
        if (Dot(inRay.GetDirection(), external_plane_normal) > 0) // Only consider planes from which it can enter
          continue;

        const auto remapped_external_plane_normal = Max(external_plane_normal, Zero<Vec3<ValueType>>());
        const auto external_plane_point = inOctree.mAACube.GetMin() + remapped_external_plane_normal * aacube_size;
        const auto external_plane = Plane<ValueType>(external_plane_normal, external_plane_point);
        const auto ray_plane_intersection_distance = ::ez::IntersectClosest(inRay, external_plane);
        if (!ray_plane_intersection_distance)
          continue;

        if (*ray_plane_intersection_distance > inMaxDistance)
          continue;

        const auto external_plane_id = static_cast<typename OctreeType::EExternalOctreePlaneId>(external_plane_i);
        const auto first_child_octree_id_to_explore = inOctree.GetNextChildOctreeIndexToExplore(external_plane_id,
            inRay.GetPoint(*ray_plane_intersection_distance));
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
        const auto internal_plane_point = inOctree.mAACube.GetCenter();
        const auto internal_plane = Plane<ValueType>(internal_plane_normal, internal_plane_point);
        const auto ray_plane_intersection_distance = ::ez::IntersectClosest(inRay, internal_plane);
        if (!ray_plane_intersection_distance)
          continue;

        if (*ray_plane_intersection_distance > inMaxDistance)
          continue;

        const auto internal_plane_id = static_cast<typename OctreeType::EInternalOctreePlaneId>(internal_plane_i);
        const auto next_child_octree_id_to_explore = inOctree.GetNextChildOctreeIndexToExplore(internal_plane_id,
            inRay.GetDirection(),
            inRay.GetPoint(*ray_plane_intersection_distance));
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
                  inRay,
                  inMaxDistance,
                  inPrimitivesPool,
                  ioIntersections))
            return true;
        }
        else if constexpr (TIntersectMode == EIntersectMode::ONLY_CLOSEST)
        {
          IntersectRecursive<TIntersectMode, TPrimitive>(*child_octree_to_explore,
              inRay,
              inMaxDistance,
              inPrimitivesPool,
              ioIntersections);
          if (ioIntersections.size() >= 1)
            break;
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
  return IntersectHelperStruct::Intersect<TIntersectMode, TPrimitive>(inTopOctree, inRay, inMaxDistance);
}

template <EIntersectMode TIntersectMode, typename TPrimitive>
auto Intersect(const Ray3<ValueType_t<TPrimitive>>& inRay,
    const Octree<TPrimitive>& inTopOctree,
    const ValueType_t<TPrimitive> inMaxDistance = Infinity<ValueType_t<TPrimitive>>())
{
  return Intersect<TIntersectMode, TPrimitive>(inTopOctree, inRay, inMaxDistance);
}

}