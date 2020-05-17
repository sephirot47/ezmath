#pragma once

#include "ez/AACube.h"
#include "ez/BinaryIndex.h"
#include "ez/IntersectMode.h"
#include "ez/MathTypeTraits.h"
#include "ez/Span.h"
#include <array>
#include <memory>
#include <vector>

namespace ez
{

template <typename TPrimitive>
class OctreeBuilder;

template <typename TPrimitive>
class Octree final
{
public:
  using ChildMultiIndex01 = BinaryIndex<3>;
  using ValueType = ValueType_t<TPrimitive>;
  using AACubeType = AACube<ValueType>;
  using ChildSequentialIndex = std::size_t;
  using PrimitiveIndex = std::size_t;

  struct Intersection final
  {
    ValueType mDistance { Infinity<ValueType>() };            // The distance to the intersection
    PrimitiveIndex mPrimitiveIndex { Max<PrimitiveIndex>() }; // The intersected primitive index in the primitive pool
  };

  Octree() = default;
  Octree(const Span<TPrimitive>& inPrimitives,
      const std::size_t inLeafNodesMaxCapacity = 8,
      const std::size_t inMaxDepth = 8);
  Octree(const Octree&) = delete;
  Octree& operator=(const Octree&) = delete;
  Octree(Octree&&) = default;
  Octree& operator=(Octree&&) = default;

  const AACube<ValueType>& GetAACube() const { return mAACube; }
  const std::vector<TPrimitive>& GetPrimitivesPool() const; // Only available in top Octree
  const std::vector<PrimitiveIndex>& GetPrimitivesIndices() const { return mPrimitivesIndices; }
  const std::array<std::unique_ptr<Octree>, 8>& GetChildren() const { return mChildren; }
  AACubeType GetChildAACube(const ChildSequentialIndex inChildSequentialIndex) const;
  Octree* GetChildOctree(const Octree::ChildMultiIndex01 inChildIndex);
  const Octree* GetChildOctree(const Octree::ChildMultiIndex01 inChildIndex) const;
  Octree* GetChildOctree(const ChildSequentialIndex inChildSequentialIndex);
  const Octree* GetChildOctree(const ChildSequentialIndex inChildSequentialIndex) const;
  bool IsEmpty() const { return mPrimitivesIndices.empty(); }
  bool IsLeaf() const;

  template <bool IsConst>
  class GIterator final
  {
  public:
    using OctreeType = std::conditional_t<IsConst, const Octree, Octree>;

    GIterator(OctreeType& ioOctree, const Octree::ChildSequentialIndex inBeginIndex);
    GIterator& operator++();
    bool operator==(const GIterator& inRHS) const { return mCurrentIndex == inRHS.mCurrentIndex; }
    bool operator!=(const GIterator& inRHS) const { return !(*this == inRHS); }
    OctreeType& operator*();
    const OctreeType& operator*() const { return const_cast<GIterator&>(*this).operator*(); }

  private:
    OctreeType& mOctree;
    ChildSequentialIndex mCurrentIndex = 0;
  };
  using Iterator = GIterator<false>;
  using ConstIterator = GIterator<true>;

  Octree::Iterator begin() { return Iterator(*this, 0); }
  Octree::Iterator end() { return Iterator(*this, 8); }
  Octree::ConstIterator begin() const { return cbegin(); }
  Octree::ConstIterator end() const { return cend(); }
  Octree::ConstIterator cbegin() const { return ConstIterator(*this, 0); }
  Octree::ConstIterator cend() const { return ConstIterator(*this, 8); }

private:
  enum class EExternalOctreePlaneId
  {
    // Order matters
    LEFT,   // X-
    RIGHT,  // X+
    BOTTOM, // Y-
    TOP,    // Y+
    FRONT,  // Z-
    BACK,   // Z+
  };

  enum class EInternalOctreePlaneId
  {
    // Order matters
    MID_X,
    MID_Y,
    MID_Z
  };

  static constexpr auto ExternalOctreePlaneNormals = std::array {
    // The order must match EExternalOctreePlaneId
    Left<Vec3<ValueType>>(),    // LEFT: 0
    Right<Vec3<ValueType>>(),   // RIGHT: 1
    Down<Vec3<ValueType>>(),    // BOTTOM: 2
    Up<Vec3<ValueType>>(),      // TOP: 3
    Forward<Vec3<ValueType>>(), // FRONT: 4
    Back<Vec3<ValueType>>(),    // BACK: 5
  };

  static constexpr auto InternalOctreePlaneNormals = std::array {
    // The order must match EInternalOctreePlaneId
    Right<Vec3<ValueType>>(), // MID_X: 0
    Up<Vec3<ValueType>>(),    // MID_Y: 1
    Back<Vec3<ValueType>>(),  // MID_Z: 2
  };

  AACube<ValueType> mAACube;
  std::optional<std::vector<TPrimitive>> mPrimitivesPool; // Only filled in top Octree
  std::vector<std::size_t> mPrimitivesIndices;
  std::array<std::unique_ptr<Octree>, 8> mChildren;

  std::optional<ChildSequentialIndex> GetNextChildOctreeIndexToExplore(
      const EExternalOctreePlaneId& inExternalOctreePlaneId,
      const Vec3<ValueType>& inIntersectionPoint) const;
  std::optional<ChildSequentialIndex> GetNextChildOctreeIndexToExplore(
      const EInternalOctreePlaneId& inInternalOctreePlaneId,
      const Vec3<ValueType>& inRayDirection,
      const Vec3<ValueType>& inIntersectionPoint) const;

  friend class OctreeBuilder<TPrimitive>;
  friend class IntersectHelperStruct;
};

template <typename TPrimitive>
class OctreeBuilder final
{
public:
  OctreeBuilder() = delete;

  static Octree<TPrimitive> Build(const Span<TPrimitive>& inPrimitives,
      const std::size_t inLeafNodesMaxCapacity = 8,
      const std::size_t inMaxDepth = 8);

private:
  static Octree<TPrimitive> BuildRecursive(const typename Octree<TPrimitive>::AACubeType& inBoundingAACube,
      const Span<TPrimitive>& inTopOctreePrimitivesPool,
      const Span<typename Octree<TPrimitive>::PrimitiveIndex>& inParentPrimitivesIndices,
      const std::size_t inLeafNodesMaxCapacity,
      const std::size_t inMaxDepth,
      const std::size_t inCurrentDepth);
};
}

#include "Octree.tcc"