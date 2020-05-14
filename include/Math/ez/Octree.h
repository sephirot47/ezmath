#pragma once

#include "ez/AACube.h"
#include "ez/BinaryIndex.h"
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

  Octree() = default;
  Octree(const Span<TPrimitive>& inPrimitives,
      const std::size_t inLeafNodesMaxCapacity = 8,
      const std::size_t inMaxDepth = 8);
  Octree(const Octree&) = delete;
  Octree& operator=(const Octree&) = delete;
  Octree(Octree&&) = default;
  Octree& operator=(Octree&&) = default;

  std::vector<ValueType> IntersectAll(const Ray3<ValueType>& inRay) const;

  const AACube<ValueType>& GetAACube() const { return mAACube; }
  const std::vector<TPrimitive>& GetPrimitives() const { return mPrimitives; }
  const std::array<std::unique_ptr<Octree>, 8>& GetChildren() const { return mChildren; }
  AACubeType GetChildAACube(const Octree::ChildMultiIndex01 inChildIndex) const;
  AACubeType GetChildAACube(const ChildSequentialIndex inChildSequentialIndex) const;
  Octree* GetChildOctree(const Octree::ChildMultiIndex01 inChildIndex);
  const Octree* GetChildOctree(const Octree::ChildMultiIndex01 inChildIndex) const;
  Octree* GetChildOctree(const ChildSequentialIndex inChildSequentialIndex);
  const Octree* GetChildOctree(const ChildSequentialIndex inChildSequentialIndex) const;
  bool IsEmpty() const { return mPrimitives.empty(); }
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
  enum class EOctreePlaneId
  {
    // The order matters (see IntersectAllRecursive)
    LEFT,   // X-
    RIGHT,  // X+
    BOTTOM, // Y-
    TOP,    // Y+
    FRONT,  // Z-
    BACK,   // Z+
    MID_X,  // XM
    MID_Y,  // YM
    MID_Z   // ZM
  };
  constexpr static bool IsExternalPlane(const EOctreePlaneId& inPlaneId) { return static_cast<int>(inPlaneId) < 6; }
  constexpr static bool IsMidPlane(const EOctreePlaneId& inPlaneId) { return !IsExternalPlane(inPlaneId); }

  static constexpr auto OctreePlaneNormals = std::array {
    // The order must match EOctreePlaneId
    Left<Vec3<ValueType>>(),    // LEFT: 0
    Right<Vec3<ValueType>>(),   // RIGHT: 1
    Down<Vec3<ValueType>>(),    // BOTTOM: 2
    Up<Vec3<ValueType>>(),      // TOP: 3
    Forward<Vec3<ValueType>>(), // FRONT: 4
    Back<Vec3<ValueType>>(),    // BACK: 5
    Right<Vec3<ValueType>>(),   // MID_X: 6
    Up<Vec3<ValueType>>(),      // MID_Y: 7
    Back<Vec3<ValueType>>(),    // MID_Z: 8
  };

  AACube<ValueType> mAACube;
  std::vector<TPrimitive> mPrimitives;
  std::array<std::unique_ptr<Octree>, 8> mChildren;

  std::optional<ChildSequentialIndex> GetNextChildOctreeIndexToExplore(const EOctreePlaneId& inOctreePlaneId,
      const Vec3<ValueType>& inRayDirection,
      const Vec3<ValueType>& inIntersectionPoint) const;
  void IntersectAllRecursive(const Ray3<ValueType>& inRay, std::vector<ValueType>& ioIntersections) const;

  friend class OctreeBuilder<TPrimitive>;
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
  static Octree<TPrimitive> BuildRecursive(const typename Octree<TPrimitive>::AACubeType& inAABoundingCube,
      const Span<TPrimitive>& inPrimitives,
      const std::size_t inLeafNodesMaxCapacity,
      const std::size_t inMaxDepth,
      const std::size_t inCurrentDepth);
};
}

#include "Octree.tcc"