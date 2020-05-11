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
  using InternalIndex = std::size_t;

  Octree() = default;
  Octree(const Span<TPrimitive>& inPrimitives,
      const std::size_t inLeafNodesMaxCapacity = 8,
      const std::size_t inMaxDepth = 8);
  Octree(const Octree&) = delete;
  Octree& operator=(const Octree&) = delete;
  Octree(Octree&&) = default;
  Octree& operator=(Octree&&) = default;

  using ForEachFunction = std::function<void(const Octree::ChildMultiIndex01 inChildIndex, Octree& ioChildOctree)>;
  void ForEach(const ForEachFunction& inForEachFunction);

  using ForEachFunctionConst
      = std::function<void(const Octree::ChildMultiIndex01 inChildIndex, const Octree& inChildOctree)>;
  void ForEach(const ForEachFunctionConst& inForEachFunctionConst) const;

  const AACubef& GetAACube() const { return mAACube; }
  const std::vector<TPrimitive>& GetPrimitives() const { return mPrimitives; }
  const std::array<std::unique_ptr<Octree>, 8>& GetChildren() const { return mChildren; }
  AACubeType GetChildAACube(const Octree::ChildMultiIndex01 inChildIndex) const;
  AACubeType GetChildAACube(const InternalIndex inInternalIndex) const;
  Octree* GetChildOctree(const Octree::ChildMultiIndex01 inChildIndex);
  const Octree* GetChildOctree(const Octree::ChildMultiIndex01 inChildIndex) const;
  Octree* GetChildOctree(const InternalIndex inInternalIndex);
  const Octree* GetChildOctree(const InternalIndex inInternalIndex) const;
  bool IsEmpty() const { return mPrimitives.empty(); }
  bool IsLeaf() const;

  template <bool IsConst>
  class GIterator final
  {
  public:
    using OctreeType = std::conditional_t<IsConst, const Octree, Octree>;

    GIterator(OctreeType& ioOctree, const Octree::InternalIndex inBeginIndex);
    GIterator& operator++();
    bool operator==(const GIterator& inRHS) const { return mCurrentIndex == inRHS.mCurrentIndex; }
    bool operator!=(const GIterator& inRHS) const { return !(*this == inRHS); }
    OctreeType& operator*();
    const OctreeType& operator*() const { return const_cast<GIterator&>(*this).operator*(); }

  private:
    OctreeType& mOctree;
    InternalIndex mCurrentIndex = 0;
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
  static inline constexpr std::array<ChildMultiIndex01, 8> AllChildMultiIndices01 = {
    MakeBinaryIndex<3>(0),
    MakeBinaryIndex<3>(1),
    MakeBinaryIndex<3>(2),
    MakeBinaryIndex<3>(3),
    MakeBinaryIndex<3>(4),
    MakeBinaryIndex<3>(5),
    MakeBinaryIndex<3>(6),
    MakeBinaryIndex<3>(7),
  };

  AACube<ValueType> mAACube;
  std::vector<TPrimitive> mPrimitives;
  std::array<std::unique_ptr<Octree>, 8> mChildren;

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