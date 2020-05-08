#pragma once

#include "AACube.h"
#include "MathTypeTraits.h"
#include "Span.h"
#include <array>
#include <memory>
#include <vector>

namespace ez
{
template <typename TPrimitive>
class Octree final
{
public:
  using ChildMultiIndex01 = Vec<uint32_t, 3>;
  using ValueType = ValueType_t<TPrimitive>;
  using AACube = AACube<ValueType>;
  using InternalIndex = std::size_t;

  Octree() = default;
  Octree(const Span<TPrimitive>& inPrimitives, const std::size_t inLeafNodesMaxCapacity = 8);
  Octree(const Octree&) = delete;
  Octree& operator=(const Octree&) = delete;
  Octree(Octree&&) = default;
  Octree& operator=(Octree&&) = default;

  using ForEachFunction = std::function<void(const Octree::ChildMultiIndex01 inChildIndex, Octree& ioChildOctree)>;
  void ForEach(const ForEachFunction& inForEachFunction);

  using ForEachFunctionConst
      = std::function<void(const Octree::ChildMultiIndex01 inChildIndex, const Octree& inChildOctree)>;
  void ForEach(const ForEachFunctionConst& inForEachFunctionConst) const;

  AACube GetChildCube(const Octree::ChildMultiIndex01 inChildIndex) const;
  AACube GetChildCube(const InternalIndex inInternalIndex) const;
  Octree* GetChildOctree(const Octree::ChildMultiIndex01 inChildIndex);
  const Octree* GetChildOctree(const Octree::ChildMultiIndex01 inChildIndex) const;
  Octree* GetChildOctree(const InternalIndex inInternalIndex);
  const Octree* GetChildOctree(const InternalIndex inInternalIndex) const;

  template <bool IsConst>
  class GIterator
  {
  public:
    using OctreeType = std::conditional_t<IsConst, Octree, const Octree>;

    GIterator(OctreeType& ioOctree, const Octree::InternalIndex inBeginIndex);
    GIterator& operator++();
    bool operator==(const Iterator& inRHS) const { return mCurrentIndex != inRHS.mCurrentIndex; }
    bool operator!=(const Iterator& inRHS) const { return !(*this == inRHS); }
    OctreeType& operator*();
    const OctreeType& operator*() const { return const_cast<Iterator&>(*this).operator*(); }

  private:
    OctreeType& mOctree;
    InternalIndex mCurrentIndex = 0;
  };
  using Iterator = GIterator<false>;
  using ConstIterator = GIterator<true>;

  Octree::Iterator begin() { return Iterator(*this, 0); }
  Octree::Iterator end()   { return Iterator(*this, 8); }
  Octree::ConstIterator begin() const { return cbegin(); }
  Octree::ConstIterator end() const { return cend(); }
  Octree::ConstIterator cbegin() const { return ConstIterator(*this, 0); }
  Octree::ConstIterator cend() const { return ConstIterator(*this, 8); }

private:
  static inline constexpr std::array<ChildMultiIndex01, 8> AllChildMultiIndices01 = {
    GetChildMultiIndex(0u),
    GetChildMultiIndex(1u),
    GetChildMultiIndex(2u),
    GetChildMultiIndex(3u),
    GetChildMultiIndex(4u),
    GetChildMultiIndex(5u),
    GetChildMultiIndex(6u),
    GetChildMultiIndex(7u),
  };

  std::vector<TPrimitive> mPrimitives;
  std::array<std::unique_ptr<Octree>, 8> mChildren;

  static constexpr Octree::ChildMultiIndex01 GetChildMultiIndex01(const InternalIndex inInternalIndex);
  static constexpr InternalIndex GetInternalIndex(const Octree::ChildMultiIndex01 inChildIndex);

  friend class OctreeBuilder<TPrimitive>;
};

template <typename TPrimitive>
class OctreeBuilder final
{
public:
  OctreeBuilder() = delete;

  static Octree<TPrimitive> Build(const Span<TPrimitive>& inPrimitives, const std::size_t inLeafNodesMaxCapacity = 8);

private:
  static Octree<TPrimitive> BuildRecursive(const typename Octree<TPrimitive>::AACube& inAABoundingCube,
      const Span<TPrimitive>& inPrimitives,
      const std::size_t inLeafNodesMaxCapacity);
};
}

#include "Octree.tcc"