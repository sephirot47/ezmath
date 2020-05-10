#include "Math.h"
#include "Octree.h"
#include <algorithm>

namespace ez
{
template <typename TPrimitive>
Octree<TPrimitive>::Octree(const Span<TPrimitive>& inPrimitives, const std::size_t inLeafNodesMaxCapacity)
{
  *this = OctreeBuilder<TPrimitive>::Build(inPrimitives, inLeafNodesMaxCapacity);
}

template <typename TPrimitive>
void Octree<TPrimitive>::ForEach(const ForEachFunction& inForEachFunction)
{
  for (const auto& child_multi_index : Octree::AllChildMultiIndices01) { inForEachFunction(child_multi_index); }
}

template <typename TPrimitive>
Octree<TPrimitive>::AACubeType Octree<TPrimitive>::GetChildAACube(const Octree::ChildMultiIndex01 inChildIndex) const
{
  const auto child_cube_size = (mAACube.GetSize() / static_cast<ValueType>(2));
  const auto child_cube_size_indexed = (child_cube_size * Vec3<ValueType>(inChildIndex));
  return AACube<ValueType>(mAACube.GetMin() + child_cube_size_indexed, mAACube.GetCenter() + child_cube_size_indexed);
}

template <typename TPrimitive>
Octree<TPrimitive>::AACubeType Octree<TPrimitive>::GetChildAACube(
    const typename Octree<TPrimitive>::InternalIndex inInternalIndex) const
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
Octree<TPrimitive>* Octree<TPrimitive>::GetChildOctree(const typename Octree<TPrimitive>::InternalIndex inInternalIndex)
{
  EXPECTS(inInternalIndex < mChildren.size());
  return mChildren.at(inInternalIndex).get();
}

template <typename TPrimitive>
const Octree<TPrimitive>* Octree<TPrimitive>::GetChildOctree(
    const Octree<TPrimitive>::InternalIndex inInternalIndex) const
{
  return const_cast<Octree&>(*this).GetChildOctree(inInternalIndex);
}

template <typename TPrimitive>
template <bool IsConst>
Octree<TPrimitive>::GIterator<IsConst>::GIterator(OctreeType& ioOctree, const InternalIndex inBeginIndex)
    : mOctree { ioOctree }, mCurrentIndex(inBeginIndex)
{
  if (mCurrentIndex < 8
      && !mOctree.GetChildOctree(mCurrentIndex)) // Advance until the first valid index (or end() if there isnt)
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
    const std::size_t inLeafNodesMaxCapacity)
{
  const auto bounding_aa_cube = BoundingAACube(inPrimitives);
  return BuildRecursive(bounding_aa_cube, inPrimitives, inLeafNodesMaxCapacity);
}

template <typename TPrimitive>
Octree<TPrimitive> OctreeBuilder<TPrimitive>::BuildRecursive(
    const typename Octree<TPrimitive>::AACubeType& inAABoundingCube,
    const Span<TPrimitive>& inPrimitives,
    const std::size_t inLeafNodesMaxCapacity)
{
  Octree<TPrimitive> octree;
  octree.mAACube = inAABoundingCube;
  octree.mPrimitives.reserve(inPrimitives.GetNumberOfElements() / 8);
  std::copy_if(inPrimitives.cbegin(),
      inPrimitives.cend(),
      std::back_inserter(octree.mPrimitives),
      [&](const auto& inPrimitive) { return Intersect(inAABoundingCube, inPrimitive); });

  if (octree.mPrimitives.size() > inLeafNodesMaxCapacity)
  {
    for (std::size_t i = 0; i < 8; ++i)
    {
      const auto child_bounding_aa_cube = octree.GetChildAACube(i);
      auto built_child = BuildRecursive(child_bounding_aa_cube, MakeSpan(octree.mPrimitives), inLeafNodesMaxCapacity);
      if (!built_child.IsEmpty())
      {
        octree.mChildren[i] = std::make_unique<Octree<TPrimitive>>(std::move(built_child));
      }
    }
  }

  return octree;
}
}