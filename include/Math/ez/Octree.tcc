#include "Math.h"
#include "Octree.h"
#include <algorithm>

namespace ez
{
template <typename TPrimitive>
Octree<TPrimitive>::Octree(const Span<TPrimitive>& inPrimitives, const std::size_t inLeafNodesMaxCapacity)
{
  OctreeBuilder<TPrimitive> octree_builder;
  *this = octree_builder.Build(inPrimitives, inLeafNodesMaxCapacity);
}

template <typename TPrimitive>
void Octree<TPrimitive>::ForEach(const ForEachFunction& inForEachFunction)
{
  for (const auto& child_multi_index : Octree::AllChildMultiIndices01) { inForEachFunction(child_multi_index); }
}

template <typename TPrimitive>
Octree<TPrimitive>::AACube Octree<TPrimitive>::GetChildCube(const Octree::ChildMultiIndex01 inChildIndex) const
{
  return {}; // TODO
}

template <typename TPrimitive>
Octree<TPrimitive>::AACube Octree<TPrimitive>::GetChildCube(
    const typename Octree<TPrimitive>::InternalIndex inInternalIndex) const
{
  return {}; // TODO
}

template <typename TPrimitive>
Octree<TPrimitive>* Octree<TPrimitive>::GetChildOctree(const Octree::ChildMultiIndex01 inChildIndex)
{
  return mChildren[GetInternalIndex(inChildIndex)];
}

template <typename TPrimitive>
const Octree<TPrimitive>* Octree<TPrimitive>::GetChildOctree(const Octree::ChildMultiIndex01 inChildIndex) const
{
  return const_cast<Octree&>(*this).GetChildOctree(inChildIndex);
}

template <typename TPrimitive>
Octree<TPrimitive>* Octree<TPrimitive>::GetChildOctree(const typename Octree<TPrimitive>::InternalIndex inInternalIndex)
{
  return mChildren.at(inInternalIndex);
}

template <typename TPrimitive>
const Octree<TPrimitive>* Octree<TPrimitive>::GetChildOctree(
    const Octree<TPrimitive>::InternalIndex inInternalIndex) const
{
  return mChildren.at(inInternalIndex);
}
template <typename TPrimitive>
constexpr Octree<TPrimitive>::ChildMultiIndex01 Octree<TPrimitive>::GetChildMultiIndex01(
    const InternalIndex inInternalIndex)
{
  return ChildMultiIndex01 { (inChildIndex[0] % 8u) / 4u, (inChildIndex[1] % 4u) / 2u, (inChildIndex[2] % 2u) / 1u };
}

template <typename TPrimitive>
constexpr typename Octree<TPrimitive>::InternalIndex Octree<TPrimitive>::GetInternalIndex(
    const Octree::ChildMultiIndex01 inChildIndex)
{
  return inChildIndex[0] * 4 + inChildIndex[1] * 2 + inChildIndex[2] * 1;
}

template <typename TPrimitive>
template <bool IsConst>
Octree<TPrimitive>::GIterator<IsConst>::GIterator(OctreeType& ioOctree, const InternalIndex inBeginIndex)
    : mOctree { ioOctree }, mBeginIndex(inBeginIndex)
{
}

template <typename TPrimitive>
template <bool IsConst>
typename Octree<TPrimitive>::GIterator<IsConst>& Octree<TPrimitive>::GIterator<IsConst>::operator++()
{
  EXPECTS(mCurrentIndex < 7);

  do
  {
    ++mCurrentIndex;
  } while (mCurrentIndex < 7 && mCurrentOctree.GetChildOctree(mCurrentIndex) == nullptr);

  ENSURES(mCurrentIndex < 8);

  return *this;
}

template <typename TPrimitive>
template <bool IsConst>
typename Octree<TPrimitive>::GIterator<IsConst>::OctreeType& Octree<TPrimitive>::GIterator<IsConst>::operator*()
{
  const auto current_octree = mCurrentOctree.GetChildOctree(mCurrentIndex);
  EXPECTS(current_octree);
  return *current_octree;
}

template <typename TPrimitive>
Octree<TPrimitive> OctreeBuilder<TPrimitive>::Build(const Span<TPrimitive>& inPrimitives,
    const std::size_t inLeafNodesMaxCapacity)
{
  const auto bounding_cube = BoundingHyperRectangle(inPrimitives);
  return BuildRecursive(bounding_cube, inPrimitives, inLeafNodesMaxCapacity);
}

template <typename TPrimitive>
Octree<TPrimitive> OctreeBuilder<TPrimitive>::BuildRecursive(
    const typename Octree<TPrimitive>::AACube& inAABoundingCube,
    const Span<TPrimitive>& inPrimitives,
    const std::size_t inLeafNodesMaxCapacity)
{
  if (inPrimitives.GetNumberOfElements() > inLeafNodesMaxCapacity) {}
  else // Base case
  {
    for (int x = 0; x <= 1; ++x)
    {
      for (int y = 0; y <= 1; ++y)
      {
        for (int z = 0; z <= 1; ++z)
        {
          auto child_octree = std::make_unique<Octree<TPrimitive>>();
          octree.mPrimitives.reserve(inPrimitives.GetNumberOfElements() / 8);
          for (const auto& primitive : inPrimitives)
          {
            if (Contains(inAABoundingCube, primitive))
            {
              inPrimitives.push_back(primitive);
            }
          }
          child_octree->Build(inPrimitives, inLeafNodesMaxCapacity);
        }
      }
    }
    octree.mPrimitives = inPrimitives;
  }
}
}