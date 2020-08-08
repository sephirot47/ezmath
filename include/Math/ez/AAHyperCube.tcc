#include "ez/AAHyperCube.h"
#include "ez/BinaryIndex.h"
#include "ez/Macros.h"
#include "ez/MathTypeTraits.h"

namespace ez
{

template <typename T, std::size_t N>
AAHyperCube<T, N>::AAHyperCube()
{
}

template <typename T, std::size_t N>
template <typename TOther>
AAHyperCube<T, N>::AAHyperCube(const AAHyperCube<TOther, N>& inRHS)
    : AAHyperCube(Vec<T, N> { inRHS.GetMin() }, static_cast<T>(inRHS.GetSize()))
{
}

template <typename T, std::size_t N>
AAHyperCube<T, N>::AAHyperCube(const Vec<T, N>& inMin, const T& inSize) : mMin { inMin }, mSize { inSize }
{
  EXPECTS(mSize >= static_cast<T>(0));
}

template <typename T, std::size_t N>
void AAHyperCube<T, N>::SetMin(const Vec<T, N>& inMin)
{
  mMin = inMin;
}

template <typename T, std::size_t N>
void AAHyperCube<T, N>::SetSize(const T& inSize)
{
  EXPECTS(mSize >= static_cast<T>(0));
  mSize = inSize;
}

template <typename T, std::size_t N>
bool AAHyperCube<T, N>::operator==(const AAHyperCube& inRHS) const
{
  return mMin == inRHS.mMin && mSize == inRHS.mSize;
}

template <typename T, std::size_t N>
AAHyperCube<T, N> AAHyperCube<T, N>::operator+(const Vec<T, N>& inRHS)
{
  return AAHyperCube<T, N>(mMin + inRHS, mSize);
}

template <typename T, std::size_t N>
AAHyperCube<T, N>& AAHyperCube<T, N>::operator+=(const Vec<T, N>& inRHS)
{
  *this = (*this + inRHS);
  return *this;
}

template <typename T, std::size_t N>
AAHyperCube<T, N> AAHyperCube<T, N>::operator*(const Vec<T, N>& inRHS)
{
  return AAHyperCube<T, N> { mMin * inRHS, mSize * inRHS };
}

template <typename T, std::size_t N>
AAHyperCube<T, N>& AAHyperCube<T, N>::operator*=(const Vec<T, N>& inRHS)
{
  *this = (*this) * inRHS;
  return *this;
}

template <EIntersectMode TIntersectMode, typename T, std::size_t N>
auto Intersect(const AAHyperCube<T, N>& inLHS, const AAHyperCube<T, N>& inRHS)
{
  return Intersect<TIntersectMode>(MakeAAHyperBoxFromAAHyperCube(inLHS), MakeAAHyperBoxFromAAHyperCube(inRHS));
}

template <typename T, std::size_t N>
bool Contains(const AAHyperCube<T, N>& inAAHyperCube, const Vec<T, N>& inPoint)
{
  return inPoint >= inAAHyperCube.GetMin() && inPoint <= inAAHyperCube.GetMax();
}

template <typename T, std::size_t N>
bool Contains(const AAHyperCube<T, N>& inAAHyperCubeContainer, const AAHyperCube<T, N>& inAAHyperCubeContainee)
{
  return Contains(inAAHyperCubeContainer, inAAHyperCubeContainee.GetMin())
      && Contains(inAAHyperCubeContainer, inAAHyperCubeContainee.GetMax());
}

template <typename T, std::size_t N>
const auto MakeAAHyperCubeFromMinSize(const Vec<T, N>& inMin, const T& inSize)
{
  return AAHyperCube<T, N>(inMin, inSize);
}

template <typename T, std::size_t N>
const auto MakeAAHyperCubeFromCenterSize(const Vec<T, N>& inCenter, const T& inSize)
{
  return MakeAAHyperCubeFromMinSize(inCenter - inSize / 2, inSize);
}

template <typename T, std::size_t N>
constexpr Vec<T, N> Center(const AAHyperCube<T, N>& inAAHyperCube)
{
  return (inAAHyperCube.GetMin() + inAAHyperCube.GetMax()) / static_cast<T>(2);
}
}