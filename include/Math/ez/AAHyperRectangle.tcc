#include "ez/AAHyperRectangle.h"
#include "ez/BinaryIndex.h"
#include "ez/Macros.h"
#include "ez/MathTypeTraits.h"

namespace ez
{

template <typename T, std::size_t N>
AAHyperRectangle<T, N>::AAHyperRectangle()
{
}

template <typename T, std::size_t N>
AAHyperRectangle<T, N>::AAHyperRectangle(const Vec<T, N>& inMin, const Vec<T, N>& inMax) : mMinMax { inMin, inMax }
{
  EXPECTS(mMinMax[0] <= mMinMax[1]);
}

template <typename T, std::size_t N>
void AAHyperRectangle<T, N>::SetMin(const Vec<T, N>& inMin)
{
  EXPECTS(inMin <= mMinMax[1]);
  mMinMax[0] = inMin;
}

template <typename T, std::size_t N>
void AAHyperRectangle<T, N>::SetMax(const Vec<T, N>& inMax)
{
  EXPECTS(inMax <= mMinMax[1]);
  mMinMax[1] = inMax;
}

template <typename T, std::size_t N>
void AAHyperRectangle<T, N>::SetMinMax(const Vec<T, N>& inMin, const Vec<T, N>& inMax)
{
  EXPECTS(inMin <= inMax);
  mMinMax[0] = inMin;
  mMinMax[1] = inMax;
}

template <typename T, std::size_t N>
void AAHyperRectangle<T, N>::Wrap(const Vec<T, N>& inPoint)
{
  mMinMax[0] = Min(mMinMax[0], inPoint);
  mMinMax[1] = Max(mMinMax[1], inPoint);
}

template <typename T, std::size_t N>
bool AAHyperRectangle<T, N>::operator==(const AAHyperRectangle& inRHS) const
{
  return mMinMax[0] == inRHS.mMinMax[0] && mMinMax[1] == inRHS.mMinMax[1];
}

template <typename T, std::size_t N>
bool AAHyperRectangle<T, N>::operator<(const AAHyperRectangle& inRHS) const
{
  return mMinMax[0] < inRHS.mMinMax[0] && mMinMax[1] < inRHS.mMinMax[1];
}

template <typename T, std::size_t N>
bool AAHyperRectangle<T, N>::operator<=(const AAHyperRectangle& inRHS) const
{
  return mMinMax[0] <= inRHS.mMinMax[0] && mMinMax[1] <= inRHS.mMinMax[1];
}

template <typename T, std::size_t N>
bool AAHyperRectangle<T, N>::operator>(const AAHyperRectangle& inRHS) const
{
  return mMinMax[0] > inRHS.mMinMax[0] && mMinMax[1] > inRHS.mMinMax[1];
}

template <typename T, std::size_t N>
bool AAHyperRectangle<T, N>::operator>=(const AAHyperRectangle& inRHS) const
{
  return mMinMax[0] >= inRHS.mMinMax[0] && mMinMax[1] >= inRHS.mMinMax[1];
}

template <typename T, std::size_t N>
template <typename TOther>
void AAHyperRectangle<T, N>::Wrap(const TOther& inThingToBound)
{
  if constexpr (IsVec_v<T>)
  {
    Wrap(inThingToBound);
  }
  else
  {
    for (const auto& subthing_to_bound : inThingToBound) { Wrap(subthing_to_bound); }
  }
}

template <typename T, std::size_t N>
template <bool IsConst>
AAHyperRectangle<T, N>::GPointsIterator<IsConst>::GPointsIterator(AAHyperRectangleType& ioHyperRectangle,
    const std::size_t inBeginIndex)
    : mAAHyperRectangle { ioHyperRectangle }, mCurrentIndex(inBeginIndex)
{
}

template <typename T, std::size_t N>
template <bool IsConst>
typename AAHyperRectangle<T, N>::template GPointsIterator<IsConst>&
AAHyperRectangle<T, N>::GPointsIterator<IsConst>::operator++()
{
  EXPECTS((mCurrentIndex < AAHyperRectangle<T, N>::NumPoints));
  ++mCurrentIndex;
  return *this;
}

template <typename T, std::size_t N>
template <bool IsConst>
typename AAHyperRectangle<T, N>::template GPointsIterator<IsConst>::VecType
    AAHyperRectangle<T, N>::GPointsIterator<IsConst>::operator*() const
{
  const auto current_binary_index = MakeBinaryIndex<N, T>(mCurrentIndex);
  const auto point = mAAHyperRectangle.GetMin() + mAAHyperRectangle.GetSize() * current_binary_index;
  return point;
}

}