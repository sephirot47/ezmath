#include "ez/AAHyperBox.h"
#include "ez/BinaryIndex.h"
#include "ez/Macros.h"
#include "ez/MathTypeTraits.h"

namespace ez
{

template <typename T, std::size_t N>
AAHyperBox<T, N>::AAHyperBox()
{
}

template <typename T, std::size_t N>
AAHyperBox<T, N>::AAHyperBox(const Vec<T, N>& inMin, const Vec<T, N>& inMax) : mMinMax { inMin, inMax }
{
  EXPECTS(mMinMax[0] <= mMinMax[1]);
}

template <typename T, std::size_t N>
void AAHyperBox<T, N>::SetMin(const Vec<T, N>& inMin)
{
  EXPECTS(inMin <= mMinMax[1]);
  mMinMax[0] = inMin;
}

template <typename T, std::size_t N>
void AAHyperBox<T, N>::SetMax(const Vec<T, N>& inMax)
{
  EXPECTS(inMax <= mMinMax[1]);
  mMinMax[1] = inMax;
}

template <typename T, std::size_t N>
void AAHyperBox<T, N>::SetMinMax(const Vec<T, N>& inMin, const Vec<T, N>& inMax)
{
  EXPECTS(inMin <= inMax);
  mMinMax[0] = inMin;
  mMinMax[1] = inMax;
}

template <typename T, std::size_t N>
void AAHyperBox<T, N>::Wrap(const Vec<T, N>& inPoint)
{
  mMinMax[0] = Min(mMinMax[0], inPoint);
  mMinMax[1] = Max(mMinMax[1], inPoint);
}

template <typename T, std::size_t N>
bool AAHyperBox<T, N>::operator==(const AAHyperBox& inRHS) const
{
  return mMinMax[0] == inRHS.mMinMax[0] && mMinMax[1] == inRHS.mMinMax[1];
}

template <typename T, std::size_t N>
bool AAHyperBox<T, N>::operator<(const AAHyperBox& inRHS) const
{
  return mMinMax[0] < inRHS.mMinMax[0] && mMinMax[1] < inRHS.mMinMax[1];
}

template <typename T, std::size_t N>
bool AAHyperBox<T, N>::operator<=(const AAHyperBox& inRHS) const
{
  return mMinMax[0] <= inRHS.mMinMax[0] && mMinMax[1] <= inRHS.mMinMax[1];
}

template <typename T, std::size_t N>
bool AAHyperBox<T, N>::operator>(const AAHyperBox& inRHS) const
{
  return mMinMax[0] > inRHS.mMinMax[0] && mMinMax[1] > inRHS.mMinMax[1];
}

template <typename T, std::size_t N>
bool AAHyperBox<T, N>::operator>=(const AAHyperBox& inRHS) const
{
  return mMinMax[0] >= inRHS.mMinMax[0] && mMinMax[1] >= inRHS.mMinMax[1];
}

template <typename T, std::size_t N>
template <typename TOther>
void AAHyperBox<T, N>::Wrap(const TOther& inThingToBound)
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
AAHyperBox<T, N>::GPointsIterator<IsConst>::GPointsIterator(AAHyperBoxType& ioHyperBox, const std::size_t inBeginIndex)
    : mAAHyperBox { ioHyperBox }, mCurrentIndex(inBeginIndex)
{
}

template <typename T, std::size_t N>
template <bool IsConst>
typename AAHyperBox<T, N>::template GPointsIterator<IsConst>& AAHyperBox<T, N>::GPointsIterator<IsConst>::operator++()
{
  EXPECTS((mCurrentIndex < AAHyperBox<T, N>::NumPoints));
  ++mCurrentIndex;
  return *this;
}

template <typename T, std::size_t N>
template <bool IsConst>
typename AAHyperBox<T, N>::template GPointsIterator<IsConst>::VecType
    AAHyperBox<T, N>::GPointsIterator<IsConst>::operator*() const
{
  const auto current_binary_index = MakeBinaryIndex<N, T>(mCurrentIndex);
  const auto point = mAAHyperBox.GetMin() + mAAHyperBox.GetSize() * current_binary_index;
  return point;
}

template <typename T>
constexpr auto BoundingAAHyperBox(const T& inThingToBound)
{
  if constexpr (IsVec_v<T>)
  {
    using BoundingAAHyperBoxType = AAHyperBox<ValueType_t<T>, NumDimensions_v<T>>;
    BoundingAAHyperBoxType bounding_aa_hyper_box;
    bounding_aa_hyper_box.Wrap(inThingToBound);
    return bounding_aa_hyper_box;
  }
  else if constexpr (IsAAHyperBox_v<T>)
  {
    return inThingToBound;
  }
  else
  {
    using BoundingAAHyperBoxType = decltype(BoundingAAHyperBox(*inThingToBound.begin()));
    BoundingAAHyperBoxType bounding_aa_hyper_box;
    if constexpr (IsAAHyperBox_v<T>) // Efficiency overloads
    {
      bounding_aa_hyper_box.Wrap(inThingToBound.GetMin());
      bounding_aa_hyper_box.Wrap(inThingToBound.GetMax());
    }
    else
    {
      for (const auto subthing_to_bound : inThingToBound)
      { bounding_aa_hyper_box.Wrap(BoundingAAHyperBox(subthing_to_bound)); }
    }
    return bounding_aa_hyper_box;
  }
}
}