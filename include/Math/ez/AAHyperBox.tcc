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
template <typename TOther>
AAHyperBox<T, N>::AAHyperBox(const AAHyperBox<TOther, N>& inRHS)
    : AAHyperBox(Vec<T, N>(inRHS.GetMin()), Vec<T, N>(inRHS.GetMax()))
{
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
AAHyperBox<T, N> AAHyperBox<T, N>::operator+(const Vec<T, N>& inRHS)
{
  return AAHyperBox<T, N>(mMinMax[0] + inRHS, mMinMax[1] + inRHS);
}

template <typename T, std::size_t N>
AAHyperBox<T, N>& AAHyperBox<T, N>::operator+=(const Vec<T, N>& inRHS)
{
  *this = (*this + inRHS);
  return *this;
}

template <typename T, std::size_t N>
AAHyperBox<T, N> AAHyperBox<T, N>::operator*(const Vec<T, N>& inRHS)
{
  return AAHyperBox<T, N> { mMinMax[0] * inRHS, mMinMax[1] * inRHS };
}

template <typename T, std::size_t N>
AAHyperBox<T, N>& AAHyperBox<T, N>::operator*=(const Vec<T, N>& inRHS)
{
  *this = (*this) * inRHS;
  return *this;
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

template <typename T, std::size_t N>
constexpr auto BoundingAAHyperBox(const AAHyperBox<T, N>& inAAHyperBox)
{
  return inAAHyperBox;
}

template <typename T, std::size_t N>
constexpr auto BoundingAAHyperBoxTransformed(const AAHyperBox<T, N>& inAAHyperBox,
    const Transformation<T, N>& inTransformation)
{
  AAHyperBox<T, N> hyper_box;
  std::for_each(inAAHyperBox.cbegin(), inAAHyperBox.cend(), [&](const auto& inPoint) {
    hyper_box.Wrap(Transformed(inPoint, inTransformation));
  });
  return hyper_box;
}

template <typename T, std::size_t N>
constexpr auto BoundingAAHyperBoxInverseTransformed(const AAHyperBox<T, N>& inAAHyperBox,
    const Transformation<T, N>& inTransformation)
{
  AAHyperBox<T, N> hyper_box;
  std::for_each(inAAHyperBox.cbegin(), inAAHyperBox.cend(), [&](const auto& inPoint) {
    hyper_box.Wrap(InverseTransformed(inPoint, inTransformation));
  });
  return hyper_box;
}

template <typename T, std::size_t N>
void Transform(AAHyperBox<T, N>& ioAAHyperBoxToTransform, const SquareMat<T, N>& inTransformMatrix)
{
  const auto p0 = Transformed(ioAAHyperBoxToTransform.GetMin(), inTransformMatrix);
  const auto p1 = Transformed(ioAAHyperBoxToTransform.GetMax(), inTransformMatrix);
  ioAAHyperBoxToTransform.SetMinMax(Min(p0, p1), Max(p0, p1));
}

template <typename T, std::size_t N>
void Transform(AAHyperBox<T, N>& ioAAHyperBoxToTransform, const SquareMat<T, N + 1>& inTransformMatrix)
{
  const auto p0 = XYZ(Transformed(XYZ1(ioAAHyperBoxToTransform.GetMin()), inTransformMatrix));
  const auto p1 = XYZ(Transformed(XYZ1(ioAAHyperBoxToTransform.GetMax()), inTransformMatrix));
  ioAAHyperBoxToTransform.SetMinMax(Min(p0, p1), Max(p0, p1));
}

template <typename T, std::size_t N>
void Transform(AAHyperBox<T, N>& ioAAHyperBoxToTransform, const Transformation<T, N>& inTransformation)
{
  const auto p0 = inTransformation.TransformedPoint(ioAAHyperBoxToTransform.GetMin());
  const auto p1 = inTransformation.TransformedPoint(ioAAHyperBoxToTransform.GetMax());
  ioAAHyperBoxToTransform.SetMinMax(Min(p0, p1), Max(p0, p1));
}

template <typename T, std::size_t N>
void InverseTransform(AAHyperBox<T, N>& ioAAHyperBoxToTransform, const Transformation<T, N>& inTransformation)
{
  const auto p0 = inTransformation.InverseTransformedPoint(ioAAHyperBoxToTransform.GetMin());
  const auto p1 = inTransformation.InverseTransformedPoint(ioAAHyperBoxToTransform.GetMax());
  ioAAHyperBoxToTransform.SetMinMax(Min(p0, p1), Max(p0, p1));
}
}