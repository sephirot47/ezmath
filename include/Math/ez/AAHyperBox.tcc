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

template <typename T, std::size_t N>
std::ostream& operator<<(std::ostream& ioLHS, const AAHyperBox<T, N>& inAAHyperBox)
{
  ioLHS << "(" << inAAHyperBox.GetMin() << ", " << inAAHyperBox.GetMax() << ")";
  return ioLHS;
}

template <typename T, std::size_t N>
const auto MakeAAHyperBoxFrom2Points(const Vec<T, N>& inPoint1, const Vec<T, N>& inPoint2)
{
  return AAHyperBox<T, N>(Min(inPoint1, inPoint2), Max(inPoint1, inPoint2));
}

template <typename T, std::size_t N>
const auto MakeAAHyperBoxFromCenterHalfSize(const Vec<T, N>& inCenter, const Vec<T, N>& inHalfSize)
{
  return MakeAAHyperBoxFrom2Points(inCenter - inHalfSize, inCenter + inHalfSize);
}

template <typename T, std::size_t N>
const auto MakeAAHyperBoxFromCenterSize(const Vec<T, N>& inCenter, const Vec<T, N>& inSize)
{
  return MakeAAHyperBoxFromCenterHalfSize(inCenter, (inSize / static_cast<T>(2)));
}

template <typename T, std::size_t N>
const auto MakeAAHyperBoxFromMinSize(const Vec<T, N>& inMin, const Vec<T, N>& inSize)
{
  return MakeAAHyperBoxFrom2Points(inMin, inMin + inSize);
}

template <typename T, std::size_t N>
const auto MakeAAHyperBoxFromMinMax(const Vec<T, N>& inMin, const Vec<T, N>& inMax)
{
  return AAHyperBox<T, N>(inMin, inMax);
}

template <typename T, std::size_t N>
const auto MakeAAHyperBoxFromAAHyperCube(const AAHyperCube<T, N>& inAAHyperCube)
{
  return MakeAAHyperBoxFromMinSize(inAAHyperCube.GetMin(), All<Vec<T, N>>(inAAHyperCube.GetSize()));
}

// Intersection functions
template <EIntersectMode TIntersectMode, typename T, std::size_t N>
auto Intersect(const AAHyperBox<T, N>& inLHS, const AAHyperBox<T, N>& inRHS)
{
  static_assert(TIntersectMode == EIntersectMode::ONLY_CHECK, "Unsupported EIntersectMode.");
  const auto some_lhs_min_coord_greater_than_max = !(inLHS.GetMin() <= inRHS.GetMax());
  const auto some_lhs_max_coord_less_than_max = !(inLHS.GetMax() >= inRHS.GetMin());
  const auto do_not_intersect = (some_lhs_min_coord_greater_than_max || some_lhs_max_coord_less_than_max);
  return !do_not_intersect;
}

template <EIntersectMode TIntersectMode, typename T, std::size_t N>
auto Intersect(const AAHyperCube<T, N>& inLHS, const AAHyperBox<T, N>& inRHS)
{
  return Intersect<TIntersectMode>(MakeAAHyperBoxFromAAHyperCube(inLHS), inRHS);
}

template <EIntersectMode TIntersectMode, typename T, std::size_t N>
auto Intersect(const AAHyperBox<T, N>& inLHS, const AAHyperCube<T, N>& inRHS)
{
  return Intersect<TIntersectMode>(inRHS, inLHS);
}

template <typename T, std::size_t N>
bool Contains(const AAHyperBox<T, N>& inAAHyperBox, const Vec<T, N>& inPoint)
{
  return inPoint >= inAAHyperBox.GetMin() && inPoint <= inAAHyperBox.GetMax();
}

template <typename T, std::size_t N>
bool Contains(const AAHyperBox<T, N>& inAAHyperBoxContainer, const AAHyperBox<T, N>& inAAHyperBoxContainee)
{
  return inAAHyperBoxContainee.GetMin() >= inAAHyperBoxContainer.GetMin()
      && inAAHyperBoxContainee.GetMax() <= inAAHyperBoxContainer.GetMax();
}

template <typename T, std::size_t N>
bool Contains(const AAHyperBox<T, N>& inAAHyperBoxContainer, const AAHyperCube<T, N>& inAAHyperCubeContainee)
{
  return Contains(inAAHyperBoxContainer, MakeAAHyperBoxFromAAHyperCube(inAAHyperCubeContainee));
}

template <typename T, std::size_t N>
bool Contains(const AAHyperCube<T, N>& inAAHyperCubeContainer, const AAHyperBox<T, N>& inAAHyperBoxContainee)
{
  return Contains(MakeAAHyperBoxFromAAHyperCube(inAAHyperCubeContainer), inAAHyperBoxContainee);
}

template <typename T>
constexpr auto BoundingAAHyperBox(const T& inThingToBound)
{
  if constexpr (IsVec_v<T>) {}
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
constexpr Vec<T, N> Center(const AAHyperBox<T, N>& inAAHyperBox)
{
  return (inAAHyperBox.GetMin() + inAAHyperBox.GetMax()) / static_cast<T>(2);
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
  AAHyperBox<T, N> transformed_hyper_box;
  std::for_each(inAAHyperBox.cbegin(), inAAHyperBox.cend(), [&](const auto& inPoint) {
    transformed_hyper_box.Wrap(Transformed(inPoint, inTransformation));
  });
  return transformed_hyper_box;
}

template <typename T, std::size_t N>
constexpr auto BoundingAAHyperBoxInverseTransformed(const AAHyperBox<T, N>& inAAHyperBox,
    const Transformation<T, N>& inTransformation)
{
  AAHyperBox<T, N> transformed_hyper_box;
  std::for_each(inAAHyperBox.cbegin(), inAAHyperBox.cend(), [&](const auto& inPoint) {
    transformed_hyper_box.Wrap(InverseTransformed(inPoint, inTransformation));
  });
  return transformed_hyper_box;
}

template <typename T, std::size_t N>
void Transform(AAHyperBox<T, N>& ioAAHyperBoxToTransform, const SquareMat<T, N>& inTransformMatrix)
{
  AAHyperBox<T, N> transformed_hyper_box;
  std::for_each(ioAAHyperBoxToTransform.cbegin(), ioAAHyperBoxToTransform.cend(), [&](const auto& inPoint) {
    transformed_hyper_box.Wrap(InverseTransformed(inPoint, inTransformMatrix));
  });
  ioAAHyperBoxToTransform = transformed_hyper_box;
}

template <typename T, std::size_t N>
void Transform(AAHyperBox<T, N>& ioAAHyperBoxToTransform, const SquareMat<T, N + 1>& inTransformMatrix)
{
  AAHyperBox<T, N> transformed_hyper_box;
  std::for_each(ioAAHyperBoxToTransform.cbegin(), ioAAHyperBoxToTransform.cend(), [&](const auto& inPoint) {
    transformed_hyper_box.Wrap(Transformed(inPoint, inTransformMatrix));
  });
  ioAAHyperBoxToTransform = transformed_hyper_box;
}

template <typename T, std::size_t N>
void Transform(AAHyperBox<T, N>& ioAAHyperBoxToTransform, const Transformation<T, N>& inTransformation)
{
  AAHyperBox<T, N> transformed_hyper_box;
  std::for_each(ioAAHyperBoxToTransform.cbegin(), ioAAHyperBoxToTransform.cend(), [&](const auto& inPoint) {
    transformed_hyper_box.Wrap(Transformed(inPoint, inTransformation));
  });
  ioAAHyperBoxToTransform = transformed_hyper_box;
}

template <typename T, std::size_t N>
void InverseTransform(AAHyperBox<T, N>& ioAAHyperBoxToTransform, const Transformation<T, N>& inTransformation)
{
  AAHyperBox<T, N> transformed_hyper_box;
  std::for_each(ioAAHyperBoxToTransform.cbegin(), ioAAHyperBoxToTransform.cend(), [&](const auto& inPoint) {
    transformed_hyper_box.Wrap(InverseTransformed(inPoint, inTransformation));
  });
  ioAAHyperBoxToTransform = transformed_hyper_box;
}
}