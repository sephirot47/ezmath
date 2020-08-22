#include "ez/AAHyperBox.h"
#include "ez/BinaryIndex.h"
#include "ez/Macros.h"
#include "ez/MathTypeTraits.h"
#include "ez/SATIntersect.h"

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

// Intersect
template <EIntersectMode TIntersectMode, typename T, std::size_t N>
auto Intersect(const AAHyperBox<T, N>& inAAHyperBoxLHS, const AAHyperBox<T, N>& inAAHyperBoxRHS)
{
  static_assert(TIntersectMode == EIntersectMode::ONLY_CHECK, "Unsupported EIntersectMode.");
  const auto some_lhs_min_coord_greater_than_max = !(inAAHyperBoxLHS.GetMin() <= inAAHyperBoxRHS.GetMax());
  const auto some_lhs_max_coord_less_than_max = !(inAAHyperBoxLHS.GetMax() >= inAAHyperBoxRHS.GetMin());
  const auto do_not_intersect = (some_lhs_min_coord_greater_than_max || some_lhs_max_coord_less_than_max);
  return !do_not_intersect;
}

template <EIntersectMode TIntersectMode, typename T, std::size_t N>
auto Intersect(const AAHyperBox<T, N>& inAAHyperBox, const Vec<T, N>& inPoint)
{
  return Intersect<TIntersectMode>(inPoint, inAAHyperBox);
}

template <EIntersectMode TIntersectMode, typename T, std::size_t N>
auto Intersect(const AAHyperBox<T, N>& inAAHyperBox, const Line<T, N>& inLine)
{
  return Intersect<TIntersectMode>(inLine, inAAHyperBox);
}

template <EIntersectMode TIntersectMode, typename T, std::size_t N>
auto Intersect(const AAHyperBox<T, N>& inAAHyperBox, const Ray<T, N>& inRay)
{
  return Intersect<TIntersectMode>(inRay, inAAHyperBox);
}

template <EIntersectMode TIntersectMode, typename T, std::size_t N>
auto Intersect(const AAHyperBox<T, N>& inAAHyperBox, const Segment<T, N>& inSegment)
{
  return Intersect<TIntersectMode>(inSegment, inAAHyperBox);
}

template <EIntersectMode TIntersectMode, typename T, std::size_t N>
auto Intersect(const AAHyperBox<T, N>& inAAHyperBox, const HyperSphere<T, N>& inHyperSphere)
{
  return Intersect<TIntersectMode>(inHyperSphere, inAAHyperBox);
}

template <EIntersectMode TIntersectMode, typename T, std::size_t N>
auto Intersect(const AAHyperBox<T, N>& inAAHyperBox, const HyperBox<T, N>& inHyperBox)
{
  static_assert(TIntersectMode == EIntersectMode::ONLY_CHECK, "Unsupported EIntersectMode.");
  return IntersectCheckSAT(inAAHyperBox, inHyperBox);
}

template <EIntersectMode TIntersectMode, typename T, std::size_t N>
auto Intersect(const AAHyperBox<T, N>& inAAHyperBox, const Capsule<T, N>& inCapsule)
{
  static_assert(TIntersectMode == EIntersectMode::ONLY_CHECK, "Unsupported EIntersectMode.");
  return (SqDistance(inAAHyperBox, inCapsule.GetSegment()) <= Sq(inCapsule.GetRadius()));
}

template <EIntersectMode TIntersectMode, typename T, std::size_t N>
auto Intersect(const AAHyperBox<T, N>& inAAHyperBox, const Triangle<T, N>& inTriangle)
{
  static_assert(TIntersectMode == EIntersectMode::ONLY_CHECK, "Unsupported EIntersectMode.");
  return IntersectCheckSAT(inAAHyperBox, inTriangle);
}

// Contains
template <typename T, std::size_t N>
bool Contains(const AAHyperBox<T, N>& inAAHyperBox, const Vec<T, N>& inPoint)
{
  return inPoint >= inAAHyperBox.GetMin() && inPoint <= inAAHyperBox.GetMax();
}

template <typename T, std::size_t N>
bool Contains(const AAHyperBox<T, N>& inAAHyperBox, const Line<T, N>& inLine)
{
  return false;
}

template <typename T, std::size_t N>
bool Contains(const AAHyperBox<T, N>& inAAHyperBox, const Ray<T, N>& inRay)
{
  return false;
}

template <typename T, std::size_t N>
bool Contains(const AAHyperBox<T, N>& inAAHyperBox, const Segment<T, N>& inSegment)
{
  return Contains(inAAHyperBox, inSegment.GetOrigin()) && Contains(inAAHyperBox, inSegment.GetDestiny());
}

template <typename T, std::size_t N>
bool Contains(const AAHyperBox<T, N>& inAAHyperBox, const HyperSphere<T, N>& inHyperSphere)
{
  if (!Contains(inAAHyperBox, inHyperSphere.GetCenter()))
    return false;

  const auto aa_hyper_box_size = inAAHyperBox.GetSize();
  const auto hyper_sphere_sq_radius = Sq(inHyperSphere.GetRadius());
  for (int i = 0; i < AAHyperBox<T, N>::NumPoints; ++i)
  {
    const auto aa_hyper_box_point = inAAHyperBox.GetMin() + MakeBinaryIndex<N, T>(i) * aa_hyper_box_size;
    const auto sq_distance = SqDistance(Center(inHyperSphere), aa_hyper_box_point);
    if (sq_distance < hyper_sphere_sq_radius)
      return false;
  }
  return true;
}

template <typename T, std::size_t N>
bool Contains(const AAHyperBox<T, N>& inAAHyperBoxContainer, const AAHyperBox<T, N>& inAAHyperBoxContainee)
{
  return inAAHyperBoxContainee.GetMin() >= inAAHyperBoxContainer.GetMin()
      && inAAHyperBoxContainee.GetMax() <= inAAHyperBoxContainer.GetMax();
}

template <typename T, std::size_t N>
bool Contains(const AAHyperBox<T, N>& inAAHyperBox, const HyperBox<T, N>& inHyperBox)
{
  const auto hyper_box_center = inHyperBox.GetCenter();
  const auto hyper_box_extents = inHyperBox.GetExtents();
  const auto hyper_box_orientation = Orientation(inHyperBox);
  for (int i = 0; i < HyperBox<T, N>::NumPoints; ++i)
  {
    const auto rotated_extents = Rotated(hyper_box_extents * (MakeBinaryIndex<N, T>(i) * 2 - 1), hyper_box_orientation);
    const auto hyper_box_point = hyper_box_center + rotated_extents;
    if (!Contains(inAAHyperBox, hyper_box_point))
      return false;
  }
  return true;
}

template <typename T, std::size_t N>
bool Contains(const AAHyperBox<T, N>& inAAHyperBox, const Capsule<T, N>& inCapsule)
{
  if (!Contains(inAAHyperBox, inCapsule.GetSegment()))
    return false;

  const auto capsule_sq_radius = Sq(inCapsule.GetRadius());
  const auto aa_hyper_box_size = inAAHyperBox.GetSize();
  for (int i = 0; i < AAHyperBox<T, N>::NumPoints; ++i)
  {
    const auto aa_hyper_box_point = inAAHyperBox.GetMin() + MakeBinaryIndex<N, T>(i) * aa_hyper_box_size;
    const auto origin_sq_distance = SqDistance(inCapsule.GetOrigin(), aa_hyper_box_point);
    if (origin_sq_distance < capsule_sq_radius)
      return false;
    const auto destiny_sq_distance = SqDistance(inCapsule.GetDestiny(), aa_hyper_box_point);
    if (destiny_sq_distance < capsule_sq_radius)
      return false;
  }
  return true;
}

template <typename T, std::size_t N>
bool Contains(const AAHyperBox<T, N>& inAAHyperBox, const Triangle<T, N>& inTriangle)
{
  return Contains(inAAHyperBox, inTriangle[0]) && Contains(inAAHyperBox, inTriangle[1])
      && Contains(inAAHyperBox, inTriangle[2]);
}

template <typename T, std::size_t N>
auto GetSATNormals(const AAHyperBox<T, N>&)
{
  if constexpr (N == 3)
    return std::array { Right<Vec<T, N>>(), Up<Vec<T, N>>(), Forward<Vec<T, N>>() };
  else if constexpr (N == 2)
    return std::array { Right<Vec<T, N>>(), Up<Vec<T, N>>() };
}

template <typename T, std::size_t N>
auto GetSATEdges(const AAHyperBox<T, N>&)
{
  if constexpr (N == 3)
    return std::array { Right<Vec<T, N>>(), Up<Vec<T, N>>(), Forward<Vec<T, N>>() };
  else if constexpr (N == 2)
    return std::array { Right<Vec<T, N>>(), Up<Vec<T, N>>() };
}

template <typename T, std::size_t N>
auto GetSATPoints(const AAHyperBox<T, N>& inAAHyperBox)
{
  const auto& aabox_min = inAAHyperBox.GetMin();
  const auto aabox_size = inAAHyperBox.GetSize();

  std::array<Vec<T, N>, AAHyperBox<T, N>::NumPoints> points;
  for (uint i = 0; i < points.size(); ++i) { points[i] = aabox_min + aabox_size * MakeBinaryIndex<N, T>(i); }

  return points;
}

template <typename T, std::size_t N>
constexpr Vec<T, N> ClosestPoint(const AAHyperBox<T, N>& inAAHyperBox, const Vec<T, N>& inPoint)
{
  const auto aabox_min = inAAHyperBox.GetMin();
  const auto aabox_max = inAAHyperBox.GetMax();
  return Clamp(inPoint, aabox_min, aabox_max);
}

template <typename T, std::size_t N>
constexpr Vec<T, N> ClosestPoint(const AAHyperBox<T, N>& inAAHyperBox, const Line<T, N>& inLine)
{
  return ClosestPoint(inAAHyperBox, ClosestPoint(inLine, inAAHyperBox));
}

template <typename T, std::size_t N>
constexpr Vec<T, N> ClosestPoint(const AAHyperBox<T, N>& inAAHyperBox, const Ray<T, N>& inRay)
{
  return ClosestPoint(inAAHyperBox, ClosestPoint(inRay, inAAHyperBox));
}

template <typename T, std::size_t N>
constexpr Vec<T, N> ClosestPoint(const AAHyperBox<T, N>& inAAHyperBox, const Segment<T, N>& inSegment)
{
  return ClosestPoint(inAAHyperBox, ClosestPoint(inSegment, inAAHyperBox));
}

template <typename T, std::size_t N>
constexpr Vec<T, N> ClosestPoint(const AAHyperBox<T, N>& inAAHyperBox, const Capsule<T, N>& inCapsule)
{
  const auto closest_point_in_capsule_segment = ClosestPoint(inCapsule.GetSegment(), inAAHyperBox);
  const auto closest_point_in_aa_hyper_box = ClosestPoint(inAAHyperBox, closest_point_in_capsule_segment);
  return closest_point_in_aa_hyper_box;
}

template <typename T, std::size_t N, typename TPrimitive>
constexpr T SqDistance(const AAHyperBox<T, N>& inAAHyperBox, const TPrimitive& inPrimitive)
{
  const auto closest_point_in_aa_hyper_box = ClosestPoint(inAAHyperBox, inPrimitive);
  const auto closest_point_in_primitive = ClosestPoint(inPrimitive, closest_point_in_aa_hyper_box);
  return SqDistance(closest_point_in_aa_hyper_box, closest_point_in_primitive);
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
constexpr AAHyperBox<T, N> Translated(const AAHyperBox<T, N>& inAAHyperBox, const Vec<T, N>& inTranslation)
{
  return AAHyperBox<T, N> { inAAHyperBox.GetMin() + inTranslation, inAAHyperBox.GetMax() + inTranslation };
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