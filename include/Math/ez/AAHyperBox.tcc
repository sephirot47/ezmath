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
    for (const auto& thing_point : MakePointsRange(inThingToBound)) { Wrap(thing_point); }
  }
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
  for (int i = 0; i < N; ++i)
  {
    const auto hyper_sphere_center_local_i = (Center(inHyperSphere)[i] - inAAHyperBox.GetMin()[i]);
    const auto size_i = (inAAHyperBox.GetMax()[i] - inAAHyperBox.GetMin()[i]);

    if (hyper_sphere_center_local_i - inHyperSphere.GetRadius() < static_cast<T>(0))
      return false;

    if (hyper_sphere_center_local_i + inHyperSphere.GetRadius() > size_i)
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
  return std::all_of(MakePointsBegin(inHyperBox), MakePointsEnd(inHyperBox), [&](const auto& in_hyper_box_point) {
    return Contains(inAAHyperBox, in_hyper_box_point);
  });
}

template <typename T, std::size_t N>
bool Contains(const AAHyperBox<T, N>& inAAHyperBox, const Capsule<T, N>& inCapsule)
{
  return Contains(inAAHyperBox, HyperSphere<T, N> { inCapsule.GetOrigin(), inCapsule.GetRadius() })
      && Contains(inAAHyperBox, HyperSphere<T, N> { inCapsule.GetDestiny(), inCapsule.GetRadius() });
}

template <typename T, std::size_t N>
bool Contains(const AAHyperBox<T, N>& inAAHyperBox, const Triangle<T, N>& inTriangle)
{
  return std::all_of(MakePointsBegin(inTriangle), MakePointsEnd(inTriangle), [&](const auto& in_triangle_point) {
    return Contains(inAAHyperBox, in_triangle_point);
  });
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
  std::array<Vec<T, N>, AAHyperBox<T, N>::NumPoints> points;
  std::copy(MakePointsBegin(inAAHyperBox), MakePointsEnd(inAAHyperBox), points.begin());
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
constexpr Vec<T, N> ClosestPoint(const AAHyperBox<T, N>& inAAHyperBox, const HyperSphere<T, N>& inHyperSphere)
{
  return ClosestPoint(inAAHyperBox, Center(inHyperSphere));
}

template <typename T, std::size_t N>
constexpr Vec<T, N> ClosestPoint(const AAHyperBox<T, N>& inAAHyperBoxLHS, const AAHyperBox<T, N>& inAAHyperBoxRHS)
{
  auto closest_point_in_lhs = Max<Vec<T, N>>();
  auto closest_sq_distance = Max<T>();
  for (const auto rhs_aa_hyper_box_segment : MakeSegmentsRange(inAAHyperBoxRHS))
  {
    const auto closest_point = ClosestPoint(inAAHyperBoxLHS, rhs_aa_hyper_box_segment);
    const auto sq_distance = SqDistance(rhs_aa_hyper_box_segment, closest_point);
    if (sq_distance < closest_sq_distance)
    {
      closest_point_in_lhs = closest_point;
      closest_sq_distance = sq_distance;
    }
  }
  return closest_point_in_lhs;
}

template <typename T, std::size_t N>
constexpr Vec<T, N> ClosestPoint(const AAHyperBox<T, N>& inAAHyperBox, const HyperBox<T, N>& inHyperBox)
{
  auto closest_point_in_aa_hyper_box = Max<Vec<T, N>>();
  auto closest_sq_distance = Max<T>();
  for (const auto hyper_box_segment : MakeSegmentsRange(inHyperBox))
  {
    const auto closest_point = ClosestPoint(inAAHyperBox, hyper_box_segment);
    const auto sq_distance = SqDistance(hyper_box_segment, closest_point);
    if (sq_distance < closest_sq_distance)
    {
      closest_point_in_aa_hyper_box = closest_point;
      closest_sq_distance = sq_distance;
    }
  }
  return closest_point_in_aa_hyper_box;
}

template <typename T, std::size_t N>
constexpr Vec<T, N> ClosestPoint(const AAHyperBox<T, N>& inAAHyperBox, const Capsule<T, N>& inCapsule)
{
  return ClosestPoint(inAAHyperBox, inCapsule.GetSegment());
}

template <typename T, std::size_t N>
constexpr Vec<T, N> ClosestPoint(const AAHyperBox<T, N>& inAAHyperBox, const Triangle<T, N>& inTriangle)
{
  return ClosestPoint(inAAHyperBox, ClosestPoint(inTriangle, inAAHyperBox));
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

// Points iterator
template <typename T, std::size_t N>
PointsIteratorSpecialization<AAHyperBox<T, N>>::PointsIteratorSpecialization(const AAHyperBox<T, N>& inAAHyperBox)
    : mAAHyperBoxSize { inAAHyperBox.GetSize() }
{
}

template <typename T, std::size_t N>
Vec<T, N> PointsIteratorSpecialization<AAHyperBox<T, N>>::GetPoint(const AAHyperBox<T, N>& inAAHyperBox,
    const std::size_t inPointIndex) const
{
  return inAAHyperBox.GetMin() + MakeBinaryIndex<N, T>(inPointIndex) * mAAHyperBoxSize;
}

// Segments iterator
template <typename T, std::size_t N>
SegmentsIteratorSpecialization<AAHyperBox<T, N>>::SegmentsIteratorSpecialization(const AAHyperBox<T, N>& inAAHyperBox)
    : mAAHyperBoxSize { inAAHyperBox.GetSize() }
{
}

template <typename T, std::size_t N>
Segment<T, N> SegmentsIteratorSpecialization<AAHyperBox<T, N>>::GetSegment(const AAHyperBox<T, N>& inAAHyperBox,
    const std::size_t inSegmentIndex) const
{
  Vec<T, N> bin_index_0, bin_index_1;
  {
    const auto dimension = (inSegmentIndex / (AAHyperBox<T, N>::NumPoints / 2));
    const auto point_i = (inSegmentIndex % (AAHyperBox<T, N>::NumPoints / 2));

    int j = 0;
    const auto low_bin_index_0 = MakeBinaryIndex<N - 1, T>(point_i);
    for (std::size_t i = 0; i < N; ++i) { bin_index_0[i] = ((i == dimension) ? 0 : low_bin_index_0[j++]); }

    bin_index_1 = bin_index_0;
    ++bin_index_1[dimension];

    assert(IsBetween(bin_index_0, Zero<Vec<T, N>>(), One<Vec<T, N>>()));
    assert(IsBetween(bin_index_1, Zero<Vec<T, N>>(), One<Vec<T, N>>()));
  }

  const auto aa_hyper_box_point_0 = (inAAHyperBox.GetMin() + mAAHyperBoxSize * bin_index_0);
  const auto aa_hyper_box_point_1 = (inAAHyperBox.GetMin() + mAAHyperBoxSize * bin_index_1);
  return Segment<T, N> { aa_hyper_box_point_0, aa_hyper_box_point_1 };
}
}