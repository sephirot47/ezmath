#include "ez/BinaryIndex.h"
#include "ez/HyperBox.h"
#include "ez/Macros.h"
#include "ez/MathTypeTraits.h"
#include "ez/SATIntersect.h"

namespace ez
{

template <typename T, std::size_t N>
HyperBox<T, N>::HyperBox()
{
}

template <typename T, std::size_t N>
HyperBox<T, N>::HyperBox(const Vec<T, N>& inCenter,
    const Vec<T, N>& inExtents,
    const RotationType_t<T, N>& inOrientation)
    : mCenter { inCenter }, mExtents { inExtents }, mOrientation { inOrientation }
{
}

template <typename T, std::size_t N>
template <typename TOther>
HyperBox<T, N>::HyperBox(const HyperBox<TOther, N>& inHyperBox)
    : HyperBox(Vec<T, N>(inHyperBox.GetCenter()),
        Vec<T, N>(inHyperBox.GetExtents()),
        RotationType_t<T, N>(inHyperBox.GetOrientation()))
{
}

template <typename T, std::size_t N>
bool HyperBox<T, N>::operator==(const HyperBox& inHyperBox) const
{
  return mCenter == inHyperBox.mCenter && mExtents == inHyperBox.mExtents && mOrientation == inHyperBox.mOrientation;
}

template <typename T, std::size_t N>
HyperBox<T, N> HyperBox<T, N>::operator+(const Vec<T, N>& inTranslation)
{
  return HyperBox<T, N> { mCenter + inTranslation, mExtents, mOrientation };
}

template <typename T, std::size_t N>
HyperBox<T, N>& HyperBox<T, N>::operator+=(const Vec<T, N>& inTranslation)
{
  *this = (*this + inTranslation);
  return *this;
}

template <typename T, std::size_t N>
HyperBox<T, N> HyperBox<T, N>::operator*(const Vec<T, N>& inScale)
{
  return HyperBox<T, N> { mCenter, mExtents * inScale, mOrientation };
}

template <typename T, std::size_t N>
HyperBox<T, N>& HyperBox<T, N>::operator*=(const Vec<T, N>& inScale)
{
  *this = (*this) * inScale;
  return *this;
}

template <typename T, std::size_t N>
std::ostream& operator<<(std::ostream& ioLHS, const HyperBox<T, N>& inHyperBox)
{
  ioLHS << "(" << inHyperBox.GetCenter() << ", " << inHyperBox.GetSize() << ", " << inHyperBox.GetOrientation() << ")";
  return ioLHS;
}

// Intersect
template <EIntersectMode TIntersectMode, typename T, std::size_t N>
auto Intersect(const HyperBox<T, N>& inHyperBox, const Vec<T, N>& inPoint)
{
  return Intersect<TIntersectMode>(inPoint, inHyperBox);
}

template <EIntersectMode TIntersectMode, typename T, std::size_t N>
auto Intersect(const HyperBox<T, N>& inHyperBoxLHS, const HyperBox<T, N>& inHyperBoxRHS)
{
  static_assert(TIntersectMode == EIntersectMode::ONLY_CHECK, "Unsupported EIntersectMode.");
  return IntersectCheckSAT(inHyperBoxLHS, inHyperBoxRHS);
}

template <EIntersectMode TIntersectMode, typename T, std::size_t N>
auto Intersect(const HyperBox<T, N>& inHyperBox, const AAHyperBox<T, N>& inAAHyperBox)
{
  return Intersect<TIntersectMode>(inAAHyperBox, inHyperBox);
}

template <EIntersectMode TIntersectMode, typename T, std::size_t N, typename TPrimitive>
auto Intersect(const HyperBox<T, N>& inHyperBox, const TPrimitive& inPrimitive)
{
  const auto hyper_box_orientation_inv = -Orientation(inHyperBox);
  const auto local_primitive = Rotated(inPrimitive, hyper_box_orientation_inv);
  const auto aa_hyper_box_center = Rotated(Center(inHyperBox), hyper_box_orientation_inv);
  const auto aa_hyper_box = MakeAAHyperBoxFromCenterHalfSize(aa_hyper_box_center, inHyperBox.GetExtents());
  return Intersect<TIntersectMode>(aa_hyper_box, local_primitive);
}

// Contains
template <typename T, std::size_t N>
bool Contains(const HyperBox<T, N>& inHyperBox, const AAHyperBox<T, N>& inAAHyperBox)
{
  return std::all_of(MakePointsBegin(inAAHyperBox),
      MakePointsEnd(inAAHyperBox),
      [&](const auto& in_aa_hyper_box_point) { return Contains(inHyperBox, in_aa_hyper_box_point); });
}

template <typename T, std::size_t N, typename TPrimitive>
bool Contains(const HyperBox<T, N>& inHyperBox, const TPrimitive& inPrimitive)
{
  const auto hyper_box_orientation_inv = -Orientation(inHyperBox);
  const auto local_primitive = Rotated(inPrimitive, hyper_box_orientation_inv);
  const auto aa_hyper_box = MakeAAHyperBoxFromCenterHalfSize(Rotated(inHyperBox.GetCenter(), hyper_box_orientation_inv),
      inHyperBox.GetExtents());
  return Contains(aa_hyper_box, local_primitive);
}

template <typename T, std::size_t N>
constexpr Vec<T, N> ClosestPoint(const HyperBox<T, N>& inHyperBox, const AAHyperBox<T, N>& inAAHyperBox)
{
  return ClosestPoint(inHyperBox, ClosestPoint(inAAHyperBox, inHyperBox));
}

template <typename T, std::size_t N, typename TPrimitive>
constexpr Vec<T, N> ClosestPoint(const HyperBox<T, N>& inHyperBox, const TPrimitive& inPrimitive)
{
  const auto hyper_box_orientation_inv = -Orientation(inHyperBox);
  const auto local_primitive = Rotated(inPrimitive, hyper_box_orientation_inv);
  const auto aa_hyper_box = MakeAAHyperBoxFromCenterHalfSize(Rotated(inHyperBox.GetCenter(), hyper_box_orientation_inv),
      inHyperBox.GetExtents());
  const auto closest_point_local = ClosestPoint(aa_hyper_box, local_primitive);
  const auto closest_point = Rotated(closest_point_local, -hyper_box_orientation_inv);
  return closest_point;
}

template <typename T, std::size_t N>
auto GetSATNormals(const HyperBox<T, N>& inHyperBox)
{
  const auto hyper_box_orientation = inHyperBox.GetOrientation();
  if constexpr (N == 3)
    return std::array { Rotated(Right<Vec<T, N>>(), hyper_box_orientation),
      Rotated(Up<Vec<T, N>>(), hyper_box_orientation),
      Rotated(Forward<Vec<T, N>>(), hyper_box_orientation) };
  else if constexpr (N == 2)
    return std::array { Rotated(Right<Vec<T, N>>(), hyper_box_orientation),
      Rotated(Up<Vec<T, N>>(), hyper_box_orientation) };
}

template <typename T, std::size_t N>
auto GetSATEdges(const HyperBox<T, N>& inHyperBox)
{
  const auto hyper_box_orientation = inHyperBox.GetOrientation();
  if constexpr (N == 3)
    return std::array { Rotated(Right<Vec<T, N>>(), hyper_box_orientation),
      Rotated(Up<Vec<T, N>>(), hyper_box_orientation),
      Rotated(Forward<Vec<T, N>>(), hyper_box_orientation) };
  else if constexpr (N == 2)
    return std::array { Rotated(Right<Vec<T, N>>(), hyper_box_orientation),
      Rotated(Up<Vec<T, N>>(), hyper_box_orientation) };
}

template <typename T, std::size_t N>
auto GetSATPoints(const HyperBox<T, N>& inHyperBox)
{
  std::array<Vec<T, N>, HyperBox<T, N>::NumPoints> points;
  std::copy(MakePointsBegin(inHyperBox), MakePointsEnd(inHyperBox), points.begin());
  return points;
}

template <typename T, std::size_t N>
constexpr HyperBox<T, N> Translated(const HyperBox<T, N>& inHyperBox, const Vec<T, N>& inTranslation)
{
  return HyperBox<T, N> { inHyperBox.GetCenter() + inTranslation,
    inHyperBox.GetExtents(),
    inHyperBox.GetOrientation() };
}

template <typename T, std::size_t N>
constexpr HyperBox<T, N> Rotated(const HyperBox<T, N>& inHyperBox, const RotationType_t<T, N>& inRotation)
{
  return HyperBox<T, N> { Rotated(inHyperBox.GetCenter(), inRotation),
    inHyperBox.GetExtents(),
    Rotated(inHyperBox.GetOrientation(), inRotation) };
}

template <typename T, std::size_t N>
constexpr Vec<T, N> Center(const HyperBox<T, N>& inHyperBox)
{
  return inHyperBox.GetCenter();
}

template <typename T, std::size_t N>
constexpr RotationType_t<T, N> Orientation(const HyperBox<T, N>& inHyperBox)
{
  return inHyperBox.GetOrientation();
}

// Points iterator
template <typename T, std::size_t N>
Vec<T, N> PointsIteratorSpecialization<HyperBox<T, N>>::GetPoint(const HyperBox<T, N>& inHyperBox,
    const std::size_t inPointIndex) const
{
  const auto point_extent = (MakeBinaryIndex<N, T>(inPointIndex) * 2 - 1) * inHyperBox.GetExtents();
  return Center(inHyperBox) + Rotated(point_extent, Orientation(inHyperBox));
}

// Segment iterator
template <typename T, std::size_t N>
Segment<T, N> SegmentsIteratorSpecialization<HyperBox<T, N>>::GetSegment(const HyperBox<T, N>& inHyperBox,
    const std::size_t inSegmentIndex) const
{
  Vec<T, N> bin_index_0, bin_index_1;
  {
    const auto dimension = (inSegmentIndex / (HyperBox<T, N>::NumPoints / 2));
    const auto point_i = (inSegmentIndex % (HyperBox<T, N>::NumPoints / 2));

    int j = 0;
    const auto low_bin_index_0 = MakeBinaryIndex<N - 1, T>(point_i);
    for (std::size_t i = 0; i < N; ++i) { bin_index_0[i] = ((i == dimension) ? 0 : low_bin_index_0[j++]); }

    bin_index_1 = bin_index_0;
    ++bin_index_1[dimension];

    bin_index_0 = bin_index_0 * 2 - 1;
    bin_index_1 = bin_index_1 * 2 - 1;

    assert(IsBetween(bin_index_0, -One<Vec<T, N>>(), One<Vec<T, N>>()));
    assert(IsBetween(bin_index_1, -One<Vec<T, N>>(), One<Vec<T, N>>()));
  }

  const auto rotated_extents_0 = Rotated(inHyperBox.GetExtents() * bin_index_0, Orientation(inHyperBox));
  const auto rotated_extents_1 = Rotated(inHyperBox.GetExtents() * bin_index_1, Orientation(inHyperBox));
  const auto hyper_box_point_0 = (Center(inHyperBox) + rotated_extents_0);
  const auto hyper_box_point_1 = (Center(inHyperBox) + rotated_extents_1);
  return Segment<T, N> { hyper_box_point_0, hyper_box_point_1 };
}
}