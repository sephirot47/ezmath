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

// Intersection functions
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
  const auto& center = inHyperBox.GetCenter();
  const auto extents = inHyperBox.GetExtents();

  const auto hyper_box_orientation = inHyperBox.GetOrientation();
  std::array<Vec<T, N>, HyperBox<T, N>::NumPoints> points;
  for (uint i = 0; i < points.size(); ++i)
  {
    const auto extents_i = extents * (MakeBinaryIndex<N, T>(i) * 2 - 1);
    points[i] = center + Rotated(extents_i, hyper_box_orientation);
  }

  return points;
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
}