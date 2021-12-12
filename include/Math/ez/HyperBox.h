#pragma once

#include <ez/IntersectMode.h>
#include <ez/MathMultiComponent.h>
#include <ez/MathTypeTraits.h>
#include <ez/PointsIterator.h>
#include <ez/SegmentsIterator.h>
#include <ez/Quat.h>
#include <ez/Vec.h>

namespace ez
{

// For AARect, AABox...
template <typename T, std::size_t N>
class HyperBox final
{
public:
  using ValueType = T;
  static constexpr auto NumPoints = static_cast<std::size_t>(Pow(static_cast<std::size_t>(2), N));
  static constexpr auto NumComponents = N;
  static constexpr auto NumDimensions = N;

  HyperBox();
  HyperBox(const Vec<T, N>& inCenter, const Vec<T, N>& inExtents, const RotationType_t<T, N>& inOrientation);
  HyperBox(const HyperBox&) = default;
  HyperBox& operator=(const HyperBox&) = default;
  HyperBox(HyperBox&&) = default;
  HyperBox& operator=(HyperBox&&) = default;

  template <typename TOther>
  explicit HyperBox(const HyperBox<TOther, N>& inHyperBox);

  ~HyperBox() = default;

  void SetCenter(const Vec<T, N>& inCenter) { mCenter = inCenter; }
  void SetExtents(const Vec<T, N>& inExtents) { mExtents = inExtents; }
  void SetSize(const Vec<T, N>& inSize) { SetExtents(inSize / static_cast<T>(2)); }
  void SetOrientation(const RotationType_t<T, N>& inOrientation) { mOrientation = inOrientation; }

  const Vec<T, N>& GetCenter() const { return mCenter; }
  const Vec<T, N>& GetExtents() const { return mExtents; }
  Vec<T, N> GetSize() const { return mExtents * static_cast<T>(2); }
  const RotationType_t<T, N>& GetOrientation() const { return mOrientation; }

  bool operator==(const HyperBox& inHyperBox) const;
  bool operator!=(const HyperBox& inHyperBox) const { return !(*this == inHyperBox); }
  HyperBox operator+(const Vec<T, N>& inTranslation);
  HyperBox operator-(const Vec<T, N>& inTranslation) { return (*this) + (-inTranslation); }
  HyperBox& operator+=(const Vec<T, N>& inTranslation);
  HyperBox& operator-=(const Vec<T, N>& inTranslation) { return ((*this) += (-inTranslation)); }
  HyperBox operator*(const Vec<T, N>& inScale);
  HyperBox operator/(const Vec<T, N>& inScale) { return (*this) * (static_cast<T>(1) / inScale); }
  HyperBox& operator*=(const Vec<T, N>& inScale);
  HyperBox& operator/=(const Vec<T, N>& inScale) { return ((*this) *= (static_cast<T>(1) / inScale)); }

private:
  Vec<T, N> mCenter = Zero<Vec<T, N>>();
  Vec<T, N> mExtents = One<Vec<T, N>>();
  RotationType_t<T, N> mOrientation;
};

// Traits
template <typename T, std::size_t N>
struct IsHyperBox<HyperBox<T, N>> : std::true_type
{
};

// Intersect
template <EIntersectMode TIntersectMode, typename T, std::size_t N>
auto Intersect(const HyperBox<T, N>& inHyperBox, const Vec<T, N>& inPoint);

template <EIntersectMode TIntersectMode, typename T, std::size_t N>
auto Intersect(const HyperBox<T, N>& inHyperBoxLHS, const HyperBox<T, N>& inHyperBoxRHS);

template <EIntersectMode TIntersectMode, typename T, std::size_t N>
auto Intersect(const HyperBox<T, N>& inHyperBox, const AAHyperBox<T, N>& inAAHyperBox);

template <EIntersectMode TIntersectMode, typename T, std::size_t N, typename TPrimitive>
auto Intersect(const HyperBox<T, N>& inHyperBox, const TPrimitive& inPrimitive);

// Contains
template <typename T, std::size_t N>
bool Contains(const HyperBox<T, N>& inHyperBox, const AAHyperBox<T, N>& inAAHyperBox);

template <typename T, std::size_t N, typename TPrimitive>
bool Contains(const HyperBox<T, N>& inHyperBox, const TPrimitive& inPrimitive);

// ClosestPoint
template <typename T, std::size_t N>
constexpr Vec<T, N> ClosestPoint(const HyperBox<T, N>& inHyperBox, const AAHyperBox<T, N>& inAAHyperBox);

template <typename T, std::size_t N, typename TPrimitive>
constexpr Vec<T, N> ClosestPoint(const HyperBox<T, N>& inHyperBox, const TPrimitive& inPrimitive);

template <typename T, std::size_t N>
auto GetSATNormals(const HyperBox<T, N>& inHyperBox);

template <typename T, std::size_t N>
auto GetSATEdges(const HyperBox<T, N>& inHyperBox);

template <typename T, std::size_t N>
auto GetSATPoints(const HyperBox<T, N>& inHyperBox);

template <typename T, std::size_t N>
constexpr HyperBox<T, N> Translated(const HyperBox<T, N>& inHyperBox, const Vec<T, N>& inTranslation);

template <typename T, std::size_t N>
constexpr HyperBox<T, N> Rotated(const HyperBox<T, N>& inHyperBox, const RotationType_t<T, N>& inRotation);

template <typename T, std::size_t N>
constexpr Vec<T, N> Center(const HyperBox<T, N>& inHyperBox);

template <typename T, std::size_t N>
constexpr RotationType_t<T, N> Orientation(const HyperBox<T, N>& inHyperBox);

// Points iterator
template <typename T, std::size_t N>
struct PointsIteratorSpecialization<HyperBox<T, N>>
{
  static constexpr std::size_t NumPoints = HyperBox<T, N>::NumPoints;
  PointsIteratorSpecialization(const HyperBox<T, N> &) {}
  Vec<T, N> GetPoint(const HyperBox<T, N>& inHyperBox, const std::size_t inPointIndex) const;
};

// Segments iterator
template <typename T, std::size_t N>
struct SegmentsIteratorSpecialization<HyperBox<T, N>>
{
  static constexpr std::size_t NumSegments = (N * (HyperBox<T, N>::NumPoints / 2));
  SegmentsIteratorSpecialization(const HyperBox<T, N>& inHyperBox) {}
  Segment<T, N> GetSegment(const HyperBox<T, N>& inHyperBox, const std::size_t inSegmentIndex) const;
};
}

#include "ez/HyperBox.tcc"