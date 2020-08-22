#pragma once

#include "ez/IntersectMode.h"
#include "ez/MathMultiComponent.h"
#include "ez/MathTypeTraits.h"
#include "ez/Vec.h"

namespace ez
{

// For AARect, AABox...
template <typename T, std::size_t N>
class AAHyperBox final
{
public:
  using ValueType = T;
  static constexpr auto NumPoints = static_cast<std::size_t>(std::pow(static_cast<std::size_t>(2), N));
  static constexpr auto NumComponents = N;
  static constexpr auto NumDimensions = N;

  AAHyperBox();
  AAHyperBox(const Vec<T, N>& inMin, const Vec<T, N>& inMax);
  AAHyperBox(const AAHyperBox&) = default;
  AAHyperBox& operator=(const AAHyperBox&) = default;
  AAHyperBox(AAHyperBox&&) = default;
  AAHyperBox& operator=(AAHyperBox&&) = default;

  template <typename TOther>
  explicit AAHyperBox(const AAHyperBox<TOther, N>& inRHS);

  ~AAHyperBox() = default;

  void SetMin(const Vec<T, N>& inMin);
  void SetMax(const Vec<T, N>& inMax);
  void SetMinMax(const Vec<T, N>& inMin, const Vec<T, N>& inMax);
  void Wrap(const Vec<T, N>& inPoint);

  template <typename TOther>
  void Wrap(const TOther& inThingToBound);

  Vec<T, N> GetCenter() const { return (GetMin() + GetMax()) / static_cast<T>(2); }
  Vec<T, N> GetSize() const { return (mMinMax[1] - mMinMax[0]); }
  const Vec<T, N>& GetMin() const { return mMinMax[0]; }
  const Vec<T, N>& GetMax() const { return mMinMax[1]; }

  bool operator==(const AAHyperBox& inRHS) const;
  bool operator!=(const AAHyperBox& inRHS) const { return !(*this == inRHS); }
  bool operator<(const AAHyperBox& inRHS) const;
  bool operator<=(const AAHyperBox& inRHS) const;
  bool operator>(const AAHyperBox& inRHS) const;
  bool operator>=(const AAHyperBox& inRHS) const;
  bool operator<(const Vec<T, N>& inRHS) const { return mMinMax[0] < inRHS && mMinMax[1] < inRHS; }
  bool operator<=(const Vec<T, N>& inRHS) const { return mMinMax[0] <= inRHS && mMinMax[1] <= inRHS; }
  bool operator>(const Vec<T, N>& inRHS) const { return mMinMax[0] > inRHS && mMinMax[1] > inRHS; }
  bool operator>=(const Vec<T, N>& inRHS) const { return mMinMax[0] >= inRHS && mMinMax[1] >= inRHS; }
  AAHyperBox operator+(const Vec<T, N>& inRHS);
  AAHyperBox operator-(const Vec<T, N>& inRHS) { return (*this) + (-inRHS); }
  AAHyperBox& operator+=(const Vec<T, N>& inRHS);
  AAHyperBox& operator-=(const Vec<T, N>& inRHS) { return ((*this) += (-inRHS)); }
  AAHyperBox operator*(const Vec<T, N>& inRHS);
  AAHyperBox operator/(const Vec<T, N>& inRHS) { return (*this) * (static_cast<T>(1) / inRHS); }
  AAHyperBox& operator*=(const Vec<T, N>& inRHS);
  AAHyperBox& operator/=(const Vec<T, N>& inRHS) { return ((*this) *= (static_cast<T>(1) / inRHS)); }

  template <bool IsConst>
  class GPointsIterator
  {
  public:
    using AAHyperBoxType = std::conditional_t<IsConst, const AAHyperBox, AAHyperBox>;
    using VecType = std::conditional_t<IsConst, const Vec<T, N>, Vec<T, N>>;

    GPointsIterator(AAHyperBoxType& ioHyperBox, const std::size_t inBeginIndex);
    GPointsIterator& operator++();
    bool operator==(const GPointsIterator& inRHS) const { return mCurrentIndex == inRHS.mCurrentIndex; }
    bool operator!=(const GPointsIterator& inRHS) const { return !(*this == inRHS); }
    VecType operator*() const;

  private:
    AAHyperBoxType& mAAHyperBox;
    std::size_t mCurrentIndex = 0;
  };
  using PointsIterator = GPointsIterator<false>;
  using PointsConstIterator = GPointsIterator<true>;

  AAHyperBox::PointsIterator begin() { return PointsIterator(*this, 0); }
  AAHyperBox::PointsIterator end() { return PointsIterator(*this, NumPoints); }
  AAHyperBox::PointsConstIterator begin() const { return cbegin(); }
  AAHyperBox::PointsConstIterator end() const { return cend(); }
  AAHyperBox::PointsConstIterator cbegin() const { return PointsConstIterator(*this, 0); }
  AAHyperBox::PointsConstIterator cend() const { return PointsConstIterator(*this, NumPoints); }

private:
  std::array<Vec<T, N>, 2> mMinMax = { Max<Vec<T, N>>(), Min<Vec<T, N>>() }; // Init with invalid
};

// Traits
template <typename T, std::size_t N>
struct IsAAHyperBox<AAHyperBox<T, N>> : std::true_type
{
};

template <typename T, std::size_t N>
std::ostream& operator<<(std::ostream& ioLHS, const AAHyperBox<T, N>& inAAHyperBox);

template <typename T, std::size_t N>
const auto MakeAAHyperBoxFrom2Points(const Vec<T, N>& inPoint1, const Vec<T, N>& inPoint2);

template <typename T, std::size_t N>
const auto MakeAAHyperBoxFromCenterHalfSize(const Vec<T, N>& inCenter, const Vec<T, N>& inHalfSize);

template <typename T, std::size_t N>
const auto MakeAAHyperBoxFromCenterSize(const Vec<T, N>& inCenter, const Vec<T, N>& inSize);

template <typename T, std::size_t N>
const auto MakeAAHyperBoxFromMinSize(const Vec<T, N>& inMin, const Vec<T, N>& inSize);

template <typename T, std::size_t N>
const auto MakeAAHyperBoxFromMinMax(const Vec<T, N>& inMin, const Vec<T, N>& inMax);

// Intersection functions
template <EIntersectMode TIntersectMode, typename T, std::size_t N>
auto Intersect(const AAHyperBox<T, N>& inAAHyperBoxLHS, const AAHyperBox<T, N>& inAAHyperBoxRHS);

template <EIntersectMode TIntersectMode, typename T, std::size_t N>
auto Intersect(const AAHyperBox<T, N>& inAAHyperBox, const Vec<T, N>& inPoint);

template <EIntersectMode TIntersectMode, typename T, std::size_t N>
auto Intersect(const AAHyperBox<T, N>& inAAHyperBox, const Line<T, N>& inLine);

template <EIntersectMode TIntersectMode, typename T, std::size_t N>
auto Intersect(const AAHyperBox<T, N>& inAAHyperBox, const Ray<T, N>& inRay);

template <EIntersectMode TIntersectMode, typename T, std::size_t N>
auto Intersect(const AAHyperBox<T, N>& inAAHyperBox, const Segment<T, N>& inSegment);

template <EIntersectMode TIntersectMode, typename T, std::size_t N>
auto Intersect(const AAHyperBox<T, N>& inAAHyperBox, const HyperSphere<T, N>& inHyperSphere);

template <EIntersectMode TIntersectMode, typename T, std::size_t N>
auto Intersect(const AAHyperBox<T, N>& inAAHyperBox, const HyperBox<T, N>& inHyperBox);

template <EIntersectMode TIntersectMode, typename T, std::size_t N>
auto Intersect(const AAHyperBox<T, N>& inAAHyperBox, const Capsule<T, N>& inCapsule);

template <EIntersectMode TIntersectMode, typename T, std::size_t N>
auto Intersect(const AAHyperBox<T, N>& inAAHyperBox, const Triangle<T, N>& inTriangle);

// Contains
template <typename T, std::size_t N>
bool Contains(const AAHyperBox<T, N>& inAAHyperBox, const Vec<T, N>& inPoint);

template <typename T, std::size_t N>
bool Contains(const AAHyperBox<T, N>& inAAHyperBox, const Line<T, N>& inLine);

template <typename T, std::size_t N>
bool Contains(const AAHyperBox<T, N>& inAAHyperBox, const Ray<T, N>& inRay);

template <typename T, std::size_t N>
bool Contains(const AAHyperBox<T, N>& inAAHyperBox, const Segment<T, N>& inSegment);

template <typename T, std::size_t N>
bool Contains(const AAHyperBox<T, N>& inAAHyperBox, const HyperSphere<T, N>& inHyperSphere);

template <typename T, std::size_t N>
bool Contains(const AAHyperBox<T, N>& inAAHyperBoxContainer, const AAHyperBox<T, N>& inAAHyperBoxContainee);

template <typename T, std::size_t N>
bool Contains(const AAHyperBox<T, N>& inAAHyperBox, const HyperBox<T, N>& inHyperBox);

template <typename T, std::size_t N>
bool Contains(const AAHyperBox<T, N>& inAAHyperBox, const Capsule<T, N>& inCapsule);

template <typename T, std::size_t N>
bool Contains(const AAHyperBox<T, N>& inAAHyperBox, const Triangle<T, N>& inTriangle);

template <typename T, std::size_t N>
auto GetSATNormals(const AAHyperBox<T, N>& inAAHyperBox);

template <typename T, std::size_t N>
auto GetSATEdges(const AAHyperBox<T, N>& inAAHyperBox);

template <typename T, std::size_t N>
auto GetSATPoints(const AAHyperBox<T, N>& inAAHyperBox);

template <typename T, std::size_t N>
constexpr Vec<T, N> ClosestPoint(const AAHyperBox<T, N>& inAAHyperBox, const Vec<T, N>& inPoint);

template <typename T, std::size_t N>
constexpr Vec<T, N> ClosestPoint(const AAHyperBox<T, N>& inAAHyperBox, const Line<T, N>& inLine);

template <typename T, std::size_t N>
constexpr Vec<T, N> ClosestPoint(const AAHyperBox<T, N>& inAAHyperBox, const Ray<T, N>& inRay);

template <typename T, std::size_t N>
constexpr Vec<T, N> ClosestPoint(const AAHyperBox<T, N>& inAAHyperBox, const Segment<T, N>& inSegment);

template <typename T, std::size_t N>
constexpr Vec<T, N> ClosestPoint(const AAHyperBox<T, N>& inAAHyperBox, const Capsule<T, N>& inCapsule);

template <typename T, std::size_t N, typename TPrimitive>
constexpr T SqDistance(const AAHyperBox<T, N>& inAAHyperBox, const TPrimitive& inPrimitive);

template <typename T, std::size_t N>
constexpr Vec<T, N> Center(const AAHyperBox<T, N>& inAAHyperBox);

template <typename T, std::size_t N>
constexpr AAHyperBox<T, N> Translated(const AAHyperBox<T, N>& inAAHyperBox, const Vec<T, N>& inTranslation);

template <typename T>
constexpr auto BoundingAAHyperBox(const T& inThingToBound);

template <typename T, std::size_t N>
constexpr auto BoundingAAHyperBox(const AAHyperBox<T, N>& inAAHyperBox);

template <typename T, std::size_t N>
constexpr auto BoundingAAHyperBoxTransformed(const AAHyperBox<T, N>& inAAHyperBox,
    const Transformation<T, N>& inTransformation);

template <typename T, std::size_t N>
constexpr auto BoundingAAHyperBoxInverseTransformed(const AAHyperBox<T, N>& inAAHyperBox,
    const Transformation<T, N>& inTransformation);

template <typename T, std::size_t N>
void Transform(AAHyperBox<T, N>& ioAAHyperBoxToTransform, const SquareMat<T, N>& inTransformMatrix);

template <typename T, std::size_t N>
void Transform(AAHyperBox<T, N>& ioAAHyperBoxToTransform, const SquareMat<T, N + 1>& inTransformMatrix);

template <typename T, std::size_t N>
void Transform(AAHyperBox<T, N>& ioAAHyperBoxToTransform, const Transformation<T, N>& inTransformation);

template <typename T, std::size_t N>
void InverseTransform(AAHyperBox<T, N>& ioAAHyperBoxToTransform, const Transformation<T, N>& inTransformation);
}

#include "ez/AAHyperBox.tcc"