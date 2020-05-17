#pragma once

#include "ez/IntersectMode.h"
#include "ez/MathMultiComponent.h"
#include "ez/Vec.h"

namespace ez
{

// For AARect, AACube...
template <typename T, std::size_t N>
class AAHyperRectangle final
{
public:
  using ValueType = T;
  static constexpr auto NumPoints = std::pow(static_cast<std::size_t>(2), N);
  static constexpr auto NumDimensions = N;

  AAHyperRectangle();
  AAHyperRectangle(const Vec<T, N>& inMin, const Vec<T, N>& inMax);
  AAHyperRectangle(const AAHyperRectangle&) = default;
  AAHyperRectangle& operator=(const AAHyperRectangle&) = default;
  AAHyperRectangle(AAHyperRectangle&&) = default;
  AAHyperRectangle& operator=(AAHyperRectangle&&) = default;
  ~AAHyperRectangle() = default;

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

  bool operator==(const AAHyperRectangle& inRHS) const;
  bool operator!=(const AAHyperRectangle& inRHS) const { return !(*this == inRHS); }
  bool operator<(const AAHyperRectangle& inRHS) const;
  bool operator<=(const AAHyperRectangle& inRHS) const;
  bool operator>(const AAHyperRectangle& inRHS) const;
  bool operator>=(const AAHyperRectangle& inRHS) const;
  bool operator<(const Vec<T, N>& inRHS) const { return mMinMax[0] < inRHS && mMinMax[1] < inRHS; }
  bool operator<=(const Vec<T, N>& inRHS) const { return mMinMax[0] <= inRHS && mMinMax[1] <= inRHS; }
  bool operator>(const Vec<T, N>& inRHS) const { return mMinMax[0] > inRHS && mMinMax[1] > inRHS; }
  bool operator>=(const Vec<T, N>& inRHS) const { return mMinMax[0] >= inRHS && mMinMax[1] >= inRHS; }

  template <bool IsConst>
  class GPointsIterator
  {
  public:
    using AAHyperRectangleType = std::conditional_t<IsConst, const AAHyperRectangle, AAHyperRectangle>;
    using VecType = std::conditional_t<IsConst, const Vec<T, N>, Vec<T, N>>;

    GPointsIterator(AAHyperRectangleType& ioHyperRectangle, const std::size_t inBeginIndex);
    GPointsIterator& operator++();
    bool operator==(const GPointsIterator& inRHS) const { return mCurrentIndex == inRHS.mCurrentIndex; }
    bool operator!=(const GPointsIterator& inRHS) const { return !(*this == inRHS); }
    VecType operator*() const;

  private:
    AAHyperRectangleType& mAAHyperRectangle;
    std::size_t mCurrentIndex = 0;
  };
  using PointsIterator = GPointsIterator<false>;
  using PointsConstIterator = GPointsIterator<true>;

  AAHyperRectangle::PointsIterator begin() { return PointsIterator(*this, 0); }
  AAHyperRectangle::PointsIterator end() { return PointsIterator(*this, NumPoints); }
  AAHyperRectangle::PointsConstIterator begin() const { return cbegin(); }
  AAHyperRectangle::PointsConstIterator end() const { return cend(); }
  AAHyperRectangle::PointsConstIterator cbegin() const { return PointsConstIterator(*this, 0); }
  AAHyperRectangle::PointsConstIterator cend() const { return PointsConstIterator(*this, NumPoints); }

  std::array<Vec<T, N>, 2>::iterator GetTransformIteratorBegin() { return mMinMax.begin(); }
  std::array<Vec<T, N>, 2>::iterator GetTransformIteratorEnd() { return mMinMax.end(); }

  using TransformIterator = std::array<Vec<T, N>, 2>::iterator;

private:
  std::array<Vec<T, N>, 2> mMinMax = { Max<Vec<T, N>>(), Min<Vec<T, N>>() }; // Init with invalid
};

template <typename T, std::size_t N>
class Ray;

// Traits
template <typename T, std::size_t N>
struct IsAAHyperRectangle<AAHyperRectangle<T, N>> : std::true_type
{
};

// Intersection functions
template <EIntersectMode TIntersectMode, typename T, std::size_t N>
auto Intersect(const AAHyperRectangle<T, N>& inLHS, const AAHyperRectangle<T, N>& inRHS)
{
  static_assert(TIntersectMode == EIntersectMode::ONLY_CHECK, "Unsupported EIntersectMode.");
  return inLHS.GetMin() <= inRHS.GetMax() || inRHS.GetMin() <= inLHS.GetMax();
}

template <typename T, std::size_t N>
bool Contains(const AAHyperRectangle<T, N>& inAAHyperRectangle, const Vec<T, N>& inPoint)
{
  return inPoint >= inAAHyperRectangle.GetMin() && inPoint <= inAAHyperRectangle.GetMax();
}

}

#include "ez/AAHyperRectangle.tcc"