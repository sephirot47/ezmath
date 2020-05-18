#pragma once

#include "ez/IntersectMode.h"
#include "ez/MathMultiComponent.h"
#include "ez/Vec.h"

namespace ez
{

// For AARect, AABox...
template <typename T, std::size_t N>
class AAHyperBox final
{
public:
  using ValueType = T;
  static constexpr auto NumPoints = std::pow(static_cast<std::size_t>(2), N);
  static constexpr auto NumDimensions = N;

  AAHyperBox();
  AAHyperBox(const Vec<T, N>& inMin, const Vec<T, N>& inMax);
  AAHyperBox(const AAHyperBox&) = default;
  AAHyperBox& operator=(const AAHyperBox&) = default;
  AAHyperBox(AAHyperBox&&) = default;
  AAHyperBox& operator=(AAHyperBox&&) = default;
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

  std::array<Vec<T, N>, 2>::iterator GetTransformIteratorBegin() { return mMinMax.begin(); }
  std::array<Vec<T, N>, 2>::iterator GetTransformIteratorEnd() { return mMinMax.end(); }

  using TransformIterator = std::array<Vec<T, N>, 2>::iterator;

private:
  std::array<Vec<T, N>, 2> mMinMax = { Max<Vec<T, N>>(), Min<Vec<T, N>>() }; // Init with invalid
};

// Traits
template <typename T, std::size_t N>
struct IsAAHyperBox<AAHyperBox<T, N>> : std::true_type
{
};

// Intersection functions
template <EIntersectMode TIntersectMode, typename T, std::size_t N>
auto Intersect(const AAHyperBox<T, N>& inLHS, const AAHyperBox<T, N>& inRHS)
{
  static_assert(TIntersectMode == EIntersectMode::ONLY_CHECK, "Unsupported EIntersectMode.");
  return inLHS.GetMin() <= inRHS.GetMax() || inRHS.GetMin() <= inLHS.GetMax();
}

template <typename T, std::size_t N>
bool Contains(const AAHyperBox<T, N>& inAAHyperBox, const Vec<T, N>& inPoint)
{
  return inPoint >= inAAHyperBox.GetMin() && inPoint <= inAAHyperBox.GetMax();
}

template <typename T>
constexpr auto BoundingAAHyperBox(const T& inThingToBound);
}

#include "ez/AAHyperBox.tcc"