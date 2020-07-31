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
  static constexpr auto NumPoints = std::pow(static_cast<std::size_t>(2), N);
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