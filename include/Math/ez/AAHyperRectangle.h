#pragma once

#include "ez/MathMultiComponent.h"
#include "ez/Vec.h"

namespace ez
{

// For AARect, AACube...
template <typename T, std::size_t N>
class AAHyperRectangle final
{
public:
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

  std::array<Vec<T, N>, 2>::iterator begin() { return mMinMax.begin(); }
  std::array<Vec<T, N>, 2>::iterator end() { return mMinMax.end(); }
  std::array<Vec<T, N>, 2>::const_iterator begin() const { return mMinMax.begin(); }
  std::array<Vec<T, N>, 2>::const_iterator end() const { return mMinMax.end(); }
  std::array<Vec<T, N>, 2>::const_iterator cbegin() const { return mMinMax.cbegin(); }
  std::array<Vec<T, N>, 2>::const_iterator cend() const { return mMinMax.cend(); }

private:
  std::array<Vec<T, N>, 2> mMinMax = { Max<Vec<T, N>>(), Min<Vec<T, N>>() }; // Init with invalid
};
}

#include "ez/AAHyperRectangle.tcc"