#pragma once

#include "ez/MathTypeTraits.h"
#include <cstdlib>
#include <iterator>
#include <type_traits>

namespace ez
{
// Points iterator
template <typename TPrimitive>
struct PointsIteratorSpecialization
{
};

template <typename TPrimitive>
class PointsIterator
{
public:
  using ValueType = ValueType_t<TPrimitive>;
  using Specialization = PointsIteratorSpecialization<TPrimitive>;
  static constexpr auto NumComponents = NumComponents_v<TPrimitive>;
  static constexpr auto NumPoints = Specialization::NumPoints;
  using VecType = Vec<ValueType, NumComponents>;

  using iterator_category = std::forward_iterator_tag;
  using difference_type = std::size_t;
  using value_type = VecType;
  using pointer = void;
  using reference = const VecType&;

  PointsIterator(const TPrimitive& inPrimitive, const std::size_t inPointIndex = 0)
      : mPrimitive { inPrimitive }, mPointIndex { inPointIndex }
  {
  }
  PointsIterator(const PointsIterator& inRHS) = default;
  PointsIterator& operator=(const PointsIterator& inRHS) = default;
  PointsIterator(PointsIterator&& ioRHS) = default;
  PointsIterator& operator=(PointsIterator&& ioRHS) = default;
  PointsIterator& operator++()
  {
    ++mPointIndex;
    return *this;
  }
  bool operator==(const PointsIterator& inRHS) const { return mPointIndex == inRHS.mPointIndex; }
  bool operator!=(const PointsIterator& inRHS) const { return !(*this == inRHS); }
  bool operator<(const PointsIterator& inRHS) const { return mPointIndex < inRHS.mPointIndex; }
  bool operator<=(const PointsIterator& inRHS) const { return mPointIndex <= inRHS.mPointIndex; }
  bool operator>(const PointsIterator& inRHS) const { return mPointIndex > inRHS.mPointIndex; }
  bool operator>=(const PointsIterator& inRHS) const { return mPointIndex >= inRHS.mPointIndex; }
  VecType operator*() const { return mSpecialization.GetPoint(mPrimitive, mPointIndex); }
  bool IsValid() const { return mPointIndex < PointsIterator<TPrimitive>::NumPoints; }

private:
  const TPrimitive& mPrimitive;
  std::size_t mPointIndex = 0;
  Specialization mSpecialization { mPrimitive };
};

template <typename TPrimitive>
PointsIterator<TPrimitive> MakePointsBegin(const TPrimitive& inPrimitive)
{
  return PointsIterator<TPrimitive>(inPrimitive);
}

template <typename TPrimitive>
PointsIterator<TPrimitive> MakePointsEnd(const TPrimitive& inPrimitive)
{
  return PointsIterator<TPrimitive>(inPrimitive, PointsIterator<TPrimitive>::NumPoints);
}

template <typename TPrimitive>
struct PointsRange
{
  PointsRange(const TPrimitive& inPrimitive) : mPrimitive { inPrimitive } {}
  PointsIterator<TPrimitive> begin() const { return MakePointsBegin(mPrimitive); }
  PointsIterator<TPrimitive> end() const { return MakePointsEnd(mPrimitive); }
  PointsIterator<TPrimitive> cbegin() const { return begin(); }
  PointsIterator<TPrimitive> cend() const { return end(); }

private:
  const TPrimitive& mPrimitive;
};

template <typename TPrimitive>
PointsRange<TPrimitive> MakePointsRange(const TPrimitive& inPrimitive)
{
  return PointsRange<TPrimitive> { inPrimitive };
}
}