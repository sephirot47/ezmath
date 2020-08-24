#pragma once

#include <ez/MathTypeTraits.h>
#include <cstdlib>
#include <iterator>
#include <type_traits>

namespace ez
{
// Segments iterator
template <typename TPrimitive>
struct SegmentsIteratorSpecialization
{
};

template <typename TPrimitive>
class SegmentsIterator
{
public:
  using ValueType = ValueType_t<TPrimitive>;
  using Specialization = SegmentsIteratorSpecialization<TPrimitive>;
  static constexpr auto NumComponents = NumComponents_v<TPrimitive>;
  static constexpr auto NumSegments = Specialization::NumSegments;
  using SegmentType = Segment<ValueType, NumComponents>;

  using iterator_category = std::forward_iterator_tag;
  using difference_type = std::size_t;
  using value_type = SegmentType;
  using Segmenter = void;
  using reference = const SegmentType&;

  SegmentsIterator(const TPrimitive& inPrimitive, const std::size_t inSegmentIndex = 0)
      : mPrimitive { inPrimitive }, mSegmentIndex { inSegmentIndex }
  {
  }
  SegmentsIterator(const SegmentsIterator& inRHS) = default;
  SegmentsIterator& operator=(const SegmentsIterator& inRHS) = default;
  SegmentsIterator(SegmentsIterator&& ioRHS) = default;
  SegmentsIterator& operator=(SegmentsIterator&& ioRHS) = default;
  SegmentsIterator& operator++()
  {
    ++mSegmentIndex;
    return *this;
  }
  bool operator==(const SegmentsIterator& inRHS) const { return mSegmentIndex == inRHS.mSegmentIndex; }
  bool operator!=(const SegmentsIterator& inRHS) const { return !(*this == inRHS); }
  bool operator<(const SegmentsIterator& inRHS) const { return mSegmentIndex < inRHS.mSegmentIndex; }
  bool operator<=(const SegmentsIterator& inRHS) const { return mSegmentIndex <= inRHS.mSegmentIndex; }
  bool operator>(const SegmentsIterator& inRHS) const { return mSegmentIndex > inRHS.mSegmentIndex; }
  bool operator>=(const SegmentsIterator& inRHS) const { return mSegmentIndex >= inRHS.mSegmentIndex; }
  SegmentType operator*() const { return mSpecialization.GetSegment(mPrimitive, mSegmentIndex); }
  bool IsValid() const { return mSegmentIndex < SegmentsIterator<TPrimitive>::NumSegments; }

private:
  const TPrimitive& mPrimitive;
  std::size_t mSegmentIndex = 0;
  Specialization mSpecialization { mPrimitive };
};

template <typename TPrimitive>
SegmentsIterator<TPrimitive> MakeSegmentsBegin(const TPrimitive& inPrimitive)
{
  return SegmentsIterator<TPrimitive>(inPrimitive);
}

template <typename TPrimitive>
SegmentsIterator<TPrimitive> MakeSegmentsEnd(const TPrimitive& inPrimitive)
{
  return SegmentsIterator<TPrimitive>(inPrimitive, SegmentsIterator<TPrimitive>::NumSegments);
}

template <typename TPrimitive>
struct SegmentsRange
{
  SegmentsRange(const TPrimitive& inPrimitive) : mPrimitive { inPrimitive } {}
  SegmentsIterator<TPrimitive> begin() const { return MakeSegmentsBegin(mPrimitive); }
  SegmentsIterator<TPrimitive> end() const { return MakeSegmentsEnd(mPrimitive); }
  SegmentsIterator<TPrimitive> cbegin() const { return begin(); }
  SegmentsIterator<TPrimitive> cend() const { return end(); }

private:
  const TPrimitive& mPrimitive;
};

template <typename TPrimitive>
SegmentsRange<TPrimitive> MakeSegmentsRange(const TPrimitive& inPrimitive)
{
  return SegmentsRange<TPrimitive> { inPrimitive };
}
}