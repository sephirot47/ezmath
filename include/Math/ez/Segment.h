#pragma once

#include "ez/MathForward.h"
#include "ez/Vec.h"
#include <cstdint>

namespace ez
{
template <typename T, std::size_t N>
class Segment final
{
public:
  using ValueType = T;
  static constexpr auto NumComponents = N;
  static constexpr auto NumDimensions = N;

  Segment() = default;
  Segment(const Vec<T, N>& inFromPoint, const Vec<T, N>& inToPoint);
  Segment(const Segment& inRHS) = default;
  Segment& operator=(const Segment& inRHS) = default;
  Segment(Segment&& inRHS) = default;
  Segment& operator=(Segment&& inRHS) = default;

  Vec<T, N> GetFromPoint() const;
  Vec<T, N> GetToPoint() const;
  Vec<T, N> GetVector() const;

private:
  Vec<T, N> mFromPoint = Vec<T, N> { 0 };
  Vec<T, N> mToPoint = Vec<T, N> { 0 };
};

template <typename T, std::size_t N>
constexpr Vec3<T> Direction(const Segment<T, N>& inSegment);

template <typename T>
Vec3<T> Projected(const Vec3<T>& inPoint, const Segment3<T>& inSegment);

template <typename T>
T SqDistance(const Vec3<T>& inPoint, const Segment3<T>& inSegment);

template <typename T>
T SqDistance(const Segment3<T>& inSegment, const Vec3<T>& inPoint);

template <typename T>
T SqDistance(const Segment3<T>& inSegmentLHS, const Segment3<T>& inSegmentRHS);

}

#include "ez/Segment.tcc"