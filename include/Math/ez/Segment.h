#pragma once

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

template <typename T>
using Segment2 = Segment<T, 2>;
using Segment2b = Segment2<bool>;
using Segment2i = Segment2<int32_t>;
using Segment2u = Segment2<uint32_t>;
using Segment2f = Segment2<float>;
using Segment2d = Segment2<double>;

template <typename T>
using Segment3 = Segment<T, 3>;
using Segment3b = Segment3<bool>;
using Segment3i = Segment3<int32_t>;
using Segment3u = Segment3<uint32_t>;
using Segment3f = Segment3<float>;
using Segment3d = Segment3<double>;

}

#include "ez/Segment.tcc"