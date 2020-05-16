#pragma once

#include "ez/MathInitializers.h"
#include "ez/Vec.h"

namespace ez
{
template <typename T, std::size_t N>
class HyperSphere
{
public:
  using ValueType = T;
  static constexpr auto NumDimensions = N;

  HyperSphere() = default;
  HyperSphere(const Vec<T, N>& inCenter, const T& inRadius) : mCenter(inCenter), mRadius(inRadius) {}

  const Vec<T, N>& GetCenter() const { return mCenter; }
  const T& GetRadius() const { return mRadius; }

private:
  Vec<T, N> mCenter = Zero<Vec<T, N>>();
  T mRadius = static_cast<T>(0);
};
}