#pragma once

#include "ez/MathInitializers.h"
#include "ez/Vec.h"

namespace ez
{
template <typename T>
class Circle
{
public:
  Circle() = default;
  Circle(const Vec2<T>& inCenter, const T& inRadius) : mCenter(inCenter), mRadius(inRadius) {}

  const Vec2<T>& GetCenter() const { return mCenter; }
  const T& GetRadius() const { return mRadius; }

private:
  Vec2<T> mCenter = Zero<Vec2<T>>();
  T mRadius = static_cast<T>(0);
};

using Circlef = Circle<float>;
using Circled = Circle<double>;
}