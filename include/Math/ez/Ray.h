#pragma once

#include "ez/MathCommon.h"
#include "ez/MathInitializers.h"
#include "ez/Vec.h"

namespace ez
{
template <typename T, std::size_t N>
class Ray final
{
public:
  using ValueType = T;
  static constexpr auto NumDimensions = N;

  Ray() = default;
  Ray(const Vec<T, N>& inOrigin, const Vec<T, N>& inDirection);
  Ray(const Ray&) = default;
  Ray& operator=(const Ray&) = default;
  Ray(Ray&&) = default;
  Ray& operator=(Ray&&) = default;
  ~Ray() = default;

  void SetOrigin(const Vec<T, N>& inOrigin) { mOrigin = inOrigin; }
  void SetDirection(const Vec<T, N>& inDirection);

  Vec<T, N> GetPoint(const T& inDistance) const { return mOrigin + inDistance * mDirection; }
  const Vec<T, N>& GetOrigin() const { return mOrigin; }
  const Vec<T, N>& GetDirection() const { return mDirection; }

private:
  Vec<T, N> mOrigin = Zero<Vec<T, N>>();
  Vec<T, N> mDirection = Right<Vec<T, N>>();
};

}

#include "ez/Ray.tcc"