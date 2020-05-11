#pragma once

#include "ez/Macros.h"
#include "ez/MathInitializers.h"
#include "ez/Vec.h"

namespace ez
{
template <typename T>
class Plane final
{
public:
  using ValueType = T;
  static constexpr auto NumDimensions = 3;

  Plane() = default;
  Plane(const Vec3<T>& inNormal, const T inDistanceFromOriginInNormalDirection);
  Plane(const Vec3<T>& inNormal, const Vec3<T>& inPlanePoint);
  Plane(const Plane&) = default;
  Plane& operator=(const Plane&) = default;
  Plane(Plane&&) = default;
  Plane& operator=(Plane&&) = default;
  ~Plane() = default;

  void SetNormal(const Vec3<T>& inNormal);
  void SetDistanceFromOrigin(const T inDistance);

  const Vec3<T>& GetNormal() const;
  T GetDistanceFromOrigin() const;
  Vec3<T> GetArbitraryPoint() const;

private:
  Vec3<T> mNormal = Forward<Vec3<T>>();      // A, B, C
  T mDistanceFromOrigin = static_cast<T>(0); // Distance from origin i-n the direction of the normal (D)
};

using Planef = Plane<float>;
using Planed = Plane<double>;
}

#include "ez/Plane.tcc"