#pragma once

#include <ez/Vec.h>

namespace ez
{
template <typename T>
class Capsule
{
public:
  Capsule() = default;
  Capsule(const Vec3<T>& inOrigin, const Vec3<T>& inDestiny, const T inRadius);

  void SetOrigin(const Vec3<T>& inOrigin) { mOrigin = inOrigin; }
  void SetDestiny(const Vec3<T>& inDestiny) { mDestiny = inDestiny; }
  void SetRadius(const T inRadius) { mRadius = inRadius; }

  const Vec3<T>& GetOrigin() const { return mOrigin; }
  const Vec3<T>& GetDestiny() const { return mDestiny; }
  const T& GetRadius() const { return mRadius; }

private:
  Vec3<T> mOrigin { Zero<Vec3<T>>() };
  Vec3<T> mDestiny { One<Vec3<T>>() };
  T mRadius { static_cast<T>(1) };
};

// Traits
template <typename T>
struct IsCapsule<Capsule<T>> : std::true_type
{
};

}

#include "ez/Capsule.tcc"