#pragma once

#include <ez/IntersectMode.h>
#include <ez/MathForward.h>
#include <ez/MathInitializers.h>
#include <ez/Vec.h>

namespace ez
{
template <typename T, std::size_t N>
class Line
{
public:
  Line() = default;
  Line(const Vec<T, N>& inOrigin, const Vec<T, N>& inDirection);

  const Vec<T, N>& GetOrigin() const { return mOrigin; }
  const Vec<T, N>& GetDirection() const { return mDirection; }
  Vec<T, N> GetPoint(const T& inDistance) const { return mOrigin + mDirection * inDistance; }

  void SetOrigin(const Vec<T, N>& inOrigin) { mOrigin = inOrigin; }
  void SetDirection(const Vec<T, N>& inDirection);

private:
  Vec<T, N> mOrigin;
  Vec<T, N> mDirection;
};

template <typename T, std::size_t N>
constexpr Vec<T, N> Direction(const Line<T, N>& inLine);

template <typename T, std::size_t N>
constexpr RotationType_t<T, N> Orientation(const Line<T, N>& inLine);

template <EIntersectMode TIntersectMode, typename T>
auto Intersect(const Line3<T>& inRay, const Plane<T>& inPlane);

template <EIntersectMode TIntersectMode, typename T>
auto Intersect(const Plane<T>& inPlane, const Line3<T>& inRay);

template <EIntersectMode TIntersectMode, typename T, std::size_t N>
auto Intersect(const Line<T, N>& inLine, const AAHyperBox<T, N>& inAAHyperBox);

template <EIntersectMode TIntersectMode, typename T, std::size_t N>
auto Intersect(const AAHyperBox<T, N>& inAAHyperBox, const Line<T, N>& inLine);
}

#include <ez/Line.tcc>