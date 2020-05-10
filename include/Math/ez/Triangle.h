#pragma once

#include "ez/Vec.h"
#include <array>

namespace ez
{
template <typename T, std::size_t N>
class Triangle final
{
public:
  using ValueType = T;
  static constexpr auto NumDimensions = N;

  Triangle() = default;
  Triangle(const Vec<T, N>& inPoint0, const Vec<T, N>& inPoint1, const Vec<T, N>& inPoint2)
      : mPoints { inPoint0, inPoint1, inPoint2 }
  {
  }
  Triangle(const Triangle&) = default;
  Triangle& operator=(const Triangle&) = default;
  Triangle(Triangle&&) = default;
  Triangle& operator=(Triangle&&) = default;
  ~Triangle() = default;

  typename std::array<Vec<T, N>, 3>::iterator begin() { return mPoints.begin(); }
  typename std::array<Vec<T, N>, 3>::iterator end() { return mPoints.end(); }
  typename std::array<Vec<T, N>, 3>::const_iterator begin() const { return mPoints.begin(); }
  typename std::array<Vec<T, N>, 3>::const_iterator end() const { return mPoints.end(); }
  typename std::array<Vec<T, N>, 3>::const_iterator cbegin() const { return mPoints.cbegin(); }
  typename std::array<Vec<T, N>, 3>::const_iterator cend() const { return mPoints.cend(); }
  const Vec<T, N>* data() const { return mPoints.data(); }
  std::size_t size() const { return mPoints.size(); }

  std::array<Vec<T, N>, 3>& GetPoints() { return mPoints; }
  const std::array<Vec<T, N>, 3>& GetPoints() const { return mPoints; }

  Vec<T, N>& operator[](const std::size_t inPointIndex) { return mPoints[inPointIndex]; }
  const Vec<T, N>& operator[](const std::size_t inPointIndex) const { return mPoints[inPointIndex]; }

private:
  std::array<Vec<T, N>, 3> mPoints;
};

template <typename T>
using Triangle2 = Triangle<T, 2>;
using Triangle2f = Triangle2<float>;

template <typename T>
using Triangle3 = Triangle<T, 3>;
using Triangle3f = Triangle3<float>;
}