#pragma once

#include "ez/IntersectMode.h"
#include "ez/MathForward.h"
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

template <typename T, std::size_t N>
T Area(const Triangle<T, N>& inTriangle);

template <typename T, std::size_t N>
T Perimeter(const Triangle<T, N>& inTriangle);

template <typename T, std::size_t N>
T Barycenter(const Triangle<T, N>& inTriangle);

template <typename T>
Vec3<T> Normal(const Triangle3<T>& inTriangle);

template <typename T>
Plane<T> GetPlane(const Triangle3<T>& inTriangle);

template <typename T>
Vec3<T> Projected(const Vec3<T>& inPoint, const Triangle3<T>& inTriangle);

template <typename T, std::size_t N>
Vec3<T> BarycentricCoordinates(const Triangle<T, N>& inTriangle, const Vec<T, N>& inPoint);

template <typename T>
auto GetSATNormals(const Triangle3<T>& inTriangle);

template <typename T>
constexpr auto GetSATNormals(const Triangle2<T>&);

template <typename T, std::size_t N>
auto GetSATEdges(const Triangle<T, N>& inTriangle);

template <EIntersectMode TIntersectMode, typename T>
auto Intersect(const Ray<T, 3>& inRay, const Triangle3<T>& inTriangle);

template <typename T>
auto Intersect(const Triangle3<T>& inTriangle, const Ray<T, 3>& inRay);
}

#include "ez/Triangle.tcc"