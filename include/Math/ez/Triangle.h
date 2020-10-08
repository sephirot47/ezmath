#pragma once

#include <ez/IntersectMode.h>
#include <ez/MathForward.h>
#include <ez/PointsIterator.h>
#include <ez/Quat.h>
#include <ez/Triangle.h>
#include <ez/Vec.h>
#include <array>

namespace ez
{
template <typename T, std::size_t N>
class Triangle final
{
public:
  using ValueType = T;
  static constexpr auto NumComponents = N;
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

// Traits
template <typename T, std::size_t N>
struct IsTriangle<Triangle<T, N>> : std::true_type
{
};

template <typename T, std::size_t N>
T Area(const Triangle<T, N>& inTriangle);

template <typename T, std::size_t N>
T GetPerimeter(const Triangle<T, N>& inTriangle);

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
auto GetSATNormals(const Triangle2<T>& inTriangle);
template <typename T>
auto GetSATNormals(const Triangle3<T>& inTriangle);

template <typename T, std::size_t N>
auto GetSATEdges(const Triangle<T, N>& inTriangle);

template <typename T, std::size_t N>
auto GetSATPoints(const Triangle<T, N>& inTriangle);

template <typename T, std::size_t N>
constexpr Triangle<T, N> Translated(const Triangle<T, N>& inTriangle, const Vec<T, N>& inTranslation);

template <typename T, std::size_t N>
constexpr Triangle<T, N> Rotated(const Triangle<T, N>& inTriangle, const RotationType_t<T, N>& inRotation);

// Intersect
template <EIntersectMode TIntersectMode, typename T, std::size_t N>
auto Intersect(const Triangle<T, N>& inTriangle, const Vec<T, N>& inPoint);

template <EIntersectMode TIntersectMode, typename T, std::size_t N>
auto Intersect(const Triangle<T, N>& inTriangle, const Line<T, N>& inLine);

template <EIntersectMode TIntersectMode, typename T, std::size_t N>
auto Intersect(const Triangle<T, N>& inTriangle, const Ray<T, N>& inRay);

template <EIntersectMode TIntersectMode, typename T, std::size_t N>
auto Intersect(const Triangle<T, N>& inTriangle, const Segment<T, N>& inSegment);

template <EIntersectMode TIntersectMode, typename T, std::size_t N>
auto Intersect(const Triangle<T, N>& inTriangle, const HyperSphere<T, N>& inHyperSphere);

template <EIntersectMode TIntersectMode, typename T, std::size_t N>
auto Intersect(const Triangle<T, N>& inTriangle, const AAHyperBox<T, N>& inAAHyperBox);

template <EIntersectMode TIntersectMode, typename T, std::size_t N>
auto Intersect(const Triangle<T, N>& inTriangle, const HyperBox<T, N>& inHyperBox);

template <EIntersectMode TIntersectMode, typename T, std::size_t N>
auto Intersect(const Triangle<T, N>& inTriangle, const Capsule<T, N>& inCapsule);

template <EIntersectMode TIntersectMode, typename T, std::size_t N>
auto Intersect(const Triangle<T, N>& inTriangleLHS, const Triangle<T, N>& inTriangleRHS);

// Contains
template <typename T, std::size_t N>
bool Contains(const Triangle<T, N>& inTriangle, const Vec<T, N>& inPoint);

template <typename T, std::size_t N>
bool Contains(const Triangle<T, N>& inTriangle, const Line<T, N>& inLine);

template <typename T, std::size_t N>
bool Contains(const Triangle<T, N>& inTriangle, const Ray<T, N>& inRay);

template <typename T, std::size_t N>
bool Contains(const Triangle<T, N>& inTriangle, const Segment<T, N>& inSegment);

template <typename T, std::size_t N>
bool Contains(const Triangle<T, N>& inTriangle, const HyperSphere<T, N>& inHyperSphere);

template <typename T, std::size_t N>
bool Contains(const Triangle<T, N>& inTriangle, const AAHyperBox<T, N>& inAAHyperBox);

template <typename T, std::size_t N>
bool Contains(const Triangle<T, N>& inTriangle, const HyperBox<T, N>& inHyperBox);

template <typename T, std::size_t N>
bool Contains(const Triangle<T, N>& inTriangle, const Capsule<T, N>& inCapsule);

template <typename T, std::size_t N>
bool Contains(const Triangle<T, N>& inTriangleContainer, const Triangle<T, N>& inTriangleContainee);

// ClosestPoint
template <typename T, std::size_t N, typename TPrimitive>
constexpr Vec<T, N> ClosestPoint(const Triangle<T, N>& inTriangle, const TPrimitive& inPrimitive);

// Points iterator
template <typename T, std::size_t N>
struct PointsIteratorSpecialization<Triangle<T, N>>
{
  static constexpr std::size_t NumPoints = 3;
  PointsIteratorSpecialization(const Triangle<T, N>& inTriangle) {}
  Vec<T, N> GetPoint(const Triangle<T, N>& inTriangle, const std::size_t inPointIndex) const;
};
}

#include "ez/Triangle.tcc"