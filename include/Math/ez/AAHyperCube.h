#pragma once

#include "ez/IntersectMode.h"
#include "ez/MathMultiComponent.h"
#include "ez/Vec.h"

namespace ez
{

// For AARect, AACube...
template <typename T, std::size_t N>
class AAHyperCube final
{
public:
  using ValueType = T;
  static constexpr auto NumPoints = std::pow(static_cast<std::size_t>(2), N);
  static constexpr auto NumComponents = N;
  static constexpr auto NumDimensions = N;

  AAHyperCube();
  template <typename TOther>
  explicit AAHyperCube(const AAHyperCube<TOther, N>& inRHS);
  AAHyperCube(const Vec<T, N>& inMin, const T& inSize);
  AAHyperCube(const AAHyperCube&) = default;
  AAHyperCube& operator=(const AAHyperCube&) = default;
  AAHyperCube(AAHyperCube&&) = default;
  AAHyperCube& operator=(AAHyperCube&&) = default;

  ~AAHyperCube() = default;

  void SetMin(const Vec<T, N>& inMin);
  void SetSize(const T& inSize);

  template <typename TOther>
  void Wrap(const TOther& inThingToBound);

  Vec<T, N> GetCenter() const { return mMin + (mSize / static_cast<T>(2)); }
  const T& GetSize() const { return mSize; }
  const Vec<T, N>& GetMin() const { return mMin; }
  const Vec<T, N> GetMax() const { return mMin + All<Vec<T, N>>(mSize); }

  bool operator==(const AAHyperCube& inRHS) const;
  bool operator!=(const AAHyperCube& inRHS) const { return !(*this == inRHS); }
  AAHyperCube operator+(const Vec<T, N>& inRHS);
  AAHyperCube operator-(const Vec<T, N>& inRHS) { return (*this) + (-inRHS); }
  AAHyperCube& operator+=(const Vec<T, N>& inRHS);
  AAHyperCube& operator-=(const Vec<T, N>& inRHS) { return ((*this) += (-inRHS)); }
  AAHyperCube operator*(const Vec<T, N>& inRHS);
  AAHyperCube operator/(const Vec<T, N>& inRHS) { return (*this) * (static_cast<T>(1) / inRHS); }
  AAHyperCube& operator*=(const Vec<T, N>& inRHS);
  AAHyperCube& operator/=(const Vec<T, N>& inRHS) { return ((*this) *= (static_cast<T>(1) / inRHS)); }

private:
  Vec<T, N> mMin = Max<Vec<T, N>>(); // Init with invalid
  T mSize = Min<T>();                // Init with invalid
};

// Traits
template <typename T, std::size_t N>
struct IsAAHyperCube<AAHyperCube<T, N>> : std::true_type
{
};

// Intersection functions
template <EIntersectMode TIntersectMode, typename T, std::size_t N>
auto Intersect(const AAHyperCube<T, N>& inAAHyperCubeLHS, const AAHyperCube<T, N>& inAAHyperCubeRHS);

template <EIntersectMode TIntersectMode, typename T, std::size_t N, typename TPrimitive>
auto Intersect(const AAHyperCube<T, N>& inAAHyperCube, const TPrimitive& inPrimitive);

template <typename T, std::size_t N>
bool Contains(const AAHyperCube<T, N>& inAAHyperCube, const Vec<T, N>& inPoint);

template <typename T, std::size_t N>
bool Contains(const AAHyperCube<T, N>& inAAHyperCubeContainer, const AAHyperCube<T, N>& inAAHyperCubeContainee);

template <typename T, std::size_t N>
const auto MakeAAHyperCubeFromMinSize(const Vec<T, N>& inMin, const T& inSize);

template <typename T, std::size_t N>
const auto MakeAAHyperCubeFromCenterSize(const Vec<T, N>& inCenter, const T& inSize);

template <typename T, std::size_t N>
constexpr Vec<T, N> Center(const AAHyperCube<T, N>& inAAHyperCube);

}

#include "ez/AAHyperCube.tcc"