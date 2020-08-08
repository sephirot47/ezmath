#include "ez/HyperSphere.h"

namespace ez
{
template <EIntersectMode TIntersectMode, typename T, std::size_t N>
auto Intersect(const HyperSphere<T, N>& inLHS, const HyperSphere<T, N>& inRHS)
{
  static_assert(TIntersectMode == EIntersectMode::ONLY_CHECK, "Unsupported EIntersectMode.");
  return SqDistance(inLHS.GetCenter(), inRHS.GetCenter()) <= Sq(inLHS.GetRadius() + inRHS.GetRadius());
}

template <typename T, std::size_t N>
bool Contains(const HyperSphere<T, N>& inHyperSphere, const Vec<T, N>& inPoint)
{
  return SqDistance(inPoint, inHyperSphere.GetCenter()) <= Sq(inHyperSphere.GetRadius());
}

template <typename T, std::size_t N>
bool Contains(const HyperSphere<T, N>& inHyperSphere, const AAHyperBox<T, N>& inAAHyperBox)
{
  const auto aa_hyper_box_min = inAAHyperBox.GetMin();
  const auto aa_hyper_box_size = inAAHyperBox.GetSize();
  constexpr auto all_binary_indices = AllBinaryIndices<N, T>();
  for (const auto binary_index : all_binary_indices)
  {
    const auto aa_hyper_box_point = aa_hyper_box_min + aa_hyper_box_size * binary_index;
    if (!Contains(inHyperSphere, aa_hyper_box_point))
      return false;
  }
  return true;
}

template <typename T, std::size_t N>
bool Contains(const HyperSphere<T, N>& inHyperSphere, const AAHyperCube<T, N>& inAAHyperCube)
{
  return Contains(inHyperSphere, MakeAAHyperBoxFromAAHyperCube(inAAHyperCube));
}

template <EIntersectMode TIntersectMode, typename T, std::size_t N>
auto Intersect(const HyperSphere<T, N>& inHyperSphere, const AAHyperBox<T, N>& inAAHyperBox)
{
  static_assert(TIntersectMode == EIntersectMode::ONLY_CHECK, "Unsupported EIntersectMode.");

  T min_distance = 0;
  const auto sphere_center = inHyperSphere.GetCenter();
  const auto aabox_min = inAAHyperBox.GetMin();
  const auto aabox_max = inAAHyperBox.GetMax();
  for (std::size_t i = 0; i < N; ++i)
  {
    if (sphere_center[i] < aabox_min[i])
      min_distance += Sq(sphere_center[i] - aabox_min[i]);
    else if (sphere_center[i] > aabox_max[i])
      min_distance += Sq(sphere_center[i] - aabox_max[i]);
  }
  return (min_distance <= Sq(inHyperSphere.GetRadius()));
}

template <EIntersectMode TIntersectMode, typename T, std::size_t N>
auto Intersect(const AAHyperBox<T, N>& inAAHyperBox, const HyperSphere<T, N>& inHyperSphere)
{
  return Intersect<TIntersectMode, T, N>(inHyperSphere, inAAHyperBox);
}

template <EIntersectMode TIntersectMode, typename T, std::size_t N>
auto Intersect(const HyperSphere<T, N>& inHyperSphere, const AAHyperCube<T, N>& inAAHyperCube)
{
  return Intersect<TIntersectMode, T, N>(inHyperSphere, MakeAAHyperBoxFromAAHyperCube(inAAHyperCube));
}

template <EIntersectMode TIntersectMode, typename T, std::size_t N>
auto Intersect(const AAHyperCube<T, N>& inAAHyperCube, const HyperSphere<T, N>& inHyperSphere)
{
  return Intersect<TIntersectMode, T, N>(inHyperSphere, inAAHyperCube);
}

template <typename T, std::size_t N>
AAHyperBox<T, N> BoundingAAHyperBox(const HyperSphere<T, N>& inHyperSphere)
{
  return MakeAAHyperBoxFromCenterHalfSize(inHyperSphere.GetCenter(), All<Vec<T, N>>(inHyperSphere.GetRadius()));
}

}