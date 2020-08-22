#include "ez/HyperSphere.h"

namespace ez
{
template <EIntersectMode TIntersectMode, typename T, std::size_t N>
auto Intersect(const HyperSphere<T, N>& inHyperSphereLHS, const HyperSphere<T, N>& inHyperSphereRHS)
{
  static_assert(TIntersectMode == EIntersectMode::ONLY_CHECK, "Unsupported EIntersectMode.");
  return SqDistance(Center(inHyperSphereLHS), Center(inHyperSphereRHS))
      <= Sq(inHyperSphereLHS.GetRadius() + inHyperSphereRHS.GetRadius());
}

template <EIntersectMode TIntersectMode, typename T, std::size_t N>
auto Intersect(const HyperSphere<T, N>& inHyperSphere, const Vec<T, N>& inPoint)
{
  return Intersect<TIntersectMode>(inPoint, inHyperSphere);
}

template <EIntersectMode TIntersectMode, typename T, std::size_t N>
auto Intersect(const HyperSphere<T, N>& inHyperSphere, const Line<T, N>& inLine)
{
  return Intersect<TIntersectMode>(inLine, inHyperSphere);
}

template <EIntersectMode TIntersectMode, typename T, std::size_t N>
auto Intersect(const HyperSphere<T, N>& inHyperSphere, const Ray<T, N>& inRay)
{
  return Intersect<TIntersectMode>(inRay, inHyperSphere);
}

template <EIntersectMode TIntersectMode, typename T, std::size_t N>
auto Intersect(const HyperSphere<T, N>& inHyperSphere, const Segment<T, N>& inSegment)
{
  return Intersect<TIntersectMode>(inSegment, inHyperSphere);
}

template <EIntersectMode TIntersectMode, typename T, std::size_t N>
auto Intersect(const HyperSphere<T, N>& inHyperSphere, const AAHyperBox<T, N>& inAAHyperBox)
{
  static_assert(TIntersectMode == EIntersectMode::ONLY_CHECK, "Unsupported EIntersectMode.");

  T min_distance = static_cast<T>(0);
  const auto sphere_center = Center(inHyperSphere);
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
auto Intersect(const HyperSphere<T, N>& inHyperSphere, const HyperBox<T, N>& inHyperBox)
{
  const auto hyper_box_orientation_inv = -Orientation(inHyperBox);
  const auto aa_hyper_box_center = Rotated(Center(inHyperBox), hyper_box_orientation_inv);
  const auto aa_hyper_box = MakeAAHyperBoxFromCenterSize(aa_hyper_box_center, inHyperBox.GetSize());
  return Intersect<TIntersectMode>(Rotated(inHyperSphere, hyper_box_orientation_inv), aa_hyper_box);
}

template <EIntersectMode TIntersectMode, typename T, std::size_t N>
auto Intersect(const HyperSphere<T, N>& inHyperSphere, const Capsule<T, N>& inCapsule)
{
  static_assert(TIntersectMode == EIntersectMode::ONLY_CHECK, "Unsupported EIntersectMode.");
  return SqDistance(inCapsule.GetSegment(), Center(inHyperSphere))
      < Sq(inHyperSphere.GetRadius() + inCapsule.GetRadius());
}

template <EIntersectMode TIntersectMode, typename T, std::size_t N>
auto Intersect(const HyperSphere<T, N>& inHyperSphere, const Triangle<T, N>& inTriangle)
{
  static_assert(TIntersectMode == EIntersectMode::ONLY_CHECK, "Unsupported EIntersectMode.");
  const auto hyper_sphere_sq_radius = Sq(inHyperSphere.GetRadius());
  return (SqDistance(Segment<T, N> { inTriangle[0], inTriangle[1] }, Center(inHyperSphere)) <= hyper_sphere_sq_radius)
      || (SqDistance(Segment<T, N> { inTriangle[1], inTriangle[2] }, Center(inHyperSphere)) <= hyper_sphere_sq_radius)
      || (SqDistance(Segment<T, N> { inTriangle[2], inTriangle[0] }, Center(inHyperSphere)) <= hyper_sphere_sq_radius);
}

template <typename T, std::size_t N>
bool Contains(const HyperSphere<T, N>& inHyperSphere, const Vec<T, N>& inPoint)
{
  return SqDistance(Center(inHyperSphere), inPoint) <= Sq(inHyperSphere.GetRadius());
}

template <typename T, std::size_t N>
bool Contains(const HyperSphere<T, N>& inHyperSphere, const Line<T, N>& inLine)
{
  return false;
}

template <typename T, std::size_t N>
bool Contains(const HyperSphere<T, N>& inHyperSphere, const Ray<T, N>& inRay)
{
  return false;
}

template <typename T, std::size_t N>
bool Contains(const HyperSphere<T, N>& inHyperSphere, const Segment<T, N>& inSegment)
{
  return Contains(inHyperSphere, inSegment.GetOrigin()) && Contains(inHyperSphere, inSegment.GetDestiny());
}

template <typename T, std::size_t N>
bool Contains(const HyperSphere<T, N>& inHyperSphereContainer, const HyperSphere<T, N>& inHyperSphereContainee)
{
  const auto radius_diff = (inHyperSphereContainer.GetRadius() - inHyperSphereContainee.GetRadius());
  if (radius_diff < static_cast<T>(0))
    return false;

  return (SqDistance(Center(inHyperSphereContainer), Center(inHyperSphereContainee)) <= Sq(radius_diff));
}

template <typename T, std::size_t N>
bool Contains(const HyperSphere<T, N>& inHyperSphere, const AAHyperBox<T, N>& inAAHyperBox)
{
  const auto aa_hyper_box_min = inAAHyperBox.GetMin();
  const auto aa_hyper_box_size = inAAHyperBox.GetSize();
  for (int i = 0; i < AAHyperBox<T, N>::NumPoints; ++i)
  {
    const auto aa_hyper_box_point = aa_hyper_box_min + aa_hyper_box_size * MakeBinaryIndex<N, T>(i);
    if (!Contains(inHyperSphere, aa_hyper_box_point))
      return false;
  }
  return true;
}

template <typename T, std::size_t N>
bool Contains(const HyperSphere<T, N>& inHyperSphere, const HyperBox<T, N>& inHyperBox)
{
  const auto hyper_box_center = inHyperBox.GetCenter();
  const auto hyper_box_extents = inHyperBox.GetExtents();
  const auto hyper_box_orientation = Orientation(inHyperBox);
  for (int i = 0; i < HyperBox<T, N>::NumPoints; ++i)
  {
    const auto rotated_extents = Rotated(hyper_box_extents * (MakeBinaryIndex<N, T>(i) * 2 - 1), hyper_box_orientation);
    const auto hyper_box_point = hyper_box_center + rotated_extents;
    if (!Contains(inHyperSphere, hyper_box_point))
      return false;
  }
  return true;
}

template <typename T, std::size_t N>
bool Contains(const HyperSphere<T, N>& inHyperSphere, const Capsule<T, N>& inCapsule)
{
  if (!Contains(inHyperSphere, inCapsule.GetOrigin()))
    return false;

  if (!Contains(inHyperSphere, inCapsule.GetDestiny()))
    return false;

  const auto sphere_sq_half_radius = Sq(inHyperSphere.GetRadius() / static_cast<T>(2));
  if (SqDistance(Center(inHyperSphere), inCapsule.GetOrigin()) >= sphere_sq_half_radius)
    return false;

  if (SqDistance(Center(inHyperSphere), inCapsule.GetDestiny()) >= sphere_sq_half_radius)
    return false;

  return true;
}

template <typename T, std::size_t N>
bool Contains(const HyperSphere<T, N>& inHyperSphere, const Triangle<T, N>& inTriangle)
{
  return Contains(inHyperSphere, inTriangle[0]) && Contains(inHyperSphere, inTriangle[1])
      && Contains(inHyperSphere, inTriangle[2]);
}

template <typename T, std::size_t N>
constexpr HyperSphere<T, N> Translated(const HyperSphere<T, N>& inHyperSphere, const Vec<T, N>& inTranslation)
{
  return HyperSphere<T, N> { Center(inHyperSphere) + inTranslation, inHyperSphere.GetRadius() };
}

template <typename T, std::size_t N>
constexpr HyperSphere<T, N> Rotated(const HyperSphere<T, N>& inHyperSphere, const RotationType_t<T, N>& inRotation)
{
  return HyperSphere<T, N> { Rotated(Center(inHyperSphere), inRotation), inHyperSphere.GetRadius() };
}

template <typename T, std::size_t N>
constexpr Vec<T, N> Center(const HyperSphere<T, N>& inHyperSphere)
{
  return inHyperSphere.GetCenter();
}

template <typename T, std::size_t N>
AAHyperBox<T, N> BoundingAAHyperBox(const HyperSphere<T, N>& inHyperSphere)
{
  return MakeAAHyperBoxFromCenterHalfSize(Center(inHyperSphere), All<Vec<T, N>>(inHyperSphere.GetRadius()));
}

}