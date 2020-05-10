#pragma once

#include "ez/MathCommon.h"
#include "ez/Plane.h"
#include "ez/Triangle.h"

namespace ez
{
template <typename T, std::size_t N>
T Area(const Triangle<T, N>& inTriangle)
{
  const auto length_01 = Distance(inTriangle[0], inTriangle[1]);
  const auto length_02 = Distance(inTriangle[0], inTriangle[2]);
  const auto length_12 = Distance(inTriangle[1], inTriangle[2]);
  const auto half_sum_of_lengths = (length_01 + length_02 + length_12) * 0.5f;
  const auto& hs = half_sum_of_lengths;
  const auto area = std::sqrt(hs * (hs - length_01) * (hs - length_02) * (hs - length_12));
  return area;
}

template <typename T, std::size_t N>
T Perimeter(const Triangle<T, N>& inTriangle)
{
  const auto length_01 = Distance(inTriangle[0], inTriangle[1]);
  const auto length_02 = Distance(inTriangle[0], inTriangle[2]);
  const auto length_12 = Distance(inTriangle[1], inTriangle[2]);
  const auto perimeter = (length_01 + length_02 + length_12);
  return perimeter;
}

template <typename T, std::size_t N>
T Barycenter(const Triangle<T, N>& inTriangle)
{
  return (inTriangle[0] + inTriangle[1] + inTriangle[2]) / static_cast<T>(3);
}

template <typename T>
Vec3<T> Normal(const Triangle3<T>& inTriangle)
{
  const auto v01 = (inTriangle[0] - inTriangle[1]);
  const auto v21 = (inTriangle[2] - inTriangle[1]);
  return NormalizedSafe(Cross(v01, v21));
}

template <typename T>
Vec3<T> Normal(const Plane<T>& inPlane)
{
  return inPlane.GetNormal();
}

template <typename T>
Plane<T> GetPlane(const Triangle3<T>& inTriangle)
{
  const auto v10 = (inTriangle[0] - inTriangle[1]);
  const auto v12 = (inTriangle[2] - inTriangle[1]);
  const auto& plane_point = inTriangle[0];
  const auto plane_normal = NormalizedSafe(Cross(v10, v12));
  const auto triangle_plane = Plane<T>(plane_normal, plane_point);
  return triangle_plane;
}

template <typename T>
Vec3<T> Projected(const Vec3<T>& inPoint, const Plane<T>& inPlaneToProjectTo)
{
  const auto plane_point = inPlaneToProjectTo.GetArbitraryPoint();
  const auto& plane_normal = inPlaneToProjectTo.GetNormal();
  const auto plane_to_point_vector = (inPoint - plane_point);
  const auto vector_projected_to_plane_normal = Dot(plane_to_point_vector, plane_normal);
  const auto point_projected_to_plane_normal = (inPoint - vector_projected_to_plane_normal);
  return point_projected_to_plane_normal;
}

template <typename T>
Vec3<T> Projected(const Vec3<T>& inPoint, const Triangle3<T>& inTriangle)
{
  const auto triangle_plane = Plane(inTriangle);
  return Projected(inPoint, triangle_plane);
}
}