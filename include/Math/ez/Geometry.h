#pragma once

#include "ez/MathCommon.h"
#include "ez/MathForward.h"

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
T Distance(const Vec3<T>& inPoint, const Plane<T>& inPlane)
{
  const auto plane_point = inPlane.GetArbitraryPoint();
  const auto& plane_normal = inPlane.GetNormal();
  const auto plane_to_point_vector = (inPoint - plane_point);
  const auto vector_projected_to_plane_normal_length = Dot(plane_to_point_vector, plane_normal);
  return vector_projected_to_plane_normal_length;
}

template <typename T>
Vec3<T> Projected(const Vec3<T>& inPoint, const Plane<T>& inPlaneToProjectTo)
{
  const auto point_to_plane_distance = Distance(inPoint, inPlaneToProjectTo);
  const auto point_projected_to_plane_normal = (inPoint - point_to_plane_distance * inPlaneToProjectTo.GetNormal());
  return point_projected_to_plane_normal;
}

template <typename T>
Vec3<T> Projected(const Vec3<T>& inPoint, const Triangle3<T>& inTriangle)
{
  const auto triangle_plane = Plane(inTriangle);
  return Projected(inPoint, triangle_plane);
}

template <typename T, std::size_t N>
Vec3<T> BarycentricCoordinates(const Triangle<T, N>& inTriangle, const Vec<T, N>& inPoint)
{
  const auto& t = inTriangle;
  const auto& p = inPoint;

  const auto v0 = (t[1] - t[0]);
  const auto v1 = (t[2] - t[0]);
  const auto v2 = (p - t[0]);
  const auto dot_00 = Dot(v0, v0);
  const auto dot_01 = Dot(v0, v1);
  const auto dot_11 = Dot(v1, v1);
  const auto dot_20 = Dot(v2, v0);
  const auto dot_21 = Dot(v2, v1);
  const auto denominator = dot_00 * dot_11 - dot_01 * dot_01;
  const auto inv_denominator = static_cast<T>(1) / denominator;
  const auto v = (dot_11 * dot_20 - dot_01 * dot_21) * inv_denominator;
  const auto w = (dot_00 * dot_21 - dot_01 * dot_20) * inv_denominator;
  const auto u = static_cast<T>(1) - v - w;

  return Vec3<T>(u, v, w);
}
}