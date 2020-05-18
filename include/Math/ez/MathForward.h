#pragma once

#include <cstdint>
#include <cstdlib>

namespace ez
{
// Vec
template <typename T, std::size_t N>
class Vec;

template <typename T>
using Vec2 = Vec<T, 2>;
using Vec2b = Vec2<bool>;
using Vec2i = Vec2<int32_t>;
using Vec2u = Vec2<uint32_t>;
using Vec2f = Vec2<float>;
using Vec2d = Vec2<double>;

template <typename T>
using Vec3 = Vec<T, 3>;
using Vec3b = Vec3<bool>;
using Vec3i = Vec3<int32_t>;
using Vec3u = Vec3<uint32_t>;
using Vec3f = Vec3<float>;
using Vec3d = Vec3<double>;

template <typename T>
using Vec4 = Vec<T, 4>;
using Vec4b = Vec4<bool>;
using Vec4i = Vec4<int32_t>;
using Vec4u = Vec4<uint32_t>;
using Vec4f = Vec4<float>;
using Vec4d = Vec4<double>;

// Quat
template <typename T>
class Quat;

using Quati = Quat<int>;
using Quatf = Quat<float>;
using Quatd = Quat<double>;

// Mat
template <typename T, std::size_t NRows, std::size_t NCols>
class Mat;

template <typename T, std::size_t N>
using SquareMat = Mat<T, N, N>;

template <typename T>
using Mat2 = Mat<T, 2, 2>;
using Mat2f = Mat2<float>;
using Mat2d = Mat2<double>;
using Mat2i = Mat2<int32_t>;

template <typename T>
using Mat3 = Mat<T, 3, 3>;
using Mat3f = Mat3<float>;
using Mat3d = Mat3<double>;
using Mat3i = Mat3<int32_t>;

template <typename T>
using Mat4 = Mat<T, 4, 4>;
using Mat4f = Mat4<float>;
using Mat4d = Mat4<double>;
using Mat4i = Mat4<int32_t>;

// Ray
template <typename T, std::size_t N>
class Ray;

template <typename T>
using Ray2 = Ray<T, 2>;
using Ray2f = Ray2<float>;
using Ray2d = Ray2<double>;

template <typename T>
using Ray3 = Ray<T, 3>;
using Ray3f = Ray3<float>;
using Ray3d = Ray3<double>;

// Plane
template <typename T>
class Plane;

using Planef = Plane<float>;
using Planed = Plane<double>;

// HyperSphere
template <typename T, std::size_t N>
class HyperSphere;

// Circle
template <typename T>
using Circle = HyperSphere<T, 2>;

using Circlef = Circle<float>;
using Circled = Circle<double>;

// Sphere
template <typename T>
using Sphere = HyperSphere<T, 3>;

using Spheref = Sphere<float>;
using Sphered = Sphere<double>;

// AAHyperBox
template <typename T, std::size_t N>
class AAHyperBox;

// AARect
template <typename T>
using AARect = AAHyperBox<T, 2>;

using AARecti = AARect<int>;
using AARectf = AARect<float>;
using AARectd = AARect<double>;

// AABox
template <typename T>
using AABox = AAHyperBox<T, 3>;

using AABoxi = AABox<int>;
using AABoxf = AABox<float>;
using AABoxd = AABox<double>;

// Octree
template <typename TPrimitive>
class Octree;

// Segment
template <typename T, std::size_t N>
class Segment;

template <typename T>
using Segment2 = Segment<T, 2>;
using Segment2b = Segment2<bool>;
using Segment2i = Segment2<int32_t>;
using Segment2u = Segment2<uint32_t>;
using Segment2f = Segment2<float>;
using Segment2d = Segment2<double>;

template <typename T>
using Segment3 = Segment<T, 3>;
using Segment3b = Segment3<bool>;
using Segment3i = Segment3<int32_t>;
using Segment3u = Segment3<uint32_t>;
using Segment3f = Segment3<float>;
using Segment3d = Segment3<double>;

// Transformation
template <typename T, std::size_t N>
class Transformation;

template <typename T>
using Transformation2 = Transformation<T, 2>;
using Transformation2f = Transformation2<float>;

template <typename T>
using Transformation3 = Transformation<T, 3>;
using Transformation3f = Transformation3<float>;

// Triangle
template <typename T, std::size_t N>
class Triangle;

template <typename T>
using Triangle2 = Triangle<T, 2>;
using Triangle2f = Triangle2<float>;

template <typename T>
using Triangle3 = Triangle<T, 3>;
using Triangle3f = Triangle3<float>;

// Color
template <typename T, ::std::size_t N>
using Color = Vec<T, N>;

template <typename T>
using Color3 = Color<T, 3>;
using Color3f = Color3<float>;
using Color3d = Color3<double>;

template <typename T>
using Color4 = Color<T, 4>;
using Color4f = Color4<float>;
using Color4d = Color4<double>;
}