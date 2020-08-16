#pragma once

#include <cstdint>
#include <cstdlib>

namespace ez
{
// Vec
template <typename T, std::size_t N>
class Vec;

template <std::size_t N>
using Vecb = Vec<bool, N>;
template <std::size_t N>
using Veci = Vec<int32_t, N>;
template <std::size_t N>
using Vecui = Vec<uint32_t, N>;
template <std::size_t N>
using Vecul = Vec<uint64_t, N>;
template <std::size_t N>
using Vecf = Vec<float, N>;
template <std::size_t N>
using Vecd = Vec<double, N>;

template <typename T>
using Vec2 = Vec<T, 2>;
using Vec2b = Vec2<bool>;
using Vec2i = Vec2<int32_t>;
using Vec2ui = Vec2<uint32_t>;
using Vec2ul = Vec2<uint64_t>;
using Vec2f = Vec2<float>;
using Vec2d = Vec2<double>;

template <typename T>
using Vec3 = Vec<T, 3>;
using Vec3b = Vec3<bool>;
using Vec3i = Vec3<int32_t>;
using Vec3ui = Vec3<uint32_t>;
using Vec3ul = Vec3<uint64_t>;
using Vec3f = Vec3<float>;
using Vec3d = Vec3<double>;

template <typename T>
using Vec4 = Vec<T, 4>;
using Vec4b = Vec4<bool>;
using Vec4i = Vec4<int32_t>;
using Vec4ui = Vec4<uint32_t>;
using Vec4ul = Vec4<uint64_t>;
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
using Mat2ui = Mat2<uint32_t>;
using Mat2ul = Mat2<uint64_t>;

template <typename T>
using Mat3 = Mat<T, 3, 3>;
using Mat3f = Mat3<float>;
using Mat3d = Mat3<double>;
using Mat3i = Mat3<int32_t>;
using Mat3ui = Mat3<uint32_t>;
using Mat3ul = Mat3<uint64_t>;

template <typename T>
using Mat4 = Mat<T, 4, 4>;
using Mat4f = Mat4<float>;
using Mat4d = Mat4<double>;
using Mat4i = Mat4<int32_t>;
using Mat4ui = Mat4<uint32_t>;
using Mat4ul = Mat4<uint64_t>;

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

// Line
template <typename T, std::size_t N>
class Line;

template <typename T>
using Line2 = Line<T, 2>;
using Line2f = Line2<float>;
using Line2d = Line2<double>;

template <typename T>
using Line3 = Line<T, 3>;
using Line3f = Line3<float>;
using Line3d = Line3<double>;

// Plane
template <typename T>
class Plane;

using Planef = Plane<float>;
using Planed = Plane<double>;

// HyperSphere
template <typename T, std::size_t N>
class HyperSphere;

template <std::size_t N>
using HyperSpheref = HyperSphere<float, N>;
template <std::size_t N>
using HyperSphered = HyperSphere<double, N>;

// Circle
template <typename T>
using Circle = HyperSphere<T, 2>;

using Circlef = Circle<float>;
using Circled = Circle<double>;
using Circlei = Circle<int32_t>;

// Sphere
template <typename T>
using Sphere = HyperSphere<T, 3>;

using Spheref = Sphere<float>;
using Sphered = Sphere<double>;
using Spherei = Sphere<int32_t>;

// Capsule
template <typename T, std::size_t N>
class Capsule;

template <typename T>
using Capsule2 = Capsule<T, 2>;
using Capsule2f = Capsule2<float>;
using Capsule2d = Capsule2<double>;

template <typename T>
using Capsule3 = Capsule<T, 3>;
using Capsule3f = Capsule3<float>;
using Capsule3d = Capsule3<double>;

// Cylinder
template <typename T>
class Cylinder;

using Cylinderf = Cylinder<float>;
using Cylinderd = Cylinder<double>;
using Cylinderi = Cylinder<int32_t>;

// AAHyperBox
template <typename T, std::size_t N>
class AAHyperBox;

// AARect
template <typename T>
using AARect = AAHyperBox<T, 2>;

using AARecti = AARect<int32_t>;
using AARectui = AARect<uint32_t>;
using AARectul = AARect<uint64_t>;
using AARectf = AARect<float>;
using AARectd = AARect<double>;

// AABox
template <typename T>
using AABox = AAHyperBox<T, 3>;

using AABoxi = AABox<int32_t>;
using AABoxui = AABox<uint32_t>;
using AABoxul = AABox<uint64_t>;
using AABoxf = AABox<float>;
using AABoxd = AABox<double>;

// AAHyperCube
template <typename T, std::size_t N>
class AAHyperCube;

// AASquare
template <typename T>
using AASquare = AAHyperCube<T, 2>;

using AASquarei = AASquare<int32_t>;
using AASquareui = AASquare<uint32_t>;
using AASquareul = AASquare<uint64_t>;
using AASquaref = AASquare<float>;
using AASquared = AASquare<double>;

// AACube
template <typename T>
using AACube = AAHyperCube<T, 3>;

using AACubei = AACube<int32_t>;
using AACubeui = AACube<uint32_t>;
using AACubeul = AACube<uint64_t>;
using AACubef = AACube<float>;
using AACubed = AACube<double>;

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
using Segment2ui = Segment2<uint32_t>;
using Segment2f = Segment2<float>;
using Segment2d = Segment2<double>;

template <typename T>
using Segment3 = Segment<T, 3>;
using Segment3b = Segment3<bool>;
using Segment3i = Segment3<int32_t>;
using Segment3ui = Segment3<uint32_t>;
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
using Color3ub = Color3<uint8_t>;

template <typename T>
using Color4 = Color<T, 4>;
using Color4f = Color4<float>;
using Color4d = Color4<double>;
using Color4ub = Color4<uint8_t>;
}