#pragma once

#include <ez/Vec.h>
#include <cstdint>

namespace ez
{
// clang-format off
template <typename T, std::size_t N> constexpr Vec2<T> XX(const Vec<T, N> &inV) { return Vec2<T>{inV[0], inV[0]}; }
template <typename T, std::size_t N> constexpr Vec2<T> XY(const Vec<T, N> &inV) { return Vec2<T>{inV[0], inV[1]}; }
template <typename T, std::size_t N> constexpr Vec2<T> XZ(const Vec<T, N> &inV) { return Vec2<T>{inV[0], inV[2]}; }
template <typename T, std::size_t N> constexpr Vec2<T> YX(const Vec<T, N> &inV) { return Vec2<T>{inV[1], inV[0]}; }
template <typename T, std::size_t N> constexpr Vec2<T> YY(const Vec<T, N> &inV) { return Vec2<T>{inV[1], inV[1]}; }
template <typename T, std::size_t N> constexpr Vec2<T> YZ(const Vec<T, N> &inV) { return Vec2<T>{inV[1], inV[2]}; }
template <typename T, std::size_t N> constexpr Vec2<T> ZX(const Vec<T, N> &inV) { return Vec2<T>{inV[2], inV[0]}; }
template <typename T, std::size_t N> constexpr Vec2<T> ZY(const Vec<T, N> &inV) { return Vec2<T>{inV[2], inV[1]}; }
template <typename T, std::size_t N> constexpr Vec2<T> ZZ(const Vec<T, N> &inV) { return Vec2<T>{inV[2], inV[2]}; }
template <typename T, std::size_t N> constexpr Vec2<T> X0(const Vec<T, N> &inV) { return Vec2<T>{inV[0], static_cast<T>(0)}; }
template <typename T, std::size_t N> constexpr Vec2<T> Y0(const Vec<T, N> &inV) { return Vec2<T>{inV[1], static_cast<T>(0)}; }
template <typename T, std::size_t N> constexpr Vec2<T> X1(const Vec<T, N> &inV) { return Vec2<T>{inV[0], static_cast<T>(1)}; }
template <typename T, std::size_t N> constexpr Vec2<T> Y1(const Vec<T, N> &inV) { return Vec2<T>{inV[1], static_cast<T>(1)}; }

template <typename T, std::size_t N> constexpr Vec3<T> XXX(const Vec<T, N> &inV) { return Vec3<T>{inV[0], inV[0], inV[0]}; }
template <typename T, std::size_t N> constexpr Vec3<T> XXY(const Vec<T, N> &inV) { return Vec3<T>{inV[0], inV[0], inV[1]}; }
template <typename T, std::size_t N> constexpr Vec3<T> XXZ(const Vec<T, N> &inV) { return Vec3<T>{inV[0], inV[0], inV[2]}; }
template <typename T, std::size_t N> constexpr Vec3<T> XYX(const Vec<T, N> &inV) { return Vec3<T>{inV[0], inV[1], inV[0]}; }
template <typename T, std::size_t N> constexpr Vec3<T> XYY(const Vec<T, N> &inV) { return Vec3<T>{inV[0], inV[1], inV[1]}; }
template <typename T, std::size_t N> constexpr Vec3<T> XYZ(const Vec<T, N> &inV) { return Vec3<T>{inV[0], inV[1], inV[2]}; }
template <typename T, std::size_t N> constexpr Vec3<T> XZX(const Vec<T, N> &inV) { return Vec3<T>{inV[0], inV[2], inV[0]}; }
template <typename T, std::size_t N> constexpr Vec3<T> XZY(const Vec<T, N> &inV) { return Vec3<T>{inV[0], inV[2], inV[1]}; }
template <typename T, std::size_t N> constexpr Vec3<T> XZZ(const Vec<T, N> &inV) { return Vec3<T>{inV[0], inV[2], inV[2]}; }
template <typename T, std::size_t N> constexpr Vec3<T> YXX(const Vec<T, N> &inV) { return Vec3<T>{inV[1], inV[0], inV[0]}; }
template <typename T, std::size_t N> constexpr Vec3<T> YXY(const Vec<T, N> &inV) { return Vec3<T>{inV[1], inV[0], inV[1]}; }
template <typename T, std::size_t N> constexpr Vec3<T> YXZ(const Vec<T, N> &inV) { return Vec3<T>{inV[1], inV[0], inV[2]}; }
template <typename T, std::size_t N> constexpr Vec3<T> YYX(const Vec<T, N> &inV) { return Vec3<T>{inV[1], inV[1], inV[0]}; }
template <typename T, std::size_t N> constexpr Vec3<T> YYY(const Vec<T, N> &inV) { return Vec3<T>{inV[1], inV[1], inV[1]}; }
template <typename T, std::size_t N> constexpr Vec3<T> YYZ(const Vec<T, N> &inV) { return Vec3<T>{inV[1], inV[1], inV[2]}; }
template <typename T, std::size_t N> constexpr Vec3<T> YZX(const Vec<T, N> &inV) { return Vec3<T>{inV[1], inV[2], inV[0]}; }
template <typename T, std::size_t N> constexpr Vec3<T> YZY(const Vec<T, N> &inV) { return Vec3<T>{inV[1], inV[2], inV[1]}; }
template <typename T, std::size_t N> constexpr Vec3<T> YZZ(const Vec<T, N> &inV) { return Vec3<T>{inV[1], inV[2], inV[2]}; }
template <typename T, std::size_t N> constexpr Vec3<T> ZXX(const Vec<T, N> &inV) { return Vec3<T>{inV[2], inV[0], inV[0]}; }
template <typename T, std::size_t N> constexpr Vec3<T> ZXY(const Vec<T, N> &inV) { return Vec3<T>{inV[2], inV[0], inV[1]}; }
template <typename T, std::size_t N> constexpr Vec3<T> ZXZ(const Vec<T, N> &inV) { return Vec3<T>{inV[2], inV[0], inV[2]}; }
template <typename T, std::size_t N> constexpr Vec3<T> ZYX(const Vec<T, N> &inV) { return Vec3<T>{inV[2], inV[1], inV[0]}; }
template <typename T, std::size_t N> constexpr Vec3<T> ZYY(const Vec<T, N> &inV) { return Vec3<T>{inV[2], inV[1], inV[1]}; }
template <typename T, std::size_t N> constexpr Vec3<T> ZYZ(const Vec<T, N> &inV) { return Vec3<T>{inV[2], inV[1], inV[2]}; }
template <typename T, std::size_t N> constexpr Vec3<T> ZZX(const Vec<T, N> &inV) { return Vec3<T>{inV[2], inV[2], inV[0]}; }
template <typename T, std::size_t N> constexpr Vec3<T> ZZY(const Vec<T, N> &inV) { return Vec3<T>{inV[2], inV[2], inV[1]}; }
template <typename T, std::size_t N> constexpr Vec3<T> ZZZ(const Vec<T, N> &inV) { return Vec3<T>{inV[2], inV[2], inV[2]}; }
template <typename T, std::size_t N> constexpr Vec3<T> XX0(const Vec<T, N> &inV) { return Vec3<T>{inV[0], inV[0], static_cast<T>(0)}; }
template <typename T, std::size_t N> constexpr Vec3<T> XY0(const Vec<T, N> &inV) { return Vec3<T>{inV[0], inV[1], static_cast<T>(0)}; }
template <typename T, std::size_t N> constexpr Vec3<T> XZ0(const Vec<T, N> &inV) { return Vec3<T>{inV[0], inV[2], static_cast<T>(0)}; }
template <typename T, std::size_t N> constexpr Vec3<T> YX0(const Vec<T, N> &inV) { return Vec3<T>{inV[1], inV[0], static_cast<T>(0)}; }
template <typename T, std::size_t N> constexpr Vec3<T> YY0(const Vec<T, N> &inV) { return Vec3<T>{inV[1], inV[1], static_cast<T>(0)}; }
template <typename T, std::size_t N> constexpr Vec3<T> YZ0(const Vec<T, N> &inV) { return Vec3<T>{inV[1], inV[2], static_cast<T>(0)}; }
template <typename T, std::size_t N> constexpr Vec3<T> ZX0(const Vec<T, N> &inV) { return Vec3<T>{inV[2], inV[0], static_cast<T>(0)}; }
template <typename T, std::size_t N> constexpr Vec3<T> ZY0(const Vec<T, N> &inV) { return Vec3<T>{inV[2], inV[1], static_cast<T>(0)}; }
template <typename T, std::size_t N> constexpr Vec3<T> ZZ0(const Vec<T, N> &inV) { return Vec3<T>{inV[2], inV[2], static_cast<T>(0)}; }
template <typename T, std::size_t N> constexpr Vec3<T> XX1(const Vec<T, N> &inV) { return Vec3<T>{inV[0], inV[0], static_cast<T>(1)}; }
template <typename T, std::size_t N> constexpr Vec3<T> XY1(const Vec<T, N> &inV) { return Vec3<T>{inV[0], inV[1], static_cast<T>(1)}; }
template <typename T, std::size_t N> constexpr Vec3<T> XZ1(const Vec<T, N> &inV) { return Vec3<T>{inV[0], inV[2], static_cast<T>(1)}; }
template <typename T, std::size_t N> constexpr Vec3<T> YX1(const Vec<T, N> &inV) { return Vec3<T>{inV[1], inV[0], static_cast<T>(1)}; }
template <typename T, std::size_t N> constexpr Vec3<T> YY1(const Vec<T, N> &inV) { return Vec3<T>{inV[1], inV[1], static_cast<T>(1)}; }
template <typename T, std::size_t N> constexpr Vec3<T> YZ1(const Vec<T, N> &inV) { return Vec3<T>{inV[1], inV[2], static_cast<T>(1)}; }
template <typename T, std::size_t N> constexpr Vec3<T> ZX1(const Vec<T, N> &inV) { return Vec3<T>{inV[2], inV[0], static_cast<T>(1)}; }
template <typename T, std::size_t N> constexpr Vec3<T> ZY1(const Vec<T, N> &inV) { return Vec3<T>{inV[2], inV[1], static_cast<T>(1)}; }
template <typename T, std::size_t N> constexpr Vec3<T> ZZ1(const Vec<T, N> &inV) { return Vec3<T>{inV[2], inV[2], static_cast<T>(1)}; }
template <typename T, std::size_t N> constexpr Vec3<T> X0X(const Vec<T, N> &inV) { return Vec3<T>{inV[0], static_cast<T>(0), inV[0]}; }
template <typename T, std::size_t N> constexpr Vec3<T> X0Y(const Vec<T, N> &inV) { return Vec3<T>{inV[0], static_cast<T>(0), inV[1]}; }
template <typename T, std::size_t N> constexpr Vec3<T> X0Z(const Vec<T, N> &inV) { return Vec3<T>{inV[0], static_cast<T>(0), inV[2]}; }
template <typename T, std::size_t N> constexpr Vec3<T> Y0X(const Vec<T, N> &inV) { return Vec3<T>{inV[1], static_cast<T>(0), inV[0]}; }
template <typename T, std::size_t N> constexpr Vec3<T> Y0Y(const Vec<T, N> &inV) { return Vec3<T>{inV[1], static_cast<T>(0), inV[1]}; }
template <typename T, std::size_t N> constexpr Vec3<T> Y0Z(const Vec<T, N> &inV) { return Vec3<T>{inV[1], static_cast<T>(0), inV[2]}; }
template <typename T, std::size_t N> constexpr Vec3<T> Z0X(const Vec<T, N> &inV) { return Vec3<T>{inV[2], static_cast<T>(0), inV[0]}; }
template <typename T, std::size_t N> constexpr Vec3<T> Z0Y(const Vec<T, N> &inV) { return Vec3<T>{inV[2], static_cast<T>(0), inV[1]}; }
template <typename T, std::size_t N> constexpr Vec3<T> Z0Z(const Vec<T, N> &inV) { return Vec3<T>{inV[2], static_cast<T>(0), inV[2]}; }
template <typename T, std::size_t N> constexpr Vec3<T> X1X(const Vec<T, N> &inV) { return Vec3<T>{inV[0], static_cast<T>(1), inV[0]}; }
template <typename T, std::size_t N> constexpr Vec3<T> X1Y(const Vec<T, N> &inV) { return Vec3<T>{inV[0], static_cast<T>(1), inV[1]}; }
template <typename T, std::size_t N> constexpr Vec3<T> X1Z(const Vec<T, N> &inV) { return Vec3<T>{inV[0], static_cast<T>(1), inV[2]}; }
template <typename T, std::size_t N> constexpr Vec3<T> Y1X(const Vec<T, N> &inV) { return Vec3<T>{inV[1], static_cast<T>(1), inV[0]}; }
template <typename T, std::size_t N> constexpr Vec3<T> Y1Y(const Vec<T, N> &inV) { return Vec3<T>{inV[1], static_cast<T>(1), inV[1]}; }
template <typename T, std::size_t N> constexpr Vec3<T> Y1Z(const Vec<T, N> &inV) { return Vec3<T>{inV[1], static_cast<T>(1), inV[2]}; }
template <typename T, std::size_t N> constexpr Vec3<T> Z1X(const Vec<T, N> &inV) { return Vec3<T>{inV[2], static_cast<T>(1), inV[0]}; }
template <typename T, std::size_t N> constexpr Vec3<T> Z1Y(const Vec<T, N> &inV) { return Vec3<T>{inV[2], static_cast<T>(1), inV[1]}; }
template <typename T, std::size_t N> constexpr Vec3<T> Z1Z(const Vec<T, N> &inV) { return Vec3<T>{inV[2], static_cast<T>(1), inV[2]}; }

template <typename T, std::size_t N> constexpr Vec4<T> XYZ0(const Vec<T, N> &inV) { return Vec4<T>{inV[0], inV[1], inV[2], static_cast<T>(0)}; }
template <typename T, std::size_t N> constexpr Vec4<T> XYZ1(const Vec<T, N> &inV) { return Vec4<T>{inV[0], inV[1], inV[2], static_cast<T>(1)}; }
// clang-format on
}