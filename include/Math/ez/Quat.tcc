#include "ez/Macros.h"
#include "ez/Quat.h"
#include <cmath>

namespace ez
{
template <typename T>
constexpr typename std::array<T, 4>::iterator Quat<T>::begin()
{
  return std::begin(mComponents);
}

template <typename T>
constexpr typename std::array<T, 4>::iterator Quat<T>::end()
{
  return std::end(mComponents);
}
template <typename T>
constexpr typename std::array<T, 4>::const_iterator Quat<T>::begin() const
{
  return std::begin(mComponents);
}

template <typename T>
constexpr typename std::array<T, 4>::const_iterator Quat<T>::end() const
{
  return std::end(mComponents);
}

template <typename T>
constexpr typename std::array<T, 4>::const_iterator Quat<T>::cbegin() const
{
  return std::cbegin(mComponents);
}

template <typename T>
constexpr typename std::array<T, 4>::const_iterator Quat<T>::cend() const
{
  return std::cend(mComponents);
}

template <typename T>
constexpr bool Quat<T>::operator==(const Quat<T>& inRHS) const
{
  return mComponents == inRHS.mComponents;
}

template <typename T>
constexpr bool Quat<T>::operator!=(const Quat<T>& inRHS) const
{
  return mComponents != inRHS.mComponents;
}

template <typename T>
constexpr T& Quat<T>::operator[](std::size_t i)
{
  return mComponents[i];
}

template <typename T>
constexpr const T& Quat<T>::operator[](std::size_t i) const
{
  return mComponents[i];
}

template <typename T>
constexpr Quat<T> Quat<T>::operator+(const Quat<T>& inRHS) const
{
  return { (*this)[0] + inRHS[0], (*this)[1] + inRHS[1], (*this)[2] + inRHS[2], (*this)[3] + inRHS[3] };
}

template <typename T>
constexpr Quat<T> Quat<T>::operator-(const Quat<T>& inRHS) const
{
  return { (*this)[0] - inRHS[0], (*this)[1] - inRHS[1], (*this)[2] - inRHS[2], (*this)[3] - inRHS[3] };
}

template <typename T>
constexpr Quat<T> Quat<T>::operator*(const T& inRHS) const
{
  return { (*this)[0] * inRHS[0], (*this)[1] * inRHS[1], (*this)[2] * inRHS[2], (*this)[3] * inRHS[3] };
}

template <typename T>
constexpr Quat<T> Quat<T>::operator*(const Quat<T>& inRHS) const
{
  return { (*this)[3] * inRHS[0] + (*this)[0] * inRHS[3] + (*this)[1] * inRHS[2] - (*this)[2] * inRHS[1],
    (*this)[3] * inRHS[1] + (*this)[1] * inRHS[3] + (*this)[2] * inRHS[0] - (*this)[0] * inRHS[2],
    (*this)[3] * inRHS[2] + (*this)[2] * inRHS[3] + (*this)[0] * inRHS[1] - (*this)[1] * inRHS[0],
    (*this)[3] * inRHS[3] - (*this)[0] * inRHS[0] - (*this)[1] * inRHS[1] - (*this)[2] * inRHS[2] };
}

template <typename T>
constexpr Vec3<T> Quat<T>::operator*(const Vec3<T>& inRHS) const
{
  const auto q_vector = Vec3<T> { (*this)[0], (*this)[1], (*this)[2] };
  const auto uv(Cross(q_vector, inRHS));
  const auto uuv(Cross(q_vector, uv));
  return inRHS + ((uv * (*this)[3]) + uuv) * static_cast<T>(2);
}

template <typename T>
constexpr Vec4<T> Quat<T>::operator*(const Vec4<T>& inRHS) const
{
  const auto v3 = (*this) * Vec3<T> { inRHS[0], inRHS[1], inRHS[2] };
  return { v3[0], v3[1], v3[2], inRHS[3] };
}

template <typename T>
constexpr Quat<T> Quat<T>::operator/(const T& inRHS) const
{
  return { (*this)[0] / inRHS, (*this)[1] / inRHS, (*this)[2] / inRHS, (*this)[3] / inRHS };
}

template <typename T>
void Quat<T>::operator+=(const Quat<T>& inRHS)
{
  *this = *this + inRHS;
}

template <typename T>
void Quat<T>::operator-=(const Quat<T>& inRHS)
{
  *this = *this - inRHS;
}

template <typename T>
void Quat<T>::operator*=(const T& inRHS)
{
  *this = *this * inRHS;
}

template <typename T>
void Quat<T>::operator*=(const Quat<T>& inRHS)
{
  *this = *this * inRHS;
}

template <typename T>
void Quat<T>::operator/=(const T& inRHS)
{
  *this = *this / inRHS;
}

template <typename T>
constexpr Quat<T> Quat<T>::operator-() const
{
  return { -(*this)[0], -(*this)[1], -(*this)[2], (*this)[3] };
}

template <typename T>
std::ostream& operator<<(std::ostream& log, const Quat<T>& q)
{
  log << "(" << q[0] << ", " << q[1] << ", " << q[2] << ", " << q[3] << ")";
  return log;
}
}
