#pragma once

#include "ez/CommonTypeTraits.h"
#include <type_traits>

namespace ez
{
// IsVec. Template specialization for Vec is in "Vec.h"
template <typename T>
struct IsVec final : std::false_type
{
};
template <typename T>
constexpr bool IsVec_v = IsVec<std::remove_cvref_t<T>>::value;

// IsMat. Template specialization for Mat is in "Mat.h"
template <typename T>
struct IsMat final : std::false_type
{
};
template <typename T>
inline constexpr bool IsMat_v = IsMat<std::remove_cvref_t<T>>::value;

// IsQuat. Template specialization for Quat is in "Quat.h"
template <typename T>
struct IsQuat final : std::false_type
{
};
template <typename T>
inline constexpr bool IsQuat_v = IsQuat<std::remove_cvref_t<T>>::value;

// IsLine. Template specialization for Line is in "Line.h"
template <typename T>
struct IsLine final : std::false_type
{
};
template <typename T>
constexpr bool IsLine_v = IsLine<std::remove_cvref_t<T>>::value;

// IsRay. Template specialization for Ray is in "Ray.h"
template <typename T>
struct IsRay final : std::false_type
{
};
template <typename T>
constexpr bool IsRay_v = IsRay<std::remove_cvref_t<T>>::value;

// IsSegment. Template specialization for Segment is in "Segment.h"
template <typename T>
struct IsSegment final : std::false_type
{
};
template <typename T>
constexpr bool IsSegment_v = IsSegment<std::remove_cvref_t<T>>::value;

// IsPlane. Template specialization for Plane is in "Plane.h"
template <typename T>
struct IsPlane final : std::false_type
{
};
template <typename T>
constexpr bool IsPlane_v = IsPlane<std::remove_cvref_t<T>>::value;

// IsAAHyperBox. Template specialization for AAHyperBox is in "AAHyperBox.h"
template <typename T>
struct IsAAHyperBox final : std::false_type
{
};
template <typename T>
constexpr bool IsAAHyperBox_v = IsAAHyperBox<std::remove_cvref_t<T>>::value;

// IsTransformation. Template specialization for Transformation is in "Transformation.h"
template <typename T>
struct IsTransformation final : std::false_type
{
};
template <typename T>
inline constexpr bool IsTransformation_v = IsTransformation<std::remove_cvref_t<T>>::value;

// IsAAHyperCube. Template specialization for AAHyperCube is in "AAHyperCube.h"
template <typename T>
struct IsAAHyperCube final : std::false_type
{
};
template <typename T>
constexpr bool IsAAHyperCube_v = IsAAHyperCube<std::remove_cvref_t<T>>::value;

// IsHyperSphere. Template specialization for HyperSphere is in "HyperSphere.h"
template <typename T>
struct IsHyperSphere final : std::false_type
{
};
template <typename T>
constexpr bool IsHyperSphere_v = IsHyperSphere<std::remove_cvref_t<T>>::value;

// IsCapsule. Template specialization for Capsule is in "Capsule.h"
template <typename T>
struct IsCapsule final : std::false_type
{
};
template <typename T>
constexpr bool IsCapsule_v = IsCapsule<std::remove_cvref_t<T>>::value;

// IsCylinder. Template specialization for Cylinder is in "Cylinder.h"
template <typename T>
struct IsCylinder final : std::false_type
{
};
template <typename T>
constexpr bool IsCylinder_v = IsCylinder<std::remove_cvref_t<T>>::value;

// IsNumber
template <typename T>
constexpr auto IsNumber_v = std::is_arithmetic_v<std::remove_cvref_t<T>>;

// IsVecOrMat
template <typename T>
constexpr auto IsVecOrMat_v = IsVec_v<T> || IsMat_v<T>;

template <typename T>
constexpr auto _GetNumComponents()
{
  if constexpr (IsNumber_v<T>)
  {
    return 1;
  }
  else if constexpr (
      IsVec_v<
          T> || IsQuat_v<T> || IsMat_v<T> || IsLine_v<T> || IsRay_v<T> || IsSegment_v<T> || IsPlane_v<T> || IsAAHyperBox_v<T> || IsAAHyperCube_v<T> || IsHyperSphere_v<T> || IsCylinder_v<T> || IsCapsule_v<T>)
  {
    return T::NumComponents;
  }
  else
  {
    return 0;
  }
}

template <typename... TArgs>
constexpr auto NumComponents_v = (_GetNumComponents<std::remove_cvref_t<TArgs>>() + ... + 0);

template <typename T>
auto _GetValueType()
{
  if constexpr (IsNumber_v<T>)
  {
    return T {};
  }
  else
  {
    return typename T::ValueType {};
  }
}

template <typename T>
using ValueType_t = decltype(_GetValueType<std::remove_cvref_t<T>>());

template <typename T>
constexpr std::size_t _GetNumDimensions()
{
  if constexpr (IsNumber_v<T>)
  {
    return 1;
  }
  else if constexpr (IsSpan_v<T>)
  {
    return _GetNumDimensions<ValueType_t<T>>();
  }
  else
  {
    return T::NumDimensions;
  }
};

template <typename... TArgs>
static constexpr auto NumDimensions_v = (_GetNumDimensions<std::remove_cvref_t<TArgs>>() + ... + 0);

}
