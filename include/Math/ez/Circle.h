#pragma once

#include "ez/HyperSphere.h"

namespace ez
{
template <typename T>
using Circle = HyperSphere<T, 2>;

using Circlef = Circle<float>;
using Circled = Circle<double>;
}