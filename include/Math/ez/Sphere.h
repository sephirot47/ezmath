#pragma once

#include "ez/HyperSphere.h"

namespace ez
{
template <typename T>
using Sphere = HyperSphere<T, 3>;

using Spheref = Sphere<float>;
using Sphered = Sphere<double>;
}