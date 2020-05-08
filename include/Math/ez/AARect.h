#pragma once

#include "AAHyperRectangle.h"

namespace ez
{
template <typename T>
using AARect = AAHyperRectangle<T, 2>;

using AARecti = AARect<int>;
using AARectf = AARect<float>;
using AARectd = AARect<double>;
}
