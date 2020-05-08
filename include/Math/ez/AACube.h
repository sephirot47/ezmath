#pragma once

#include "AAHyperRectangle.h"

namespace ez
{
template <typename T>
using AACube = AAHyperRectangle<T, 3>;

using AACubei = AACube<int>;
using AACubef = AACube<float>;
using AACubed = AACube<double>;
};