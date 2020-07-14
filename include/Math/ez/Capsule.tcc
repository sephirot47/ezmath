#include "ez/Capsule.h"

namespace ez
{
template <typename T>
Capsule<T>::Capsule(const Vec3<T>& inOrigin, const Vec3<T>& inDestiny, const T inRadius)
    : mOrigin { inOrigin }, mDestiny { inDestiny }, mRadius { inRadius }
{
}
}