#include "ez/Rect.h"

namespace ez
{

template <typename T>
Rect<T>::Rect()
{
}

template <typename T>
Rect<T>::Rect(const Vec2<T>& inMin, const Vec2<T>& inMax) : mMin(inMin), mMax(inMax)
{
  EXPECTS(mMin <= mMax);
}

template <typename T>
void Rect<T>::SetMin(const Vec2<T>& inMin)
{
  EXPECTS(inMin <= mMax);
  mMin = inMin;
}

template <typename T>
void Rect<T>::SetMax(const Vec2<T>& inMax)
{
  EXPECTS(inMax <= mMax);
  mMax = inMax;
}

template <typename T>
void Rect<T>::SetMinMax(const Vec2<T>& inMin, const Vec2<T>& inMax)
{
  EXPECTS(inMin <= inMax);
  mMin = inMin;
  mMax = inMax;
}
}