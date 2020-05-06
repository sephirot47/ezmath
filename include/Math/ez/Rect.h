#pragma once

#include "ez/Vec.h"
#include "ez/MathInitializers.h"

namespace ez
{

template <typename T>
class Rect final
{
public:
  Rect();
  Rect(const Vec2<T>& inMin, const Vec2<T>& inMax);
  Rect(const Rect&) = default;
  Rect& operator=(const Rect&) = default;
  Rect(Rect&&) = default;
  Rect& operator=(Rect&&) = default;
  ~Rect() = default;

  void SetMin(const Vec2<T>& inMin);
  void SetMax(const Vec2<T>& inMax);
  void SetMinMax(const Vec2<T>& inMin, const Vec2<T>& inMax);
  Vec2<T> GetSize() const { return (mMax - mMin); }
  const Vec2<T>& GetMin() const { return mMin; }
  const Vec2<T>& GetMax() const { return mMax; }

  bool operator==(const Rect& inRHS) const { return mMin == inRHS.mMin && mMax == inRHS.mMax; }
  bool operator!=(const Rect& inRHS) const { return !(*this == inRHS); }
  bool operator<(const Rect& inRHS) const { return mMin < inRHS.mMin && mMax < inRHS.mMax; }
  bool operator<=(const Rect& inRHS) const { return mMin <= inRHS.mMin && mMax <= inRHS.mMax; }
  bool operator>(const Rect& inRHS) const { return mMin > inRHS.mMin && mMax > inRHS.mMax; }
  bool operator>=(const Rect& inRHS) const { return mMin >= inRHS.mMin && mMax >= inRHS.mMax; }
  bool operator<(const Vec2<T>& inRHS) const { return mMin < inRHS && mMax < inRHS; }
  bool operator<=(const Vec2<T>& inRHS) const { return mMin <= inRHS && mMax <= inRHS; }
  bool operator>(const Vec2<T>& inRHS) const { return mMin > inRHS && mMax > inRHS; }
  bool operator>=(const Vec2<T>& inRHS) const { return mMin >= inRHS && mMax >= inRHS; }

private:
  Vec2<T> mMin = Zero<Vec2<T>>();
  Vec2<T> mMax = Zero<Vec2<T>>();
};

using Recti = Rect<int>;
using Rectf = Rect<float>;
using Rectd = Rect<double>;

}

#include "ez/Rect.tcc"