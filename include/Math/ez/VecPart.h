#pragma once

#include <ez/Macros.h>
#include <ez/VariadicRepeat.h>
#include <array>
#include <cstdlib>

namespace ez
{
template <typename T, std::size_t N>
class Vec;

template <typename T, std::size_t N>
class VecPart final
{
public:
  static_assert(N >= 1);
  using ValueType = T;
  static constexpr auto NumComponents = N;

  template <std::size_t NOther, std::size_t NBegin, std::size_t NEnd>
  constexpr VecPart(Vec<T, NOther>& ioRHS)
  {
    *this = Create<NOther, NBegin, NEnd>(ioRHS);
  }

  template <std::size_t NOther, std::size_t NBegin, std::size_t NEnd>
  constexpr static VecPart<T, N> Create(Vec<T, NOther>& ioRHS)
  {
    static_assert(NEnd - NBegin == N);
    static_assert(NBegin <= NOther);
    static_assert(NEnd <= NOther);

    VecPart part;
    for (std::size_t i = 0; i < N; ++i) { part.mComponentsPtrs[i] = &ioRHS[i + NBegin]; }
    return part;
  }

  constexpr VecPart(Vec<T, N>&& ioRHS) = delete;

  constexpr VecPart(const VecPart&) = default;
  constexpr VecPart& operator=(const VecPart&) = default;
  constexpr VecPart(VecPart&&) = default;

  constexpr VecPart& operator=(VecPart&&) = default;
  ~VecPart() = default;

  constexpr operator Vec<T, N>()
  {
    Vec<T, N> vec;
    for (std::size_t i = 0; i < N; ++i) { vec[i] = *mComponentsPtrs[i]; }
    return vec;
  }

  constexpr VecPart& operator=(const Vec<T, N>& inRHS)
  {
    for (std::size_t i = 0; i < N; ++i) { *mComponentsPtrs[i] = inRHS[i]; }
    return *this;
  }

  constexpr Vec<T, N> operator+(const Vec<T, N>& inRHS) { return (Vec<T, N>(*this) + inRHS); }
  constexpr Vec<T, N> operator-(const Vec<T, N>& inRHS) { return (Vec<T, N>(*this) - inRHS); }
  constexpr Vec<T, N> operator*(const Vec<T, N>& inRHS) { return (Vec<T, N>(*this) * inRHS); }
  constexpr Vec<T, N> operator/(const Vec<T, N>& inRHS) { return (Vec<T, N>(*this) / inRHS); }
  constexpr Vec<T, N> operator+(const VecPart<T, N>& inRHS) { return ((*this) + Vec<T, N>(inRHS)); }
  constexpr Vec<T, N> operator-(const VecPart<T, N>& inRHS) { return ((*this) - Vec<T, N>(inRHS)); }
  constexpr Vec<T, N> operator*(const VecPart<T, N>& inRHS) { return ((*this) * Vec<T, N>(inRHS)); }
  constexpr Vec<T, N> operator/(const VecPart<T, N>& inRHS) { return ((*this) / Vec<T, N>(inRHS)); }
  constexpr Vec<T, N> operator+(const T& inRHS) { return (Vec<T, N>(*this) + inRHS); }
  constexpr Vec<T, N> operator-(const T& inRHS) { return (Vec<T, N>(*this) - inRHS); }
  constexpr Vec<T, N> operator*(const T& inRHS) { return (Vec<T, N>(*this) * inRHS); }
  constexpr Vec<T, N> operator/(const T& inRHS) { return (Vec<T, N>(*this) / inRHS); }

  constexpr VecPart& operator+=(const Vec<T, N>& inRHS) { return (*this = (*this) + inRHS); }
  constexpr VecPart& operator-=(const Vec<T, N>& inRHS) { return (*this = (*this) - inRHS); }
  constexpr VecPart& operator*=(const Vec<T, N>& inRHS) { return (*this = (*this) * inRHS); }
  constexpr VecPart& operator/=(const Vec<T, N>& inRHS) { return (*this = (*this) / inRHS); }
  constexpr VecPart& operator+=(const T& inRHS) { return (*this = (*this) + inRHS); }
  constexpr VecPart& operator-=(const T& inRHS) { return (*this = (*this) - inRHS); }
  constexpr VecPart& operator*=(const T& inRHS) { return (*this = (*this) * inRHS); }
  constexpr VecPart& operator/=(const T& inRHS) { return (*this = (*this) / inRHS); }

  constexpr T& operator[](std::size_t i) { return *mComponentsPtrs[i]; }
  constexpr const T& operator[](std::size_t i) const { return *mComponentsPtrs[i]; }

private:
  std::array<T*, N> mComponentsPtrs;

  constexpr VecPart() : mComponentsPtrs { GetArrayWithRepeatedValue<T*, N>(nullptr) } {}
};

// Traits
template <typename T, std::size_t N>
struct IsVec<VecPart<T, N>> : std::true_type
{
};

template <typename T, std::size_t N>
inline constexpr Vec<T, N> operator+(const T& inLHS, const VecPart<T, N>& inRHS)
{
  return inLHS + Vec<T, N>(inRHS);
}
template <typename T, std::size_t N>
inline constexpr Vec<T, N> operator-(const T& inLHS, const VecPart<T, N>& inRHS)
{
  return inLHS + Vec<T, N>(inRHS);
}
template <typename T, std::size_t N>
inline constexpr Vec<T, N> operator/(const T& inLHS, const VecPart<T, N>& inRHS)
{
  return inLHS + Vec<T, N>(inRHS);
}
template <typename T, std::size_t N>
inline constexpr Vec<T, N> operator*(const T& inLHS, const VecPart<T, N>& inRHS)
{
  return inLHS + Vec<T, N>(inRHS);
}
template <typename T, std::size_t N>
inline constexpr std::ostream& operator<<(std::ostream& ioLHS, const VecPart<T, N>& inRHS)
{
  return (ioLHS << Vec<T, N>(inRHS));
}

template <typename T, std::size_t N>
struct MaybeVec
{
  using type = Vec<T, N>;
};

template <typename T>
struct MaybeVec<T, 1ul>
{
  using type = T;
};

template <typename T, std::size_t N>
using MaybeVec_t = typename MaybeVec<T, N>::type;

template <std::size_t NBegin, std::size_t NEnd, typename T, std::size_t N>
constexpr Vec<T, N> WithPart(const Vec<T, N>& inLHS, const MaybeVec_t<T, (NEnd - NBegin)>& inNewPart)
{
  auto vec = inLHS;
  for (std::size_t i = NBegin; i < NEnd; ++i)
  {
    if constexpr (IsNumber_v<decltype(inNewPart)>)
      vec[i] = inNewPart;
    else
      vec[i] = inNewPart[i - NBegin];
  }
  return vec;
}

template <typename T, std::size_t N, std::size_t NPart>
constexpr Vec<T, N> WithPart(const Vec<T, N>& inLHS,
    const std::size_t inBegin,
    const std::size_t inEnd,
    const MaybeVec_t<T, NPart>& inNewPart)
{
  assert((inEnd - inBegin) == NPart);

  auto vec = inLHS;
  for (std::size_t i = inBegin; i < inEnd; ++i)
  {
    if constexpr (IsNumber_v<decltype(inNewPart)>)
      vec[i] = inNewPart;
    else
      vec[i] = inNewPart[i - inBegin];
  }
  return vec;
}

template <std::size_t NBegin, std::size_t NEnd, typename T, std::size_t N>
constexpr auto Part(const Vec<T, N>& inRHS)
{
  static_assert(NBegin < N);
  static_assert(NEnd <= N);
  static_assert(NEnd > NBegin);

  constexpr auto PartN = (NEnd - NBegin);
  static_assert(PartN >= 1);

  if constexpr (PartN == 1)
  {
    return inRHS[NBegin];
  }
  else
  {
    Vec<T, PartN> vec;
    for (std::size_t i = 0; i < PartN; ++i) { vec[i] = inRHS[i + NBegin]; }
    return vec;
  }
}

template <std::size_t NBegin, std::size_t NEnd, typename T, std::size_t N>
constexpr auto Part(Vec<T, N>&& ioRHS)
{
  return Part<NBegin, NEnd, T, N>(static_cast<const Vec<T, N>&>(ioRHS));
}

template <std::size_t NBegin, std::size_t NEnd, typename T, std::size_t N>
constexpr decltype(auto) Part(Vec<T, N>& ioRHS)
{
  if constexpr (NBegin == NEnd)
  {
    static_assert(NBegin < N);
    return static_cast<T&>(ioRHS[NBegin]);
  }
  else
  {
    return VecPart<T, (NEnd - NBegin)>::template Create<N, NBegin, NEnd>(ioRHS);
  }
}
}