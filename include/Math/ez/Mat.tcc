#include "ez/Mat.h"
#include "ez/StreamOperators.h"
#include "ez/VariadicRepeat.h"
#include <cmath>

namespace ez
{

template <typename T, std::size_t NRows, std::size_t NCols>
constexpr Mat<T, NRows, NCols>::Mat(const MITAll<T>& inMITAll) noexcept
    : mRows { GetArrayWithRepeatedValue<Vec<T, NCols>, NRows>(Vec<T, NCols> { inMITAll }) }
{
}

template <typename T, std::size_t NRows, std::size_t NCols>
constexpr Mat<T, NRows, NCols>::Mat(const MITMultiplicativeIdentity&) noexcept
{
  static_assert(NRows == NCols, "Multiplicate Identity only supported for square matrices.");
  for (std::size_t r = 0; r < NRows; ++r)
  {
    for (std::size_t c = 0; c < NCols; ++c) { (*this)[r][c] = static_cast<T>(((r == c) ? 1 : 0)); }
  }
}

template <typename T, std::size_t NRows, std::size_t NCols>
template <typename... TArgs, typename>
constexpr Mat<T, NRows, NCols>::Mat(TArgs&&... inArgs) noexcept : mRows { std::forward<TArgs>(inArgs)... }
{
}

template <typename T, std::size_t NRows, std::size_t NCols>
T* Mat<T, NRows, NCols>::Data()
{
  return mRows[0].Data();
}

template <typename T, std::size_t NRows, std::size_t NCols>
const T* Mat<T, NRows, NCols>::Data() const
{
  return mRows[0].Data();
}

template <typename T, std::size_t NRows, std::size_t NCols>
constexpr typename std::array<Vec<T, NCols>, NRows>::iterator Mat<T, NRows, NCols>::begin()
{
  return mRows.begin();
}

template <typename T, std::size_t NRows, std::size_t NCols>
constexpr typename std::array<Vec<T, NCols>, NRows>::iterator Mat<T, NRows, NCols>::end()
{
  return mRows.end();
}

template <typename T, std::size_t NRows, std::size_t NCols>
constexpr typename std::array<Vec<T, NCols>, NRows>::const_iterator Mat<T, NRows, NCols>::begin() const
{
  return mRows.begin();
}

template <typename T, std::size_t NRows, std::size_t NCols>
constexpr typename std::array<Vec<T, NCols>, NRows>::const_iterator Mat<T, NRows, NCols>::end() const
{
  return mRows.end();
}

template <typename T, std::size_t NRows, std::size_t NCols>
constexpr typename std::array<Vec<T, NCols>, NRows>::const_iterator Mat<T, NRows, NCols>::cbegin() const
{
  return mRows.cbegin();
}

template <typename T, std::size_t NRows, std::size_t NCols>
constexpr typename std::array<Vec<T, NCols>, NRows>::const_iterator Mat<T, NRows, NCols>::cend() const
{
  return mRows.cend();
}

template <typename T, std::size_t NRows, std::size_t NCols>
constexpr bool Mat<T, NRows, NCols>::operator==(const Mat& inRHS) const
{
  return mRows == inRHS.mRows;
}

template <typename T, std::size_t NRows, std::size_t NCols>
constexpr bool Mat<T, NRows, NCols>::operator!=(const Mat& inRHS) const
{
  return !(*this == inRHS);
}

template <typename T, std::size_t NRows, std::size_t NCols>
constexpr Vec<T, NCols>& Mat<T, NRows, NCols>::operator[](std::size_t inRow)
{
  return mRows[inRow];
}

template <typename T, std::size_t NRows, std::size_t NCols>
constexpr const Vec<T, NCols>& Mat<T, NRows, NCols>::operator[](std::size_t inRow) const
{
  return mRows[inRow];
}

template <typename T, std::size_t NRows, std::size_t NCols>
constexpr Mat<T, NRows, NCols> Mat<T, NRows, NCols>::operator+(const Mat<T, NRows, NCols>& inRHS) const
{
  Mat<T, NRows, NCols> result;
  for (std::size_t r = 0; r < NRows; ++r)
  {
    for (std::size_t c = 0; c < NCols; ++c) { result[r][c] = (*this)[r][c] + inRHS[r][c]; }
  }
  return result;
}

template <typename T, std::size_t NRows, std::size_t NCols>
void Mat<T, NRows, NCols>::operator+=(const Mat<T, NRows, NCols>& inRHS)
{
  *this = *this + inRHS;
}

template <typename T, std::size_t NRows, std::size_t NCols>
void Mat<T, NRows, NCols>::operator-=(const Mat<T, NRows, NCols>& inRHS)
{
  *this = *this - inRHS;
}

template <typename T, std::size_t NRows, std::size_t NCols>
constexpr Mat<T, NRows, NCols> Mat<T, NRows, NCols>::operator-(const Mat<T, NRows, NCols>& inRHS) const
{
  Mat<T, NRows, NCols> result;
  for (std::size_t r = 0; r < NRows; ++r)
  {
    for (std::size_t c = 0; c < NCols; ++c) { result[r][c] = (*this)[r][c] - inRHS[r][c]; }
  }
  return result;
}
template <typename T, std::size_t NRows, std::size_t NCols>
constexpr Mat<T, NRows, NCols> Mat<T, NRows, NCols>::operator+(const T& inRHS) const
{
  Mat<T, NRows, NCols> result;
  for (std::size_t r = 0; r < NRows; ++r)
  {
    for (std::size_t c = 0; c < NCols; ++c) { result[r][c] = (*this)[r][c] + inRHS; }
  }
  return result;
}

template <typename T, std::size_t NRows, std::size_t NCols>
constexpr Mat<T, NRows, NCols> Mat<T, NRows, NCols>::operator-(const T& inRHS) const
{
  Mat<T, NRows, NCols> result;
  for (std::size_t r = 0; r < NRows; ++r)
  {
    for (std::size_t c = 0; c < NCols; ++c) { result[r][c] = (*this)[r][c] - inRHS; }
  }
  return result;
}

template <typename T, std::size_t NRows, std::size_t NCols>
constexpr Mat<T, NRows, NCols> Mat<T, NRows, NCols>::operator*(const T& inRHS) const
{
  Mat<T, NRows, NCols> result;
  for (std::size_t r = 0; r < NRows; ++r)
  {
    for (std::size_t c = 0; c < NCols; ++c) { result[r][c] = (*this)[r][c] * inRHS; }
  }
  return result;
}

template <typename T, std::size_t NRows, std::size_t NCols>
constexpr Mat<T, NRows, NCols> Mat<T, NRows, NCols>::operator/(const T& inRHS) const
{
  Mat<T, NRows, NCols> result;
  for (std::size_t r = 0; r < NRows; ++r)
  {
    for (std::size_t c = 0; c < NCols; ++c) { result[r][c] = (*this)[r][c] / inRHS; }
  }
  return result;
}

template <typename T, std::size_t NRows, std::size_t NCols>
constexpr Vec<T, NCols> Mat<T, NRows, NCols>::operator*(const Vec<T, NCols>& inRHS) const
{
  Vec<T, NCols> result;
  for (std::size_t r = 0; r < NRows; ++r)
  {
    const auto& row_vector = (*this)[r];
    result[r] = 0;
    for (std::size_t c = 0; c < NCols; ++c) { result[r] += (row_vector[c] * inRHS[c]); }
  }
  return result;
}

template <typename T, std::size_t NRows, std::size_t NCols>
void Mat<T, NRows, NCols>::operator+=(const T& inRHS)
{
  *this = *this + inRHS;
}

template <typename T, std::size_t NRows, std::size_t NCols>
void Mat<T, NRows, NCols>::operator-=(const T& inRHS)
{
  *this = *this - inRHS;
}

template <typename T, std::size_t NRows, std::size_t NCols>
void Mat<T, NRows, NCols>::operator*=(const T& inRHS)
{
  *this = *this * inRHS;
}

template <typename T, std::size_t NRows, std::size_t NCols>
void Mat<T, NRows, NCols>::operator/=(const T& inRHS)
{
  *this = *this / inRHS;
}

template <typename T, std::size_t NRows, std::size_t NCols>
constexpr Mat<T, NRows, NCols> Mat<T, NRows, NCols>::operator-() const
{
  Mat<T, NRows, NCols> result;
  for (std::size_t r = 0; r < NRows; ++r)
  {
    for (std::size_t c = 0; c < NCols; ++c) { result[r][c] = -((*this)[r][c]); }
  }
  return result;
}

template <typename T, std::size_t NRows, std::size_t NCols>
template <std::size_t TRHSCols>
constexpr Mat<T, NRows, TRHSCols> Mat<T, NRows, NCols>::operator*(const Mat<T, NCols, TRHSCols>& inRHS) const
{
  auto result = All<Mat<T, NRows, TRHSCols>>(static_cast<T>(0));
  for (std::size_t r = 0; r < NRows; ++r)
  {
    for (std::size_t c = 0; c < TRHSCols; ++c)
    {
      for (std::size_t k = 0; k < NCols; ++k) { result[r][c] += ((*this)[r][k] * inRHS[k][c]); }
    }
  }
  return result;
}

template <typename T, std::size_t NRows, std::size_t NCols>
inline std::ostream& operator<<(std::ostream& inLHS, const Mat<T, NRows, NCols>& inRHS)
{
  inLHS << "(";
  for (std::size_t r = 0; r < NRows; ++r)
  {
    if (r > 0)
      inLHS << ", ";

    inLHS << inRHS[r];

    if (r < NRows - 1)
      inLHS << std::endl;
  }
  inLHS << ")";
  return inLHS;
}
}