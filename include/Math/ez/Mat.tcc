#include "ez/Mat.h"
#include "ez/MathInitializers.h"
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

template <typename T, std::size_t N, std::size_t NCols>
constexpr Vec<T, N> Column(const Mat<T, N, NCols>& inMat, const std::size_t inColumn)
{
  Vec<T, N> column;
  for (std::size_t i = 0; i < N; ++i) { column[i] = inMat[i][inColumn]; }
  return column;
}

template <typename T, std::size_t N, std::size_t NCols>
constexpr Vec<T, N> Row(const Mat<T, N, NCols>& inMat, const std::size_t inRow)
{
  Vec<T, N> row;
  for (std::size_t i = 0; i < N; ++i) { row[i] = inMat[inRow][i]; }
  return row;
}

template <std::size_t IColumn, typename T, std::size_t N, std::size_t NCols>
constexpr Vec<T, N> Column(const Mat<T, N, NCols>& inMat)
{
  Vec<T, N> column;
  for (std::size_t i = 0; i < N; ++i) { column[i] = inMat[i][IColumn]; }
  return column;
}

template <std::size_t IRow, typename T, std::size_t N, std::size_t NCols>
constexpr Vec<T, N> Row(const Mat<T, N, NCols>& inMat)
{
  Vec<T, N> row;
  for (std::size_t i = 0; i < N; ++i) { row[i] = inMat[IRow][i]; }
  return row;
}

template <typename T, std::size_t N>
constexpr SquareMat<T, N> Transposed(const SquareMat<T, N>& inMat)
{
  SquareMat<T, N> transposed;
  for (std::size_t row = 0; row < N; ++row)
  {
    for (std::size_t col = 0; col < N; ++col) { transposed[row][col] = inMat[col][row]; }
  }
  return transposed;
}

template <typename T, std::size_t N>
constexpr SquareMat<T, N - 1>
Cofactor(const SquareMat<T, N>& inMat, const std::size_t inRowToRemove, const std::size_t inColumnToRemove)
{
  std::size_t cofactor_matrix_row = 0;
  std::size_t cofactor_matrix_col = 0;
  SquareMat<T, N - 1> cofactor_matrix;
  for (std::size_t row = 0; row < N; row++)
  {
    if (row == inRowToRemove)
      continue;

    for (std::size_t col = 0; col < N; col++)
    {
      if (col == inColumnToRemove)
        continue;

      cofactor_matrix[cofactor_matrix_row][cofactor_matrix_col] = inMat[row][col];

      ++cofactor_matrix_col;
      if (cofactor_matrix_col == N - 1)
      {
        ++cofactor_matrix_row;
        cofactor_matrix_col = 0;
      }
    }
  }
  return cofactor_matrix;
}

template <typename T, std::size_t N>
constexpr SquareMat<T, N> Adjoint(const SquareMat<T, N>& inMat)
{
  if (N == 1)
    return All<SquareMat<T, N>>(static_cast<T>(1));

  SquareMat<T, N> adjoint;
  for (std::size_t row = 0; row < N; row++)
  {
    for (std::size_t col = 0; col < N; col++)
    {
      const auto cofactor = Cofactor(inMat, row, col);
      const auto sign = static_cast<T>(((row + col) % 2 == 0) ? 1 : -1);
      adjoint[col][row] = sign * Determinant(cofactor);
    }
  }
  return adjoint;
}

template <typename T, std::size_t N>
constexpr T Determinant(const SquareMat<T, N>& inMat)
{
  if constexpr (N == 1)
    return inMat[0][0];
  else
  {
    auto sign = static_cast<T>(1);
    auto determinant = static_cast<T>(0);
    for (std::size_t f = 0; f < N; f++)
    {
      const auto cofactor = Cofactor(inMat, 0, f);
      determinant += sign * inMat[0][f] * Determinant(cofactor);
      sign = -sign;
    }
    return determinant;
  }
}

template <typename T>
constexpr auto Inverted(const T& inValue)
{
  const auto determinant = Determinant(inValue);
  if (determinant == 0)
    THROW_EXCEPTION("Singular matrix (determinant is 0), can not compute its inverse");

  const auto adjoint = Adjoint(inValue);

  const auto inverse = adjoint / determinant;
  return inverse;
}

template <typename T>
constexpr auto Translation(const SquareMat<T, 3>& inMat)
{
  return Part<0, 1>(Column<2>(inMat));
}

template <typename T>
constexpr auto Translation(const SquareMat<T, 4>& inMat)
{
  return Part<0, 2>(Column<3>(inMat));
}

template <typename T>
constexpr auto Scale(const SquareMat<T, 4>& inMat)
{
  return Vec<T, 3> { inMat[0][0], inMat[1][1] };
}

template <typename T>
constexpr auto Scale(const SquareMat<T, 3>& inMat)
{
  return Vec<T, 3> { inMat[0][0], inMat[1][1], inMat[2][2] };
}

template <typename T, std::size_t N>
constexpr SquareMat<T, N> NormalMat(const SquareMat<T, N>& inModelViewMatrix)
{
  return Transposed(Inverted(inModelViewMatrix));
}

template <typename T, std::size_t N>
constexpr SquareMat<T, N + 1> TranslationMat(const Vec<T, N>& inTranslation)
{
  auto translation_matrix = Identity<SquareMat<T, N + 1>>();
  for (std::size_t i = 0; i < N; ++i) { translation_matrix[i][N] = inTranslation[i]; }
  return translation_matrix;
}

template <typename T>
constexpr std::enable_if_t<IsNumber_v<T>, SquareMat<T, 3>> RotationMat(const T inAngle)
{
  return SquareMat<T, 3> { Vec3<T> { std::cos(inAngle), -std::sin(inAngle), static_cast<T>(0) },
    Vec3<T> { std::sin(inAngle), std::cos(inAngle), static_cast<T>(0) },
    Vec3<T> { static_cast<T>(0), static_cast<T>(0), static_cast<T>(1) } };
}

template <typename T>
constexpr SquareMat<T, 4> RotationMat(const Quat<T>& inRotation)
{
  const auto& q = inRotation;
  const auto qxx { q[0] * q[0] };
  const auto qyy { q[1] * q[1] };
  const auto qzz { q[2] * q[2] };
  const auto qxz { q[0] * q[2] };
  const auto qxy { q[0] * q[1] };
  const auto qyz { q[1] * q[2] };
  const auto qwx { q[3] * q[0] };
  const auto qwy { q[3] * q[1] };
  const auto qwz { q[3] * q[2] };

  return Mat4<T> { Vec4<T> { static_cast<T>(1 - 2 * (qyy + qzz)),
                       static_cast<T>(2 * (qxy - qwz)),
                       static_cast<T>(2 * (qxz + qwy)),
                       static_cast<T>(0) },
    Vec4<T> { static_cast<T>(2 * (qxy + qwz)),
        static_cast<T>(1 - 2 * (qxx + qzz)),
        static_cast<T>(2 * (qyz - qwx)),
        static_cast<T>(0) },
    Vec4<T> { static_cast<T>(2 * (qxz - qwy)),
        static_cast<T>(2 * (qyz + qwx)),
        static_cast<T>(1 - 2 * (qxx + qyy)),
        static_cast<T>(0) },
    Vec4<T> { static_cast<T>(0), static_cast<T>(0), static_cast<T>(0), static_cast<T>(1) } };
}

template <typename T, std::size_t N>
constexpr SquareMat<T, N + 1> ScaleMat(const Vec<T, N>& inScale)
{
  auto scale_matrix = Identity<SquareMat<T, N + 1>>();
  for (std::size_t i = 0; i < N; ++i) { scale_matrix[i][i] = inScale[i]; }
  scale_matrix[N][N] = static_cast<T>(1);
  return scale_matrix;
}

template <typename T>
constexpr Mat4<T> PerspectiveMat(const T inAngleOfViewRads, const T inAspectRatio, const T inZNear, const T inZFar)
{
  const T tanHalfFovy = std::tan(inAngleOfViewRads / static_cast<T>(2));

  Mat4<T> perspective_matrix = {};
  perspective_matrix[0][0] = static_cast<T>(1) / (inAspectRatio * tanHalfFovy);
  perspective_matrix[1][1] = static_cast<T>(1) / (tanHalfFovy);
  perspective_matrix[2][2] = -(inZFar + inZNear) / (inZFar - inZNear);
  perspective_matrix[3][2] = -static_cast<T>(1);
  perspective_matrix[2][3] = -(static_cast<T>(2) * inZFar * inZNear) / (inZFar - inZNear);
  return perspective_matrix;
}

template <typename T, std::size_t N>
constexpr SquareMat<T, N + 1> OrthographicMat(const Vec<T, N>& inMin, const Vec<T, N>& inMax)
{
  SquareMat<T, N + 1> orthographic_matrix;
  for (std::size_t i = 0; i < N; ++i)
  {
    orthographic_matrix[i][i] = static_cast<T>(2) / (inMax[i] - inMin[i]);
    orthographic_matrix[i][N] = -static_cast<T>(inMax[i] + inMin[i]) / static_cast<T>(inMax[i] - inMin[i]);
  }
  orthographic_matrix[N][N] = static_cast<T>(1);
  return orthographic_matrix;
}

template <typename T>
constexpr T Diagonal(const ValueType_t<T>& inDiagonalValue)
{
  if constexpr (IsMat_v<T>)
  {
    static_assert(T::NumRows == T::NumCols, "Diagonal only supported for square matrices.");

    T diagonal_matrix = All<T>(static_cast<ValueType_t<T>>(0));
    for (std::size_t i = 0; i < T::NumRows; ++i) { diagonal_matrix[i][i] = inDiagonalValue; }
    return diagonal_matrix;
  }
  else
  {
    static_assert(!std::is_same_v<T, T>, "Not implemented for this type.");
  }
}
}