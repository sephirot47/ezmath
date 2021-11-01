#pragma once

#include <ez/Macros.h>
#include <ez/Vec.h>
#include <type_traits>

namespace ez
{
template <typename T, std::size_t N>
using GBinaryIndex = Vec<T, N>;

template <std::size_t N>
using BinaryIndex = GBinaryIndex<std::size_t, N>;

template <std::size_t N, typename T = std::size_t>
constexpr GBinaryIndex<T, N> _MakeBinaryIndex(const std::size_t inSequentialIndex)
{
  GBinaryIndex<T, N> binary_index;
  std::size_t two_to_the_i = 1;
  for (std::size_t i = 0; i < N; ++i)
  {
    const auto twice_two_to_the_i = two_to_the_i * 2;
    binary_index[N - i - 1] = static_cast<T>(static_cast<std::size_t>(static_cast<std::size_t>(inSequentialIndex) % twice_two_to_the_i) / two_to_the_i);
    two_to_the_i = twice_two_to_the_i;
  }
  return binary_index;
}

template <std::size_t N, typename T = std::size_t>
constexpr auto _ComputeAllBinaryIndices()
{
  std::array<GBinaryIndex<T, N>, static_cast<std::size_t>(ConstexprPow(static_cast<std::size_t>(2), N))> all_binary_indices;
  for (std::size_t i = 0; i < all_binary_indices.size(); ++i) { all_binary_indices[i] = _MakeBinaryIndex<N, T>(i); }
  return all_binary_indices;
}

template <std::size_t N, typename T = std::size_t>
constexpr auto AllBinaryIndices()
{
  constexpr auto all_binary_indices = _ComputeAllBinaryIndices<N, T>();
  return all_binary_indices;
}

template <std::size_t N, typename T = std::size_t>
constexpr GBinaryIndex<T, N> MakeBinaryIndex(const std::size_t inSequentialIndex)
{
  constexpr auto BinaryIndicesTable = AllBinaryIndices<N, T>();
  return BinaryIndicesTable.at(inSequentialIndex);
}

template <std::size_t N, typename T = std::size_t>
constexpr T MakeSequentialIndex(const GBinaryIndex<T, N>& inBinaryIndex)
{
  T sequential_index = 0;
  std::size_t two_to_the_i = 1;
  for (std::size_t i = 0; i < N; ++i)
  {
    const auto& binary_index_comp = inBinaryIndex[N - i - 1];
    EXPECTS(binary_index_comp >= 0 && binary_index_comp <= 1);
    sequential_index += two_to_the_i * binary_index_comp;
    two_to_the_i *= 2;
  }
  return sequential_index;
}

}