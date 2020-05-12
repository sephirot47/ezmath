#pragma once

#include "ez/Macros.h"
#include "ez/Vec.h"

namespace ez
{
template <std::size_t N>
using BinaryIndex = Vec<std::size_t, N>;

template <std::size_t N>
constexpr BinaryIndex<N> MakeBinaryIndex(const std::size_t inSequentialIndex)
{
  BinaryIndex<N> binary_index;
  std::size_t two_to_the_i = 1;
  for (std::size_t i = 0; i < N; ++i)
  {
    auto& binary_index_comp = binary_index[N - i - 1];
    binary_index_comp = (inSequentialIndex % (two_to_the_i * 2)) / two_to_the_i;
    EXPECTS(binary_index_comp >= 0 && binary_index_comp <= 1);
    two_to_the_i *= 2;
  }
  return binary_index;
}

template <std::size_t N>
constexpr std::size_t MakeSequentialIndex(const BinaryIndex<N>& inBinaryIndex)
{
  std::size_t sequential_index = 0;
  auto two_to_the_i = 1;
  for (std::size_t i = 0; i < N; ++i)
  {
    const auto& binary_index_comp = inBinaryIndex[N - i - 1];
    EXPECTS(binary_index_comp >= 0 && binary_index_comp <= 1);

    sequential_index += two_to_the_i * binary_index_comp;
    two_to_the_i *= 2;
  }
  return sequential_index;
}

template <std::size_t N>
constexpr auto _ComputeAllBinaryIndices()
{
  std::array<BinaryIndex<N>, static_cast<std::size_t>(std::pow(static_cast<std::size_t>(2), N))> all_binary_indices;
  for (std::size_t i = 0; i < all_binary_indices.size(); ++i) { all_binary_indices[i] = MakeBinaryIndex<N>(i); }
  return all_binary_indices;
}

template <std::size_t N>
const auto AllBinaryIndices()
{
  static const auto all_binary_indices = _ComputeAllBinaryIndices<N>();
  return all_binary_indices;
}

}