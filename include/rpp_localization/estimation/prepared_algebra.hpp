#pragma once

#include <rpp_localization/estimation/model.hpp>
#include <Eigen/Cholesky>
#include <algorithm>

namespace rpp_localization::estimation
{
// Eigen's dynamic GEMM/triangular kernels may allocate packing panels for large
// dimensions. Keep the ordinary small-matrix path; larger products use bounded
// fixed-size tiles. Inputs and destination must not alias.
template<class Left, class Right>
void product(MatrixOut output, const Eigen::MatrixBase<Left>& left,
  const Eigen::MatrixBase<Right>& right, bool accumulate = false, double scale = 1.0)
{
  constexpr Eigen::Index small = 32, tile = 16;
  if (std::max({left.rows(), left.cols(), right.cols()}) <= small) {
    if (accumulate) {output.noalias() += scale * (left * right);}
    else {output.noalias() = left * right; if (scale != 1.0) {output *= scale;}}
    return;
  }
  Eigen::Matrix<double, tile, tile> a, b, sum;
  for (Eigen::Index column = 0; column < right.cols(); column += tile) {
    const auto width = std::min(tile, right.cols() - column);
    for (Eigen::Index row = 0; row < left.rows(); row += tile) {
      const auto height = std::min(tile, left.rows() - row);
      sum.setZero();
      for (Eigen::Index inner = 0; inner < left.cols(); inner += tile) {
        const auto depth = std::min(tile, left.cols() - inner);
        a.setZero(); b.setZero();
        a.topLeftCorner(height, depth) = left.block(row, inner, height, depth);
        b.topLeftCorner(depth, width) = right.block(inner, column, depth, width);
        sum.noalias() += a * b;
      }
      if (accumulate) {output.block(row, column, height, width) += scale * sum.topLeftCorner(height, width);}
      else {output.block(row, column, height, width) = scale * sum.topLeftCorner(height, width);}
    }
  }
}

// Solve S X = C^T into prepared X. Large problems use vector solves to avoid
// matrix-RHS triangular packing. Scratch is already allocated by the caller.
inline void solve_transposed(const Eigen::LDLT<Matrix>& factor, MatrixIn cross,
  MatrixOut output, VectorOut scratch)
{
  if (std::max(cross.rows(), cross.cols()) <= 32) {
    output = factor.solve(cross.transpose());
  } else {
    for (Eigen::Index column = 0; column < cross.rows(); ++column) {
      scratch = cross.row(column).transpose();
      output.col(column) = factor.solve(scratch);
    }
  }
}
}  // namespace rpp_localization::estimation
