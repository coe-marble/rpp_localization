#pragma once

#include "rpp_localization/estimation/filter.hpp"

#include <rpp_cpp/list_definitions.hpp>
#include <rpp_interfaces/field_routing.hpp>

#include <Eigen/Cholesky>
#include <array>
#include <memory>

namespace rpp_localization::estimation::shared
{
namespace codes = rpp_interfaces::contract;

// Both views pin one result slot. Retained outputs survive further calls and
// cleanup. Exhaustion is reported instead of allocating or overwriting a view.
class OutputPool
{
  struct Buffer
  {
    rpp::List<double> first, second;
    Buffer(std::size_t a, std::size_t b) {first.resize(a); second.resize(b);}
  };
public:
  static constexpr std::size_t capacity = 3;
  static constexpr std::size_t unavailable = capacity;
  OutputPool() : empty_(std::make_shared<Buffer>(0, 0)) {}

  void prepare(std::size_t first, std::size_t second = 0)
  {
    if (first > 1024 * 1024 || second > 1024 * 1024) {
      throw std::invalid_argument("Numerical result exceeds the output pool capacity");
    }
    std::array<std::shared_ptr<Buffer>, capacity> prepared;
    for (auto& value : prepared) {value = std::make_shared<Buffer>(first, second);}
    buffers_.swap(prepared);
  }
  std::size_t acquire() const
  {
    for (std::size_t index = 0; index < capacity; ++index) {
      if (buffers_[index] && buffers_[index].use_count() == 1) {return index;}
    }
    return unavailable;
  }
  rpp::List<double>& first(std::size_t index) {return buffers_[index]->first;}
  rpp::List<double>& second(std::size_t index) {return buffers_[index]->second;}
  rpp::ListConst<double> first_view(std::size_t index) const
  {return {buffers_[index]->first, buffers_[index]};}
  rpp::ListConst<double> second_view(std::size_t index) const
  {return {buffers_[index]->second, buffers_[index]};}
  rpp::ListConst<double> empty() const {return {empty_->first, empty_};}
  void clear() {for (auto& buffer : buffers_) {buffer.reset();}}
private:
  std::array<std::shared_ptr<Buffer>, capacity> buffers_;
  std::shared_ptr<Buffer> empty_;
};

inline VectorSpace vector_space(const rpp_interfaces::FieldLayout& layout)
{
  std::vector<Eigen::Index> angles;
  for (std::size_t field = 0; field < layout.fields().size(); ++field) {
    const auto kind = layout.fields()[field].kind;
    if (kind == codes::COORDINATE_QUATERNION) {
      throw std::invalid_argument("This estimator supports Euclidean and scalar-angle coordinates");
    }
    if (kind == codes::COORDINATE_CONTINUOUS_ANGLE) {
      const auto range = layout.ranges()[field];
      for (std::size_t axis = 0; axis < range.size; ++axis) {
        angles.push_back(static_cast<Eigen::Index>(range.offset + axis));
      }
    }
  }
  return VectorSpace(static_cast<Eigen::Index>(layout.size()), std::move(angles));
}

inline bool read(rpp::ListConst<double> values, VectorOut output)
{
  if (values.size() != static_cast<std::size_t>(output.size())) {return false;}
  for (Eigen::Index index = 0; index < output.size(); ++index) {
    output(index) = values[static_cast<std::size_t>(index)];
  }
  return output.allFinite();
}

inline void write_vector(VectorIn values, rpp::List<double>& output)
{
  for (Eigen::Index index = 0; index < values.size(); ++index) {
    output[static_cast<std::size_t>(index)] = values(index);
  }
}

inline void write_matrix(MatrixIn values, rpp::List<double>& output)
{
  for (Eigen::Index row = 0; row < values.rows(); ++row) {
    for (Eigen::Index column = 0; column < values.cols(); ++column) {
      output[static_cast<std::size_t>(row * values.cols() + column)] = values(row, column);
    }
  }
}

inline bool read_matrix(rpp::ListConst<double> values, MatrixOut output)
{
  if (values.size() != static_cast<std::size_t>(output.size())) {return false;}
  for (Eigen::Index row = 0; row < output.rows(); ++row) {
    for (Eigen::Index column = 0; column < output.cols(); ++column) {
      output(row, column) = values[static_cast<std::size_t>(row * output.cols() + column)];
    }
  }
  return output.allFinite();
}

inline bool covariance_structure(MatrixIn value, bool& diagonal)
{
  if (value.rows() == 0 || value.rows() != value.cols() || !value.allFinite()) {return false;}
  diagonal = true;
  for (Eigen::Index row = 0; row < value.rows(); ++row) {
    if (value(row, row) < 0.0) {return false;}
    for (Eigen::Index column = 0; column < row; ++column) {
      const double a = value(row, column), b = value(column, row);
      if (std::abs(a - b) > 1e-10 * std::max({1.0, std::abs(a), std::abs(b)})) {return false;}
      diagonal = diagonal && a == 0.0 && b == 0.0;
    }
  }
  return true;
}

inline bool valid_covariance(MatrixIn value, Eigen::LDLT<Matrix>& factor)
{
  bool diagonal = false;
  if (!covariance_structure(value, diagonal)) {return false;}
  if (diagonal) {return true;}
  factor.compute(value);
  return factor.info() == Eigen::Success && factor.vectorD().allFinite() && factor.vectorD().minCoeff() >= 0.0;
}

// Prepared boundary validation. Exact repeats reuse a successful PSD check;
// finite diagonal covariances need no factorization. Changed correlated input
// still undergoes the original symmetry and LDLT checks, without tolerating a
// near match to the cache or trusting the caller's allocation/address.
class CovarianceValidatorBase
{
public:
  virtual ~CovarianceValidatorBase() = default;
  virtual bool valid(MatrixIn value) = 0;
};

template<bool CollectMetrics>
class CovarianceValidatorImplementation final : public CovarianceValidatorBase
{
public:
  explicit CovarianceValidatorImplementation(Eigen::Index size, ExecutionMetrics* metrics = nullptr)
  : metrics_(metrics)
  {
    if constexpr (CollectMetrics) {
      if (!metrics_) {throw std::invalid_argument("Instrumented covariance validation requires a metrics sink");}
    }
    prepare(size);
  }
  void prepare(Eigen::Index size)
  {
    previous_.setIdentity(size, size);
    factor_.compute(previous_);
    cached_ = false;
  }

  bool valid(MatrixIn value) override
  {
    if (value.rows() != previous_.rows() || value.cols() != previous_.cols() || value.rows() == 0) {
      return false;
    }
    if (cached_ && (value.array() == previous_.array()).all()) {
      if constexpr (CollectMetrics) {++metrics_->covariance_cache_hits;}
      return true;
    }
    bool diagonal = false;
    if (!covariance_structure(value, diagonal)) {return false;}
    if (diagonal) {
      if constexpr (CollectMetrics) {++metrics_->covariance_diagonal_checks;}
      return true;
    }
    if constexpr (CollectMetrics) {++metrics_->covariance_factorizations;}
    factor_.compute(value);
    if (factor_.info() != Eigen::Success || !factor_.vectorD().allFinite() ||
      factor_.vectorD().minCoeff() < 0.0) {return false;}
    previous_ = value;
    cached_ = true;
    return true;
  }
private:
  Matrix previous_;
  Eigen::LDLT<Matrix> factor_;
  bool cached_{false};
  ExecutionMetrics* metrics_{nullptr};
};
using CovarianceValidator = CovarianceValidatorImplementation<false>;
using ProfiledCovarianceValidator = CovarianceValidatorImplementation<true>;

inline std::uint8_t status_code(Status status, std::uint8_t model_code)
{
  switch (status) {
    case Status::ok: return model_code == codes::CODE_LIMITED ? codes::CODE_LIMITED : codes::CODE_OK;
    case Status::gated: return codes::CODE_REJECTED;
    case Status::not_initialized: return codes::CODE_NOT_READY;
    case Status::invalid_input: return codes::CODE_INVALID_INPUT;
    case Status::stale_input: return codes::CODE_STALE_INPUT;
    case Status::model_failure:
      return model_code != codes::CODE_OK && model_code != codes::CODE_LIMITED ?
             model_code : codes::CODE_INFEASIBLE;
    case Status::numerical_failure: return codes::CODE_INFEASIBLE;
  }
  return codes::CODE_INVALID;
}
}  // namespace rpp_localization::estimation::shared
