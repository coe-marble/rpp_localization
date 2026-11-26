#pragma once

#include "rpp_localization/estimation/filter.hpp"
#include "rpp_localization/estimation/prepared_algebra.hpp"

#include <Eigen/Cholesky>

namespace rpp_localization::estimation
{

// Runtime dimensions, estimator-owned posterior, and model-supplied Jacobians.
// All storage is prepared at construction. Shape changes require reconstruction.
template<bool CollectMetrics>
class EkfImplementation final : public GaussianFilter
{
public:
  EkfImplementation(const ProcessModel& process, const std::vector<ObservationBinding>& observations,
    ExecutionMetrics* metrics = nullptr)
  : GaussianFilter(process), a_(size_, size_), projected_(size_, size_),
    gain_residual_(size_, size_), delta_(size_)
  {
    if constexpr (CollectMetrics) {
      if (!metrics) {throw std::invalid_argument("Instrumented EKF requires a metrics sink");}
      metrics_ = metrics;
    }
    corrections_.reserve(observations.size());
    for (const auto& binding : observations) {
      validate_binding(binding);
      corrections_.emplace_back(binding, size_);
    }
  }

  Status predict(VectorIn input, MatrixIn process_covariance, TimeNs time,
    TimeNs dt, TimeNs input_dt = 0) override
  {
    if constexpr (CollectMetrics) {++metrics_->core_predictions;}
    const auto status = prediction_input(input, process_covariance, time, dt);
    if (status != Status::ok) {
      return status;
    }
    if (!evaluate([&] {
        return process_.predict_and_linearize(state_, input, time_, dt, input_dt, next_state_, a_);
      }))
    {
      return Status::model_failure;
    }
    if (!next_state_.allFinite() || !a_.allFinite()) {
      return Status::numerical_failure;
    }
    space_.normalize(next_state_);
    product(projected_, a_, covariance_);
    product(next_covariance_, projected_, a_.transpose());
    // Q is already discrete. The estimator never scales or adds it twice.
    next_covariance_ += process_covariance;
    symmetrize(next_covariance_);
    if (!valid_candidate()) {
      return Status::numerical_failure;
    }
    commit(time);
    return Status::ok;
  }

  Status correct(std::size_t observation, VectorIn measurement, MatrixIn measurement_covariance,
    VectorIn input, TimeNs time, TimeNs input_dt = 0) override
  {
    if constexpr (CollectMetrics) {++metrics_->core_corrections;}
    if (observation >= corrections_.size()) {
      return Status::invalid_input;
    }
    auto& work = corrections_[observation];
    const auto status = correction_input(work.space.size(), work.input_size,
      measurement, measurement_covariance, input, time);
    if (status != Status::ok) {
      return status;
    }
    if (work.projection.empty()) {
      if (!evaluate([&] {
          return work.binding.model->observe(state_, input, time_, input_dt, work.predicted) &&
                 work.binding.model->jacobian(state_, input, time_, input_dt, work.h);
        })) {return Status::model_failure;}
      if (!work.predicted.allFinite() || !work.h.allFinite()) {return Status::numerical_failure;}
      product(work.pht, covariance_, work.h.transpose());
      product(work.s, work.h, work.pht);
    } else {
      if constexpr (CollectMetrics) {++metrics_->projected_corrections;}
      for (Eigen::Index row = 0; row < work.space.size(); ++row) {
        const auto index = work.projection[static_cast<std::size_t>(row)];
        work.predicted(row) = state_(index);
        work.pht.col(row) = covariance_.col(index);
      }
      for (Eigen::Index row = 0; row < work.space.size(); ++row) {
        work.s.row(row) = work.pht.row(work.projection[static_cast<std::size_t>(row)]);
      }
    }
    work.space.difference(work.predicted, measurement, work.innovation);
    work.s += measurement_covariance;
    symmetrize(work.s);
    if (!work.s.allFinite() || !work.innovation.allFinite()) {
      return Status::numerical_failure;
    }
    work.ldlt.compute(work.s);
    if (work.ldlt.info() != Eigen::Success || !work.ldlt.vectorD().allFinite() ||
      work.ldlt.vectorD().minCoeff() <= 0.0)
    {
      return Status::numerical_failure;
    }
    work.weighted = work.ldlt.solve(work.innovation);
    const double squared_distance = work.innovation.dot(work.weighted);
    if (!work.weighted.allFinite() || !std::isfinite(squared_distance) || squared_distance < 0.0) {
      return Status::numerical_failure;
    }
    if (!gate(squared_distance, work.binding.mahalanobis_threshold)) {
      return Status::gated;
    }
    solve_transposed(work.ldlt, work.pht, work.gain_transpose, work.weighted);
    work.gain = work.gain_transpose.transpose();
    delta_.noalias() = work.gain * work.innovation;
    space_.retract(state_, delta_, next_state_);
    if (work.projection.empty() || work.space.size() * 2 >= size_) {
      gain_residual_.setIdentity();
      product(gain_residual_, work.gain, work.h, true, -1.0);
      product(projected_, gain_residual_, covariance_);
      product(next_covariance_, projected_, gain_residual_.transpose());
    } else {
      // Multiply the sparse rows of M=I-KH directly: O(n*n*m), preserving
      // factored Joseph M*P*M' + K*R*K'. Form each 1-K diagonal first, rather
      // than cancelling it later by subtracting two covariance products.
      for (Eigen::Index row = 0; row < size_; ++row) {
        const auto self = work.inverse_projection[static_cast<std::size_t>(row)];
        projected_.row(row) = (self < 0 ? 1.0 : 1.0 - work.gain(row, self)) * covariance_.row(row);
        for (Eigen::Index index = 0; index < work.space.size(); ++index) {
          if (index != self) {
            projected_.row(row).noalias() -= work.gain(row, index) *
              covariance_.row(work.projection[static_cast<std::size_t>(index)]);
          }
        }
      }
      for (Eigen::Index column = 0; column < size_; ++column) {
        const auto self = work.inverse_projection[static_cast<std::size_t>(column)];
        next_covariance_.col(column) = (self < 0 ? 1.0 : 1.0 - work.gain(column, self)) * projected_.col(column);
        for (Eigen::Index index = 0; index < work.space.size(); ++index) {
          if (index != self) {
            next_covariance_.col(column).noalias() -= work.gain(column, index) *
              projected_.col(work.projection[static_cast<std::size_t>(index)]);
          }
        }
      }
    }
    product(work.kr, work.gain, measurement_covariance);
    product(next_covariance_, work.kr, work.gain.transpose(), true);
    symmetrize(next_covariance_);
    if (!valid_candidate()) {
      return Status::numerical_failure;
    }
    commit(time_);
    return Status::ok;
  }

private:
  struct Correction
  {
    Correction(const ObservationBinding& value, Eigen::Index state_size)
    : binding(value), space(value.model->observation_space()),
      input_size(value.model->input_size()), predicted(space.size()), innovation(space.size()),
      weighted(space.size()), h(space.size(), state_size), pht(state_size, space.size()),
      s(space.size(), space.size()), ldlt(space.size()), gain(state_size, space.size()),
      gain_transpose(space.size(), state_size), kr(state_size, space.size())
    {
      if (const auto* indices = value.model->state_projection()) {
        projection = *indices;
        // Wide projections retain Eigen's dense Joseph products; H is constant
        // and prepared once, rather than requested through the model API.
        h.setZero();
        inverse_projection.assign(static_cast<std::size_t>(state_size), -1);
        for (std::size_t row = 0; row < projection.size(); ++row) {
          h(static_cast<Eigen::Index>(row), projection[row]) = 1.0;
          inverse_projection[static_cast<std::size_t>(projection[row])] = static_cast<Eigen::Index>(row);
        }
      }
    }

    ObservationBinding binding;
    VectorSpace space;
    Eigen::Index input_size;
    Vector predicted, innovation, weighted;
    Matrix h, pht, s;
    Eigen::LDLT<Matrix> ldlt;
    Matrix gain, gain_transpose, kr;
    std::vector<Eigen::Index> projection, inverse_projection;
  };

  Matrix a_, projected_, gain_residual_;
  Vector delta_;
  std::vector<Correction> corrections_;
};
using Ekf = EkfImplementation<false>;
using ProfiledEkf = EkfImplementation<true>;

}  // namespace rpp_localization::estimation
