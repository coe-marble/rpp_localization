#pragma once

#include "rpp_localization/estimation/filter.hpp"
#include "rpp_localization/estimation/prepared_algebra.hpp"

#include <Eigen/Cholesky>

namespace rpp_localization::estimation
{

enum class UkfCorrection
{
  // Reproduce the current localization UKF: retain propagated sigma points
  // before additive Q, and subtract K * predicted_observation_covariance * K'.
  legacy,
  // Regenerate from the posterior including Q, and subtract K * innovation * K'.
  innovation,
};

struct UkfOptions
{
  double alpha{0.001};
  double kappa{0.0};
  double beta{2.0};
  UkfCorrection correction{UkfCorrection::legacy};
};

template<bool CollectMetrics>
class UkfImplementation final : public GaussianFilter
{
public:
  UkfImplementation(const ProcessModel& process, const std::vector<ObservationBinding>& observations,
    UkfOptions options = {}, ExecutionMetrics* metrics = nullptr)
  : GaussianFilter(process), options_(options), count_(2 * size_ + 1),
    weights_(count_), covariance_weights_(count_), sigma_(size_, count_),
    projected_sigma_(size_, count_), retained_sigma_(size_, count_),
    scaled_covariance_(size_, size_), root_(size_, size_), llt_(size_), delta_(size_)
  {
    if constexpr (CollectMetrics) {
      if (!metrics) {throw std::invalid_argument("Instrumented UKF requires a metrics sink");}
      metrics_ = metrics;
    }
    const double lambda = options.alpha * options.alpha * (size_ + options.kappa) - size_;
    scale_ = size_ + lambda;
    if (!std::isfinite(options.alpha) || options.alpha <= 0.0 || !std::isfinite(options.kappa) ||
      !std::isfinite(options.beta) || !std::isfinite(scale_) ||
      scale_ <= std::numeric_limits<double>::min())
    {
      throw std::invalid_argument("UKF parameters must produce a finite positive sigma scale");
    }
    weights_.setConstant(1.0 / (2.0 * scale_));
    covariance_weights_ = weights_;
    weights_(0) = lambda / scale_;
    covariance_weights_(0) = weights_(0) + 1.0 - options.alpha * options.alpha + options.beta;
    if (!weights_.allFinite() || !covariance_weights_.allFinite()) {
      throw std::invalid_argument("UKF parameters produce non-finite sigma weights");
    }
    corrections_.reserve(observations.size());
    for (const auto& binding : observations) {
      validate_binding(binding);
      corrections_.emplace_back(binding, size_, count_);
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
    if (!generate_sigma_points()) {
      return Status::numerical_failure;
    }
    for (Eigen::Index point = 0; point < count_; ++point) {
      if (!evaluate([&] {
          return process_.predict(sigma_.col(point), input, time_, dt, input_dt,
            projected_sigma_.col(point));
        }))
      {
        return Status::model_failure;
      }
    }
    if (!projected_sigma_.allFinite()) {
      return Status::numerical_failure;
    }
    space_.mean(projected_sigma_, weights_, next_state_);
    next_covariance_.setZero();
    for (Eigen::Index point = 0; point < count_; ++point) {
      space_.difference(next_state_, projected_sigma_.col(point), delta_);
      next_covariance_.noalias() += covariance_weights_(point) * (delta_ * delta_.transpose());
    }
    next_covariance_ += process_covariance;
    symmetrize(next_covariance_);
    if (!valid_candidate()) {
      return Status::numerical_failure;
    }
    commit(time);
    retained_sigma_ = projected_sigma_;
    retained_current_ = true;
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
    const bool use_retained = retained_current_ && options_.correction == UkfCorrection::legacy;
    if (!use_retained && !generate_sigma_points()) {
      return Status::numerical_failure;
    }
    const Matrix& points = use_retained ? retained_sigma_ : sigma_;
    if (work.projection.empty()) {
      for (Eigen::Index point = 0; point < count_; ++point) {
        if (!evaluate([&] {
            return work.binding.model->observe(points.col(point), input, time_, input_dt,
              work.sigma.col(point));
          })) {return Status::model_failure;}
      }
    } else {
      if constexpr (CollectMetrics) {++metrics_->projected_corrections;}
      for (Eigen::Index row = 0; row < work.space.size(); ++row) {
        work.sigma.row(row) = points.row(work.projection[static_cast<std::size_t>(row)]);
      }
    }
    if (!work.sigma.allFinite()) {
      return Status::numerical_failure;
    }
    work.space.mean(work.sigma, weights_, work.predicted);
    work.predicted_covariance.setZero();
    work.cross_covariance.setZero();
    for (Eigen::Index point = 0; point < count_; ++point) {
      work.space.difference(work.predicted, work.sigma.col(point), work.difference);
      space_.difference(state_, points.col(point), delta_);
      work.predicted_covariance.noalias() += covariance_weights_(point) *
        (work.difference * work.difference.transpose());
      work.cross_covariance.noalias() += covariance_weights_(point) *
        (delta_ * work.difference.transpose());
    }
    work.space.difference(work.predicted, measurement, work.innovation);
    work.s = work.predicted_covariance + measurement_covariance;
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
    solve_transposed(work.ldlt, work.cross_covariance, work.gain_transpose, work.weighted);
    work.gain = work.gain_transpose.transpose();
    delta_.noalias() = work.gain * work.innovation;
    space_.retract(state_, delta_, next_state_);
    const Matrix& correction_covariance = options_.correction == UkfCorrection::legacy ?
      work.predicted_covariance : work.s;
    product(work.gain_covariance, work.gain, correction_covariance);
    next_covariance_ = covariance_;
    product(next_covariance_, work.gain_covariance, work.gain.transpose(), true, -1.0);
    symmetrize(next_covariance_);
    if (!valid_candidate()) {
      return Status::numerical_failure;
    }
    commit(time_);
    retained_current_ = false;
    return Status::ok;
  }

private:
  struct UkfCheckpoint final : Checkpoint
  {
    explicit UkfCheckpoint(Eigen::Index n) : Checkpoint(n), retained(n, 2 * n + 1) {}
    bool copy_to(Checkpoint& output) const override
    {
      auto* target = dynamic_cast<UkfCheckpoint*>(&output);
      if (!target || target->retained.rows() != retained.rows() ||
        target->retained.cols() != retained.cols() || !Checkpoint::copy_to(output)) {return false;}
      target->current = current;
      if (current) {target->retained = retained;}
      return true;
    }
    Matrix retained;
    bool current{false};
  };
public:
  std::unique_ptr<Checkpoint> make_checkpoint() const override
  {return std::make_unique<UkfCheckpoint>(size_);}
private:
  bool save_history(Checkpoint& output) const override
  {
    auto* checkpoint = dynamic_cast<UkfCheckpoint*>(&output);
    if (!checkpoint || checkpoint->retained.rows() != size_ ||
      checkpoint->retained.cols() != count_) {return false;}
    checkpoint->current = retained_current_;
    if (retained_current_) {checkpoint->retained = retained_sigma_;}
    return true;
  }
  bool restore_history(const Checkpoint& input) override
  {
    const auto* checkpoint = dynamic_cast<const UkfCheckpoint*>(&input);
    if (!checkpoint || checkpoint->retained.rows() != size_ ||
      checkpoint->retained.cols() != count_) {return false;}
    retained_current_ = checkpoint->current;
    if (retained_current_) {retained_sigma_ = checkpoint->retained;}
    return true;
  }
  void clear_algorithm_history() override { retained_current_ = false; }

  bool generate_sigma_points()
  {
    scaled_covariance_ = scale_ * covariance_;
    // Larger Eigen blocked LLT paths can allocate temporary packing storage.
    // The prepared unblocked path below also supports semidefinite covariance.
    const bool small = size_ <= 32;
    if (small) {llt_.compute(scaled_covariance_);}
    if (small && llt_.info() == Eigen::Success) {
      root_ = llt_.matrixL();
    } else {
      // Semidefinite startup covariances can have exact zero pivots. Keep a
      // prepared, allocation-free Cholesky fallback instead of injecting noise.
      root_.setZero();
      const double tolerance = 64.0 * std::numeric_limits<double>::epsilon() *
        std::max(scaled_covariance_.cwiseAbs().maxCoeff(), std::numeric_limits<double>::min());
      for (Eigen::Index row = 0; row < size_; ++row) {
        for (Eigen::Index column = 0; column <= row; ++column) {
          double value = scaled_covariance_(row, column);
          for (Eigen::Index index = 0; index < column; ++index) {
            value -= root_(row, index) * root_(column, index);
          }
          if (row == column) {
            if (value < -tolerance) {
              return false;
            }
            root_(row, column) = std::sqrt(std::max(0.0, value));
          } else if (root_(column, column) > 0.0) {
            root_(row, column) = value / root_(column, column);
          } else if (std::abs(value) > tolerance) {
            return false;
          }
        }
      }
    }
    sigma_.col(0) = state_;
    for (Eigen::Index axis = 0; axis < size_; ++axis) {
      // Keep legacy sigma generation; wrapping happens in residuals and means.
      sigma_.col(axis + 1) = state_ + root_.col(axis);
      sigma_.col(axis + 1 + size_) = state_ - root_.col(axis);
    }
    return sigma_.allFinite();
  }

  struct Correction
  {
    Correction(const ObservationBinding& value, Eigen::Index n, Eigen::Index count)
    : binding(value), space(value.model->observation_space()), input_size(value.model->input_size()),
      sigma(space.size(), count), predicted(space.size()), difference(space.size()),
      innovation(space.size()), weighted(space.size()), predicted_covariance(space.size(), space.size()),
      cross_covariance(n, space.size()), s(space.size(), space.size()), ldlt(space.size()),
      gain(n, space.size()), gain_transpose(space.size(), n), gain_covariance(n, space.size())
    {
      if (const auto* indices = value.model->state_projection()) {projection = *indices;}
    }

    ObservationBinding binding;
    std::vector<Eigen::Index> projection;
    VectorSpace space;
    Eigen::Index input_size;
    Matrix sigma;
    Vector predicted, difference, innovation, weighted;
    Matrix predicted_covariance, cross_covariance, s;
    Eigen::LDLT<Matrix> ldlt;
    Matrix gain, gain_transpose, gain_covariance;
  };

  UkfOptions options_;
  Eigen::Index count_;
  double scale_;
  Vector weights_, covariance_weights_;
  Matrix sigma_, projected_sigma_, retained_sigma_, scaled_covariance_, root_;
  Eigen::LLT<Matrix> llt_;
  Vector delta_;
  bool retained_current_{false};
  std::vector<Correction> corrections_;
};
using Ukf = UkfImplementation<false>;
using ProfiledUkf = UkfImplementation<true>;

}  // namespace rpp_localization::estimation
