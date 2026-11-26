#pragma once

#include "rpp_localization/estimation/model.hpp"
#include "rpp_localization/estimation/execution_metrics.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <exception>
#include <limits>
#include <memory>

namespace rpp_localization::estimation
{

// Internal outcomes; adapters must map them explicitly to public schema codes.
enum class Status
{
  ok,
  gated,
  not_initialized,
  invalid_input,
  stale_input,
  model_failure,
  numerical_failure,
};

class GaussianFilter
{
public:
  struct Checkpoint
  {
    explicit Checkpoint(Eigen::Index size) : state(size), covariance(size, size) {}
    virtual ~Checkpoint() = default;
    virtual bool copy_to(Checkpoint& output) const
    {
      if (output.state.size() != state.size() || output.covariance.rows() != covariance.rows() ||
        output.covariance.cols() != covariance.cols()) {return false;}
      output.state = state; output.covariance = covariance;
      output.time = time; output.initialized = initialized;
      return true;
    }
    Vector state;
    Matrix covariance;
    TimeNs time{0};
    bool initialized{false};
  };

  virtual ~GaussianFilter() = default;
  GaussianFilter(const GaussianFilter&) = delete;
  GaussianFilter& operator=(const GaussianFilter&) = delete;

  virtual Status predict(VectorIn input, MatrixIn process_covariance, TimeNs time,
    TimeNs dt, TimeNs input_dt = 0) = 0;
  virtual Status correct(std::size_t observation, VectorIn measurement, MatrixIn covariance,
    VectorIn input, TimeNs time, TimeNs input_dt = 0) = 0;

  // Allocate checkpoints only during preparation. Algorithms with persistent
  // history override these hooks; callers never inspect algorithm-specific data.
  virtual std::unique_ptr<Checkpoint> make_checkpoint() const
  {return std::make_unique<Checkpoint>(size_);}
  bool save(Checkpoint& output) const
  {
    if (output.state.size() != size_ || output.covariance.rows() != size_ ||
      output.covariance.cols() != size_ || !save_history(output)) {return false;}
    output.state = state_; output.covariance = covariance_;
    output.time = time_; output.initialized = initialized_;
    return true;
  }
  bool restore(const Checkpoint& input)
  {
    if (input.state.size() != size_ || input.covariance.rows() != size_ ||
      input.covariance.cols() != size_ || !restore_history(input)) {return false;}
    state_ = input.state; covariance_ = input.covariance;
    time_ = input.time; initialized_ = input.initialized;
    return true;
  }

  Status initialize(VectorIn state, MatrixIn covariance, TimeNs time)
  {
    if (state.size() != size_ || !state.allFinite() || !covariance_values(covariance, size_)) {
      return Status::invalid_input;
    }
    state_ = state;
    covariance_ = covariance;
    time_ = time;
    initialized_ = true;
    clear_algorithm_history();
    return Status::ok;
  }

  Status reset(VectorIn state, MatrixIn covariance, TimeNs time)
  {
    return initialize(state, covariance, time);
  }

  void clear()
  {
    initialized_ = false;
    time_ = 0;
    state_.setZero();
    covariance_.setZero();
    clear_algorithm_history();
  }

  // Borrowed views remain valid until the next mutating call. Public adapters
  // must provide coherent copies/output-pool snapshots, not retain these views.
  const Vector& state() const { return state_; }
  const Matrix& covariance() const { return covariance_; }
  TimeNs time() const { return time_; }
  bool initialized() const { return initialized_; }

protected:
  explicit GaussianFilter(const ProcessModel& process)
  : process_(process), space_(process.state_space()), size_(space_.size()),
    input_size_(process.input_size()), state_(size_), covariance_(size_, size_),
    next_state_(size_), next_covariance_(size_, size_)
  {
    if (input_size_ < 0) {
      throw std::invalid_argument("Model input dimension cannot be negative");
    }
    state_.setZero();
    covariance_.setZero();
  }

  virtual void clear_algorithm_history() {}
  virtual bool save_history(Checkpoint&) const {return true;}
  virtual bool restore_history(const Checkpoint&) {return true;}

  static bool covariance_values(MatrixIn covariance, Eigen::Index size)
  {
    if (covariance.rows() != size || covariance.cols() != size || !covariance.allFinite()) {
      return false;
    }
    // Lightweight boundary checks. PSD is a caller contract; required numerical
    // factorizations separately reject unusable innovation/state covariances.
    for (Eigen::Index row = 0; row < size; ++row) {
      if (covariance(row, row) < 0.0) {
        return false;
      }
      for (Eigen::Index column = 0; column < row; ++column) {
        const double a = covariance(row, column);
        const double b = covariance(column, row);
        if (std::abs(a - b) > 1e-10 * std::max({1.0, std::abs(a), std::abs(b)})) {
          return false;
        }
      }
    }
    return true;
  }

  static void symmetrize(MatrixOut covariance)
  {
    for (Eigen::Index row = 0; row < covariance.rows(); ++row) {
      for (Eigen::Index column = 0; column < row; ++column) {
        const double value = (covariance(row, column) + covariance(column, row)) * 0.5;
        covariance(row, column) = value;
        covariance(column, row) = value;
      }
    }
  }

  Status prediction_input(VectorIn input, MatrixIn q, TimeNs time, TimeNs dt) const
  {
    if (!initialized_) {
      return Status::not_initialized;
    }
    if (time < time_) {
      return Status::stale_input;
    }
    if (dt != time - time_ || input.size() != input_size_ || !input.allFinite() ||
      !covariance_values(q, size_))
    {
      return Status::invalid_input;
    }
    return Status::ok;
  }

  Status correction_input(Eigen::Index size, Eigen::Index input_size, VectorIn measurement,
    MatrixIn r, VectorIn input, TimeNs time) const
  {
    if (!initialized_) {
      return Status::not_initialized;
    }
    if (time < time_) {
      return Status::stale_input;
    }
    if (time != time_ || measurement.size() != size || !measurement.allFinite() ||
      input.size() != input_size || !input.allFinite() ||
      !covariance_values(r, size))
    {
      return Status::invalid_input;
    }
    return Status::ok;
  }

  void validate_binding(const ObservationBinding& binding) const
  {
    if (!binding.model || binding.model->state_size() != size_ || binding.model->input_size() < 0 ||
      std::isnan(binding.mahalanobis_threshold) || binding.mahalanobis_threshold < 0.0)
    {
      throw std::invalid_argument("Invalid observation model binding");
    }
    if (const auto* indices = binding.model->state_projection()) {
      const auto& observed = binding.model->observation_space();
      if (indices->size() != static_cast<std::size_t>(observed.size())) {
        throw std::invalid_argument("State projection width does not match observation space");
      }
      std::vector<bool> used(static_cast<std::size_t>(size_), false);
      for (std::size_t row = 0; row < indices->size(); ++row) {
        const auto index = (*indices)[row];
        if (index < 0 || index >= size_ || used[static_cast<std::size_t>(index)]) {
          throw std::invalid_argument("State projection indices must be unique state coordinates");
        }
        used[static_cast<std::size_t>(index)] = true;
        const bool state_angle = std::find(space_.angles().begin(), space_.angles().end(), index) != space_.angles().end();
        const bool observed_angle = std::find(observed.angles().begin(), observed.angles().end(),
          static_cast<Eigen::Index>(row)) != observed.angles().end();
        if (state_angle != observed_angle) {throw std::invalid_argument("State projection coordinate kinds disagree");}
      }
    }
  }

  template<typename Function>
  static bool evaluate(Function&& function)
  {
    try {
      return function();
    } catch (const std::exception&) {
      return false;
    }
  }

  static bool gate(double squared_distance, double threshold)
  {
    return std::isfinite(squared_distance) && squared_distance >= 0.0 &&
           std::sqrt(squared_distance) < threshold;
  }

  bool valid_candidate() const
  {
    return next_state_.allFinite() && covariance_values(next_covariance_, size_);
  }

  void commit(TimeNs time)
  {
    state_.swap(next_state_);
    covariance_.swap(next_covariance_);
    time_ = time;
  }

  const ProcessModel& process_;
  VectorSpace space_;
  Eigen::Index size_;
  Eigen::Index input_size_;
  Vector state_;
  Matrix covariance_;
  Vector next_state_;
  Matrix next_covariance_;
  TimeNs time_{0};
  bool initialized_{false};
  ExecutionMetrics* metrics_{nullptr};
};

}  // namespace rpp_localization::estimation
