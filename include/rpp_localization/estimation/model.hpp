#pragma once

#include <Eigen/Dense>
#include <angles/angles.h>

#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <utility>
#include <vector>

namespace rpp_localization::estimation
{

using Vector = Eigen::VectorXd;
using Matrix = Eigen::MatrixXd;
using VectorIn = Eigen::Ref<const Vector>;
using VectorOut = Eigen::Ref<Vector>;
using MatrixIn = Eigen::Ref<const Matrix>;
using MatrixOut = Eigen::Ref<Matrix>;
using TimeNs = std::uint64_t;

// Initial coordinate support: additive vector coordinates and scalar angles.
// Stored and covariance dimensions coincide. Quaternion/error-state covariance
// transport is intentionally outside this contract's current implementation.
class VectorSpace
{
public:
  explicit VectorSpace(Eigen::Index size, std::vector<Eigen::Index> angles = {})
  : size_(size), angles_(std::move(angles))
  {
    if (size <= 0) {
      throw std::invalid_argument("Coordinate spaces must have positive dimension");
    }
    std::vector<bool> used(static_cast<std::size_t>(size), false);
    for (const auto index : angles_) {
      if (index < 0 || index >= size || used[static_cast<std::size_t>(index)]) {
        throw std::invalid_argument("Angle coordinates must be unique indices within the space");
      }
      used[static_cast<std::size_t>(index)] = true;
    }
  }

  Eigen::Index size() const { return size_; }
  const std::vector<Eigen::Index>& angles() const { return angles_; }

  void normalize(VectorOut value) const
  {
    for (const auto index : angles_) {
      value(index) = ::angles::normalize_angle(value(index));
    }
  }

  void difference(VectorIn origin, VectorIn target, VectorOut delta) const
  {
    delta = target - origin;
    normalize(delta);
  }

  void retract(VectorIn origin, VectorIn delta, VectorOut next) const
  {
    next = origin + delta;
    normalize(next);
  }

  void mean(MatrixIn points, VectorIn weights, VectorOut result) const
  {
    result.setZero();
    for (Eigen::Index point = 0; point < points.cols(); ++point) {
      result.noalias() += weights(point) * points.col(point);
    }
    // Preserve the existing UKF's mean around the central sigma point.
    for (const auto index : angles_) {
      double offset = 0.0;
      for (Eigen::Index point = 1; point < points.cols(); ++point) {
        offset += weights(point) * ::angles::normalize_angle(points(index, point) - points(index, 0));
      }
      result(index) = ::angles::normalize_angle(points(index, 0) + offset);
    }
  }

private:
  Eigen::Index size_;
  std::vector<Eigen::Index> angles_;
};

// Models own configuration and optional scratch, never a live estimate. Calls
// write prepared outputs, preserve their dimensions, and return false on failure.
// A configured model must outlive its filters and be called serially.
class ProcessModel
{
public:
  virtual ~ProcessModel() = default;
  virtual const VectorSpace& state_space() const = 0;
  virtual Eigen::Index input_size() const = 0;
  virtual bool predict(VectorIn state, VectorIn input, TimeNs time, TimeNs dt,
    TimeNs input_dt, VectorOut next) const = 0;
  virtual bool linearize(VectorIn state, VectorIn input, TimeNs time, TimeNs dt,
    TimeNs input_dt, MatrixOut a) const = 0;

  virtual bool predict_and_linearize(VectorIn state, VectorIn input, TimeNs time,
    TimeNs dt, TimeNs input_dt, VectorOut next, MatrixOut a) const
  {
    return predict(state, input, time, dt, input_dt, next) &&
           linearize(state, input, time, dt, input_dt, a);
  }
};

// Explicit native capability: every output is exactly one state coordinate,
// independent of time/input, with matching Euclidean or scalar-angle semantics.
// Field names and a Jacobian at one state do not establish this contract.
class NativeStateObservation
{
public:
  virtual ~NativeStateObservation() = default;
  virtual std::vector<Eigen::Index> observation_state_indices() const = 0;
};

class ObservationModel
{
public:
  virtual ~ObservationModel() = default;
  virtual Eigen::Index state_size() const = 0;
  virtual Eigen::Index input_size() const = 0;
  virtual const VectorSpace& observation_space() const = 0;
  // Immutable after preparation. nullptr retains the general observation path.
  virtual const std::vector<Eigen::Index>* state_projection() const noexcept {return nullptr;}
  virtual bool observe(VectorIn state, VectorIn input, TimeNs time,
    TimeNs input_dt, VectorOut values) const = 0;
  virtual bool jacobian(VectorIn state, VectorIn input, TimeNs time,
    TimeNs input_dt, MatrixOut h) const = 0;
};

struct ObservationBinding
{
  const ObservationModel* model;
  double mahalanobis_threshold;
};

// A configured state-subset observation, including angular residual routing.
class StateSubsetObservation final : public ObservationModel
{
public:
  StateSubsetObservation(const VectorSpace& state, std::vector<Eigen::Index> indices)
  : state_size_(state.size()), indices_(std::move(indices)),
    space_(make_space(state, indices_)) {}

  Eigen::Index state_size() const override { return state_size_; }
  Eigen::Index input_size() const override { return 0; }
  const VectorSpace& observation_space() const override { return space_; }
  const std::vector<Eigen::Index>* state_projection() const noexcept override {return &indices_;}

  bool observe(VectorIn state, VectorIn, TimeNs, TimeNs, VectorOut values) const override
  {
    for (std::size_t index = 0; index < indices_.size(); ++index) {
      values(static_cast<Eigen::Index>(index)) = state(indices_[index]);
    }
    return true;
  }

  bool jacobian(VectorIn, VectorIn, TimeNs, TimeNs, MatrixOut h) const override
  {
    h.setZero();
    for (std::size_t index = 0; index < indices_.size(); ++index) {
      h(static_cast<Eigen::Index>(index), indices_[index]) = 1.0;
    }
    return true;
  }

private:
  static VectorSpace make_space(const VectorSpace& state, const std::vector<Eigen::Index>& indices)
  {
    std::vector<bool> used(static_cast<std::size_t>(state.size()), false);
    std::vector<Eigen::Index> angles;
    for (std::size_t index = 0; index < indices.size(); ++index) {
      const auto axis = indices[index];
      if (axis < 0 || axis >= state.size() || used[static_cast<std::size_t>(axis)]) {
        throw std::invalid_argument("Observation indices must be unique state coordinates");
      }
      used[static_cast<std::size_t>(axis)] = true;
      for (const auto angle : state.angles()) {
        if (axis == angle) {
          angles.push_back(static_cast<Eigen::Index>(index));
        }
      }
    }
    return VectorSpace(static_cast<Eigen::Index>(indices.size()), std::move(angles));
  }

  Eigen::Index state_size_;
  std::vector<Eigen::Index> indices_;
  VectorSpace space_;
};

}  // namespace rpp_localization::estimation
