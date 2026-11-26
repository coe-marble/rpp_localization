/*
 * SPDX-FileCopyrightText: (c) 2014, 2015, 2016 Charles River Analytics, Inc.
 * SPDX-License-Identifier: BSD-3-Clause
 */
#ifndef RPP_LOCALIZATION__FILTERS__UNSCENTED_KALMAN_FILTER_HPP_
#define RPP_LOCALIZATION__FILTERS__UNSCENTED_KALMAN_FILTER_HPP_

#include <algorithm>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <vector>

#include "Eigen/Cholesky"
#include "Eigen/Dense"
#include "angles/angles.h"
#include "rpp_localization/core/filter_base.hpp"
#include "rpp_localization/core/filter_common.hpp"
#include "rpp_localization/core/filter_utilities.hpp"
#include "rpp_localization/core/validation.hpp"

namespace rpp_localization
{

/**
 * @brief  Unscented Kalman filter class
 *
 * Implementation of an unscenter Kalman filter (UKF). This class derives from
 * FilterBase and overrides the predict() and correct() methods in keeping with
 * the discrete time UKF algorithm. The algorithm was derived from the UKF
 * Wikipedia article at
 * http://en.wikipedia.org/wiki/Kalman_filter#Unscented_Kalman_filter
 * as well as this paper:
 * J. J. LaViola, Jr., “A comparison of unscented and extended Kalman filtering
 * for estimating quaternion motion,” in Proc. American Control Conf., Denver,
 * CO, June 4–6, 2003, pp. 2435–2440 Obtained here:
 * http://www.cs.ucf.edu/~jjl/pubs/laviola_acc2003.pdf
 */

class Ukf : public FilterBase
{
public:
  /**
   * @brief  Constructor for the Ukf class
   *
   * @param[in] model - Injected state-prediction model.
   */
  explicit Ukf(ModelBase& model);

  /**
   * @brief  Destructor for the Ukf class
   */
  ~Ukf();

  std::unique_ptr<FilterBase> clone() const override
  {
    return std::make_unique<Ukf>(*this);
  }

  void initialize_runtime(
    const CovarianceMatrix& process_noise_covariance,
    bool use_dynamic_process_noise_covariance,
    double alpha,
    double kappa,
    double beta);

  void correct(const Measurement& measurement) override;

  void predict(TimestampNs reference_time, DurationNs delta) override;

  void reset() override;

  void set_constants(double alpha, double kappa, double beta);

protected:
  /**
   * @brief  Computes the weighted covariance and sigma points
   */
  void generate_sigma_points(const Eigen::VectorXd& state, const Eigen::MatrixXd& covariance);

  /**
   * @brief  Carries out the predict step for the posteriori state of a sigma point
   * @param[in,out] sigma_point - The sigma point (state vector) to project
   * @param[in] delta - The time step over which to project
   */
  void project_sigma_point(
    TimestampNs reference_time,
    Eigen::VectorXd& sigma_point,
    DurationNs delta);

  void compute_dynamic_process_noise_covariance(
    const Eigen::VectorXd & state, Eigen::MatrixXd& covariance);

  /**
   * @brief  The UKF sigma points
   *
   * Used to sample possible next states during prediction.
   */
  std::vector<Eigen::VectorXd> sigma_points_;

  /**
   * @brief  This matrix is used to generate the sigmaPoints_
   */
  Eigen::MatrixXd weighted_covar_sqrt_;

  Eigen::MatrixXd dynamic_process_noise_covariance_;

  /**
   * @brief  The weights associated with each sigma point when generating a new
   * state
   */
  std::vector<double> state_weights_;

  /**
   * @brief  The weights associated with each sigma point when calculating a
   * predicted estimateErrorCovariance_
   */
  std::vector<double> covar_weights_;

  /**
   * @brief  Used in weight generation for the sigma points
   */
  double lambda_;

  /**
   * @brief  Used to determine if we need to re-compute the sigma points when
   * carrying out multiple corrections
   */
  bool uncorrected_;

  // Estimate the retained sigma points belong to; a state set from outside regenerates them.
  Eigen::VectorXd sigma_points_state_;
  Eigen::MatrixXd sigma_points_covariance_;

  bool use_dynamic_process_noise_covariance_;

  Eigen::MatrixXd process_noise_covariance_;

private:
  struct CorrectionWorkspace
  {
    CorrectionWorkspace(
      Eigen::Index state_size, Eigen::Index measurement_size, std::size_t sigma_count)
    : state_subset(measurement_size),
      measurement_subset(measurement_size),
      measurement_covariance_subset(measurement_size, measurement_size),
      state_to_measurement_subset(measurement_size, state_size),
      kalman_gain_subset(state_size, measurement_size),
      innovation_subset(measurement_size),
      predicted_measurement(measurement_size),
      sigma_diff(measurement_size),
      predicted_meas_covar(measurement_size, measurement_size),
      cross_covar(state_size, measurement_size),
      sigma_point_measurements(sigma_count, Eigen::VectorXd(measurement_size)),
      innovation_covariance(measurement_size, measurement_size),
      innovation_decomposition(measurement_size),
      inverse_innovation_covariance(measurement_size, measurement_size),
      weighted_innovation(measurement_size),
      gain_predicted_covariance(state_size, measurement_size)
    {
    }

    Eigen::VectorXd state_subset;
    Eigen::VectorXd measurement_subset;
    Eigen::MatrixXd measurement_covariance_subset;
    Eigen::MatrixXd state_to_measurement_subset;
    Eigen::MatrixXd kalman_gain_subset;
    Eigen::VectorXd innovation_subset;
    Eigen::VectorXd predicted_measurement;
    Eigen::VectorXd sigma_diff;
    Eigen::MatrixXd predicted_meas_covar;
    Eigen::MatrixXd cross_covar;
    std::vector<Eigen::VectorXd> sigma_point_measurements;
    Eigen::MatrixXd innovation_covariance;
    Eigen::PartialPivLU<Eigen::MatrixXd> innovation_decomposition;
    Eigen::MatrixXd inverse_innovation_covariance;
    Eigen::VectorXd weighted_innovation;
    Eigen::MatrixXd gain_predicted_covariance;
  };

  struct Workspace
  {
    Workspace(Eigen::Index state_size, std::size_t sigma_count)
    : update_indices(static_cast<std::size_t>(state_size)),
      state(state_size),
      sigma_state_diff(state_size),
      scaled_covariance(state_size, state_size),
      sigma_decomposition(state_size),
      velocity_matrix(TWIST_SIZE, TWIST_SIZE),
      velocity_covariance(TWIST_SIZE, TWIST_SIZE)
    {
      corrections.reserve(static_cast<std::size_t>(state_size) + 1);
      for (Eigen::Index size = 0; size <= state_size; ++size) {
        corrections.emplace_back(state_size, size, sigma_count);
      }
    }

    std::vector<Eigen::Index> update_indices;
    std::vector<CorrectionWorkspace> corrections;
    Eigen::VectorXd state;
    Eigen::VectorXd sigma_state_diff;
    Eigen::MatrixXd scaled_covariance;
    Eigen::LLT<Eigen::MatrixXd> sigma_decomposition;
    Eigen::MatrixXd velocity_matrix;
    Eigen::MatrixXd velocity_covariance;
  };

  // Replay clones share a model and execute serially. Their numerical history
  // remains owned by each clone, while temporary workspace can be shared.
  std::shared_ptr<Workspace> workspace_;
};

}  // namespace rpp_localization

namespace rpp_localization
{

inline Ukf::Ukf(ModelBase& model)
: FilterBase(static_cast<int>(model.get_state().size())),
  uncorrected_(false)
{
  if (model.get_state().size() != STATE_SIZE) {
    throw std::invalid_argument("UKF runtime model must have 15 state elements");
  }

  set_model_base(model);
  size_t sigma_count = (STATE_SIZE << 1) + 1;
  sigma_points_.resize(sigma_count, Eigen::VectorXd(STATE_SIZE));
  state_weights_.resize(sigma_count);
  covar_weights_.resize(sigma_count);
  weighted_covar_sqrt_.setZero(_state_dim, _state_dim);
  sigma_points_state_.setZero(_state_dim);
  sigma_points_covariance_.setZero(_state_dim, _state_dim);
  workspace_ = std::make_shared<Workspace>(_state_dim, sigma_count);
}

inline Ukf::~Ukf() {}


inline void Ukf::initialize_runtime(
  const CovarianceMatrix& process_noise_covariance,
  const bool use_dynamic_process_noise_covariance,
  const double alpha,
  const double kappa,
  const double beta)
{
  if (process_noise_covariance.rows() != STATE_SIZE ||
    process_noise_covariance.cols() != STATE_SIZE ||
    !process_noise_covariance.allFinite())
  {
    throw std::invalid_argument("UKF process noise covariance must be finite 15x15");
  }

  _model_as_base->set_computes_covariance(false);
  _model_as_base->set_computes_jacobian(false);
  process_noise_covariance_ = process_noise_covariance;
  dynamic_process_noise_covariance_ = process_noise_covariance_;
  use_dynamic_process_noise_covariance_ = use_dynamic_process_noise_covariance;
  // Retained sigma points become valid after prediction records their estimate.
  uncorrected_ = false;
  set_constants(alpha, kappa, beta);
}

inline void Ukf::reset()
{
  uncorrected_ = false;
  dynamic_process_noise_covariance_ = process_noise_covariance_;
  for (auto& point : sigma_points_) {
    point.setZero();
  }
  weighted_covar_sqrt_.setZero();
  sigma_points_state_.setZero();
  sigma_points_covariance_.setZero();
}

inline void Ukf::set_constants(double alpha, double kappa, double beta)
{
  // Prepare constants
  lambda_ = alpha * alpha * (STATE_SIZE + kappa) - STATE_SIZE;
  size_t sigma_count = (STATE_SIZE << 1) + 1;
  state_weights_[0] = lambda_ / (STATE_SIZE + lambda_);
  covar_weights_[0] = state_weights_[0] + (1 - (alpha * alpha) + beta);
  sigma_points_[0].setZero();
  for (size_t i = 1; i < sigma_count; ++i) {
    sigma_points_[i].setZero();
    state_weights_[i] = 1 / (2 * (STATE_SIZE + lambda_));
    covar_weights_[i] = state_weights_[i];
  }
}

inline void Ukf::correct(const Measurement & measurement)
{
  Eigen::VectorXd& state = _model_as_base->get_state_unsafe();
  FB_DEBUG(
    "---------------------- Ukf::correct ----------------------\n" <<
      "State is:\n" <<
      state << "\nMeasurement is:\n" <<
      measurement.measurement_ << "\nMeasurement covariance is:\n" <<
      measurement.covariance_ << "\n");

  Eigen::MatrixXd& state_covariance = _model_as_base->get_state_covariance_unsafe();
  // In our implementation, it may be that after we call predict once, we call
  // correct several times in succession (multiple measurements with different
  // time stamps). In that event, the sigma points need to be updated to reflect
  // the current state. Throughout prediction and correction, we attempt to
  // maximize efficiency in Eigen.
  const bool sigma_points_are_current =
    uncorrected_ &&
    sigma_points_state_.size() == state.size() &&
    sigma_points_state_ == state &&
    sigma_points_covariance_ == state_covariance;
  if (!sigma_points_are_current) {
    generate_sigma_points(state, state_covariance);
  }

  // We don't want to update everything, so we need to build matrices that only
  // update the measured parts of our state vector

  // First, determine how many state vector values we're updating
  auto& update_indices = workspace_->update_indices;
  Eigen::Index update_size = 0;
  const auto mask_size = std::min<std::size_t>(
    measurement.update_vector_.size(), static_cast<std::size_t>(_state_dim));
  for (std::size_t i = 0; i < mask_size; ++i) {
    if (measurement.update_vector_[i]) {
      // Handle nan and inf values in measurements
      if (std::isnan(measurement.measurement_(i))) {
        FB_DEBUG(
          "Value at index " << i <<
            " was nan. Excluding from update.\n");
      } else if (std::isinf(measurement.measurement_(i))) {
        FB_DEBUG(
          "Value at index " << i <<
            " was inf. Excluding from update.\n");
      } else {
        update_indices[static_cast<std::size_t>(update_size++)] =
          static_cast<Eigen::Index>(i);
      }
    }
  }

  using IndexRow = Eigen::Matrix<Eigen::Index, 1, Eigen::Dynamic, Eigen::RowMajor>;
  FB_DEBUG("Update indices are:\n" <<
    IndexRow::Map(update_indices.data(), update_size) << "\n");

  auto& correction = workspace_->corrections[static_cast<std::size_t>(update_size)];
  auto& state_subset = correction.state_subset;
  auto& measurement_subset = correction.measurement_subset;
  auto& measurement_covariance_subset = correction.measurement_covariance_subset;
  auto& state_to_measurement_subset = correction.state_to_measurement_subset;
  auto& kalman_gain_subset = correction.kalman_gain_subset;
  auto& innovation_subset = correction.innovation_subset;
  auto& predicted_measurement = correction.predicted_measurement;
  auto& sigma_diff = correction.sigma_diff;
  auto& predicted_meas_covar = correction.predicted_meas_covar;
  auto& cross_covar = correction.cross_covar;
  auto& sigma_point_measurements = correction.sigma_point_measurements;

  state_subset.setZero();
  measurement_subset.setZero();
  measurement_covariance_subset.setZero();
  state_to_measurement_subset.setZero();
  kalman_gain_subset.setZero();
  innovation_subset.setZero();
  predicted_measurement.setZero();
  predicted_meas_covar.setZero();
  cross_covar.setZero();

  // Now build the sub-matrices from the full-sized matrices
  for (Eigen::Index i = 0; i < update_size; ++i) {
    measurement_subset(i) = measurement.measurement_(update_indices[i]);
    state_subset(i) = state(update_indices[i]);

    for (Eigen::Index j = 0; j < update_size; ++j) {
      measurement_covariance_subset(i, j) =
        measurement.covariance_(update_indices[i], update_indices[j]);
    }

    const auto normalized_covariance =
      validation::normalize_measurement_covariance(measurement_covariance_subset(i, i));

    if (validation::has_negative_covariance(normalized_covariance.status)) {
      FB_DEBUG(
        "WARNING: Negative covariance for index " <<
          i << " of measurement (value is" <<
          measurement_covariance_subset(i, i) <<
          "). Using absolute value...\n");
    }

    if (validation::has_near_zero_covariance(normalized_covariance.status)) {
      FB_DEBUG(
        "WARNING: measurement had very small error covariance for index " <<
          update_indices[i] <<
          ". Adding some noise to maintain filter stability.\n");
    }

    measurement_covariance_subset(i, i) = normalized_covariance.value;
  }

  // The state-to-measurement function, h, will now be a measurement_size x
  // full_state_size matrix, with ones in the (i, i) locations of the values to
  // be updated
  for (Eigen::Index i = 0; i < update_size; ++i) {
    state_to_measurement_subset(i, update_indices[i]) = 1;
  }

  FB_DEBUG(
    "Current state subset is:\n" <<
      state_subset << "\nMeasurement subset is:\n" <<
      measurement_subset << "\nMeasurement covariance subset is:\n" <<
      measurement_covariance_subset <<
      "\nState-to-measurement subset is:\n" <<
      state_to_measurement_subset << "\n");

  // (1) Generate sigma points, use them to generate a predicted measurement
  for (size_t sigma_ind = 0; sigma_ind < sigma_points_.size(); ++sigma_ind) {
    sigma_point_measurements[sigma_ind].noalias() =
      state_to_measurement_subset * sigma_points_[sigma_ind];
    predicted_measurement.noalias() +=
      state_weights_[sigma_ind] * sigma_point_measurements[sigma_ind];
  }

  // Average angles as wrapped offsets from the central sigma point.
  for (Eigen::Index i = 0; i < update_size; ++i) {
    if (update_indices[i] == StateMemberRoll ||
      update_indices[i] == StateMemberPitch ||
      update_indices[i] == StateMemberYaw)
    {
      const double centre = sigma_point_measurements[0](i);
      double offset = 0.0;
      for (size_t sigma_ind = 1; sigma_ind < sigma_points_.size(); ++sigma_ind) {
        offset += state_weights_[sigma_ind] *
          angles::normalize_angle(sigma_point_measurements[sigma_ind](i) - centre);
      }
      predicted_measurement(i) = angles::normalize_angle(centre + offset);
    }
  }

  // (2) Use the sigma point measurements and predicted measurement to compute a
  // predicted measurement covariance matrix P_zz and a state/measurement
  // cross-covariance matrix P_xz.
  for (size_t sigma_ind = 0; sigma_ind < sigma_points_.size(); ++sigma_ind) {
    sigma_diff = sigma_point_measurements[sigma_ind] - predicted_measurement;
    auto& sigma_state_diff = workspace_->sigma_state_diff;
    sigma_state_diff = sigma_points_[sigma_ind] - state;

    for (Eigen::Index i = 0; i < update_size; ++i) {
      if (update_indices[i] == StateMemberRoll ||
        update_indices[i] == StateMemberPitch ||
        update_indices[i] == StateMemberYaw)
      {
        sigma_diff(i) = angles::normalize_angle(sigma_diff(i));
      }
    }
    // Wrap every state angle, indexed by state rather than by measurement.
    for (const auto angle : {StateMemberRoll, StateMemberPitch, StateMemberYaw}) {
      sigma_state_diff(angle) = angles::normalize_angle(sigma_state_diff(angle));
    }

    predicted_meas_covar.noalias() +=
      covar_weights_[sigma_ind] * (sigma_diff * sigma_diff.transpose());
    cross_covar.noalias() += covar_weights_[sigma_ind] *
      (sigma_state_diff * sigma_diff.transpose());
  }

  // (3) Compute the Kalman gain, making sure to use the actual measurement
  // covariance: K = P_xz * (P_zz + R)^-1
  correction.innovation_covariance = predicted_meas_covar + measurement_covariance_subset;
  correction.innovation_decomposition.compute(correction.innovation_covariance);
  auto& inv_innov_cov = correction.inverse_innovation_covariance;
  inv_innov_cov = correction.innovation_decomposition.inverse();
  kalman_gain_subset.noalias() = cross_covar * inv_innov_cov;

  // (4) Apply the gain to the difference between the actual and predicted
  // measurements: x = x + K(z - z_hat)
  innovation_subset = (measurement_subset - predicted_measurement);

  // Wrap angles in the innovation
  for (Eigen::Index i = 0; i < update_size; ++i) {
    if (update_indices[i] == StateMemberRoll ||
      update_indices[i] == StateMemberPitch ||
      update_indices[i] == StateMemberYaw)
    {
      innovation_subset(i) = ::angles::normalize_angle(innovation_subset(i));
    }
  }

  // (5) Check Mahalanobis distance of innovation
  correction.weighted_innovation.noalias() = inv_innov_cov * innovation_subset;
  if (filter_utilities::check_squared_mahalanobis_threshold(
      innovation_subset.dot(correction.weighted_innovation),
      measurement.mahalanobis_thresh_))
  {
    state.noalias() += kalman_gain_subset * innovation_subset;

    // (6) Compute the new estimate error covariance P = P - (K * P_zz * K')
    correction.gain_predicted_covariance.noalias() =
      kalman_gain_subset * predicted_meas_covar;
    state_covariance.noalias() -=
      correction.gain_predicted_covariance * kalman_gain_subset.transpose();

    filter_utilities::wrap_state_angles(state);

    // Mark that we need to re-compute sigma points for successive corrections
    uncorrected_ = false;

    FB_DEBUG(
      "Predicated measurement covariance is:\n" <<
        predicted_meas_covar << "\nCross covariance is:\n" <<
        cross_covar << "\nKalman gain subset is:\n" <<
        kalman_gain_subset << "\nInnovation:\n" <<
        innovation_subset << "\nCorrected full state is:\n" <<
        state << "\nCorrected full estimate error covariance is:\n" <<
        state_covariance <<
        "\n\n---------------------- /Ukf::correct ----------------------\n");
  }
  else
  {
    FB_DEBUG(
      "Innovation mahalanobis distance test failed.");

  }
}

inline void Ukf::predict(
  const TimestampNs reference_time,
  const DurationNs delta)
{
  auto& state = workspace_->state;
  state = _model_as_base->get_state();
  FB_DEBUG(
    "---------------------- Ukf::predict ----------------------\n" <<
      "delta is " << nanoseconds_to_seconds(delta) << "\nstate is " << state << "\n");

  Eigen::MatrixXd& state_covariance = _model_as_base->get_state_covariance_unsafe();
  generate_sigma_points(state, state_covariance);

  // Sum the weighted sigma points to generate a new state prediction
  state.setZero();
  for (size_t sigma_ind = 0; sigma_ind < sigma_points_.size(); ++sigma_ind) {
    // Apply the state transition function to this sigma point
    project_sigma_point(reference_time, sigma_points_[sigma_ind], delta);
    state.noalias() += state_weights_[sigma_ind] * sigma_points_[sigma_ind];
  }

  // Average angles as wrapped offsets from the central sigma point.
  for (const auto angle : {StateMemberRoll, StateMemberPitch, StateMemberYaw}) {
    const double centre = sigma_points_[0](angle);
    double offset = 0.0;
    for (size_t sigma_ind = 1; sigma_ind < sigma_points_.size(); ++sigma_ind) {
      offset += state_weights_[sigma_ind] *
        angles::normalize_angle(sigma_points_[sigma_ind](angle) - centre);
    }
    state(angle) = angles::normalize_angle(centre + offset);
  }

  // Now use the sigma points and the predicted state to compute a predicted covariance
  state_covariance.setZero();
  auto& sigma_diff = workspace_->sigma_state_diff;
  for (size_t sigma_ind = 0; sigma_ind < sigma_points_.size(); ++sigma_ind) {
    sigma_diff = (sigma_points_[sigma_ind] - state);

    sigma_diff(StateMemberRoll) = angles::normalize_angle(sigma_diff(StateMemberRoll));
    sigma_diff(StateMemberPitch) = angles::normalize_angle(sigma_diff(StateMemberPitch));
    sigma_diff(StateMemberYaw) = angles::normalize_angle(sigma_diff(StateMemberYaw));

    state_covariance.noalias() += covar_weights_[sigma_ind] *
      (sigma_diff * sigma_diff.transpose());
  }

  // Not strictly in the theoretical UKF formulation, but necessary here
  // to ensure that we actually incorporate the process_noise_covariance_
  Eigen::MatrixXd * process_noise_covariance = &process_noise_covariance_;

  if (use_dynamic_process_noise_covariance_) {
    compute_dynamic_process_noise_covariance(state, dynamic_process_noise_covariance_);
    process_noise_covariance = &dynamic_process_noise_covariance_;
  }

  state_covariance.noalias() += nanoseconds_to_seconds(delta) * (*process_noise_covariance);
  // Keep the angles bounded
  filter_utilities::wrap_state_angles(state);

  // Mark that we can keep these sigma points
  uncorrected_ = true;
  sigma_points_state_ = state;
  sigma_points_covariance_ = state_covariance;
  _model_as_base->set_state(state);
  FB_DEBUG(
    "Predicted state is:\n" << state <<
      "\nPredicted estimate error covariance is:\n" << state_covariance <<
      "\n\n--------------------- /Ukf::predict ----------------------\n");
}

inline void Ukf::generate_sigma_points(const Eigen::VectorXd& state, const Eigen::MatrixXd& state_covariance)
{

  // Take the square root of a small fraction of the state_covariance using LL'
  // decomposition
  workspace_->scaled_covariance =
    (static_cast<double>(_state_dim) + lambda_) * state_covariance;
  workspace_->sigma_decomposition.compute(workspace_->scaled_covariance);
  weighted_covar_sqrt_ = workspace_->sigma_decomposition.matrixL();

  // Compute sigma points

  // First sigma point is the current state
  sigma_points_[0] = state;

  // Next STATE_SIZE sigma points are state + weighted_covar_sqrt_[ith column]
  // STATE_SIZE sigma points after that are state - weighted_covar_sqrt_[ith column]
  for (size_t sigma_ind = 0; sigma_ind < STATE_SIZE; ++sigma_ind) {
    sigma_points_[sigma_ind + 1] = state + weighted_covar_sqrt_.col(sigma_ind);
    sigma_points_[sigma_ind + 1 + STATE_SIZE] = state - weighted_covar_sqrt_.col(sigma_ind);
  }
}

inline void Ukf::project_sigma_point(
  const TimestampNs reference_time,
  Eigen::VectorXd& sigma_point,
  const DurationNs delta)
{
  _model_as_base->predict(sigma_point, reference_time, delta);
}


inline void Ukf::compute_dynamic_process_noise_covariance(
  const Eigen::VectorXd & state, Eigen::MatrixXd& covariance)
{
  // A more principled approach would be to get the current velocity from the
  // state, make a diagonal matrix from it, and then rotate it to be in the
  // world frame (i.e., the same frame as the pose data). We could then use this
  // rotated velocity matrix to scale the process noise covariance for the pose
  // variables as rotatedVelocityMatrix * poseCovariance *
  // rotatedVelocityMatrix' However, this presents trouble for robots that may
  // incur rotational error as a result of linear motion (and vice-versa).
  // Instead, we create a diagonal matrix whose diagonal values are the vector
  // norm of the state's velocity. We use that to scale the process noise
  // covariance.
  auto& velocity_matrix = workspace_->velocity_matrix;
  velocity_matrix.setIdentity();
  velocity_matrix.diagonal() *=
    state.segment(POSITION_V_OFFSET, TWIST_SIZE).norm();

  workspace_->velocity_covariance.noalias() =
    velocity_matrix * process_noise_covariance_.block<TWIST_SIZE, TWIST_SIZE>(
    POSITION_OFFSET, POSITION_OFFSET);
  covariance.block<TWIST_SIZE, TWIST_SIZE>(
    POSITION_OFFSET, POSITION_OFFSET).noalias() =
    workspace_->velocity_covariance * velocity_matrix.transpose();
}

}  // namespace rpp_localization

#endif  // RPP_LOCALIZATION__FILTERS__UNSCENTED_KALMAN_FILTER_HPP_
