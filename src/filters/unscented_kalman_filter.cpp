/*
 * SPDX-FileCopyrightText: (c) 2014, 2015, 2016 Charles River Analytics, Inc.
 * SPDX-License-Identifier: BSD-3-Clause
 */
#include "rpp_localization/filters/unscented_kalman_filter.hpp"

#include <stdexcept>
#include <vector>

#include "angles/angles.h"
#include "Eigen/Cholesky"
#include "Eigen/Dense"
#include "rpp_localization/core/filter_common.hpp"
#include "rpp_localization/core/filter_utilities.hpp"
#include "rpp_localization/core/validation.hpp"



namespace rpp_localization
{

Ukf::Ukf(ModelBase& model)
: FilterBase(static_cast<int>(model.get_state().size())),
  uncorrected_(true)
{
  if (model.get_state().size() != STATE_SIZE) {
    throw std::invalid_argument("UKF runtime model must have 15 state elements");
  }

  set_model_base(model);
  size_t sigma_count = (STATE_SIZE << 1) + 1;
  sigma_points_.resize(sigma_count, Eigen::VectorXd(STATE_SIZE));
  state_weights_.resize(sigma_count);
  covar_weights_.resize(sigma_count);
}

Ukf::~Ukf() {}


void Ukf::initialize_runtime(
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
  uncorrected_ = true;
  set_constants(alpha, kappa, beta);
}

void Ukf::set_constants(double alpha, double kappa, double beta)
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

void Ukf::correct(const Measurement & measurement)
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
  if (!uncorrected_) {
    generate_sigma_points(state, state_covariance);
  }

  // We don't want to update everything, so we need to build matrices that only
  // update the measured parts of our state vector

  // First, determine how many state vector values we're updating
  std::vector<size_t> update_indices;
  for (size_t i = 0; i < measurement.update_vector_.size(); ++i) {
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
        update_indices.push_back(i);
      }
    }
  }

  FB_DEBUG("Update indices are:\n" << update_indices << "\n");

  size_t update_size = update_indices.size();

  // Now set up the relevant matrices
  Eigen::VectorXd state_subset(update_size);       // x (in most literature)
  Eigen::VectorXd measurement_subset(update_size);  // z
  Eigen::MatrixXd measurement_covariance_subset(update_size, update_size);  // R
  Eigen::MatrixXd state_to_measurement_subset(update_size, STATE_SIZE);   // H
  Eigen::MatrixXd kalman_gain_subset(STATE_SIZE, update_size);            // K
  Eigen::VectorXd innovation_subset(update_size);  // z - Hx
  Eigen::VectorXd predicted_measurement(update_size);
  Eigen::VectorXd sigma_diff(update_size);
  Eigen::MatrixXd predicted_meas_covar(update_size, update_size);
  Eigen::MatrixXd cross_covar(STATE_SIZE, update_size);

  std::vector<Eigen::VectorXd> sigma_point_measurements(
    sigma_points_.size(), Eigen::VectorXd(update_size));

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
  for (size_t i = 0; i < update_size; ++i) {
    measurement_subset(i) = measurement.measurement_(update_indices[i]);
    state_subset(i) = state(update_indices[i]);

    for (size_t j = 0; j < update_size; ++j) {
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
  for (size_t i = 0; i < update_size; ++i) {
    state_to_measurement_subset(i, update_indices[i]) = 1;
  }

  FB_DEBUG(
    "Current state subset is:\n" <<
      state_subset << "\nMeasurement subset is:\n" <<
      measurement_subset << "\nMeasurement covariance subset is:\n" <<
      measurement_covariance_subset <<
      "\nState-to-measurement subset is:\n" <<
      state_to_measurement_subset << "\n");

  double roll_sum_x {};
  double roll_sum_y {};
  double pitch_sum_x {};
  double pitch_sum_y {};
  double yaw_sum_x {};
  double yaw_sum_y {};

  // (1) Generate sigma points, use them to generate a predicted measurement
  for (size_t sigma_ind = 0; sigma_ind < sigma_points_.size(); ++sigma_ind) {
    sigma_point_measurements[sigma_ind] =
      state_to_measurement_subset * sigma_points_[sigma_ind];
    predicted_measurement.noalias() +=
      state_weights_[sigma_ind] * sigma_point_measurements[sigma_ind];

    // Euler angle averaging requires special care
    for (size_t i = 0; i < update_size; ++i) {
      if (update_indices[i] == StateMemberRoll) {
        roll_sum_x += state_weights_[sigma_ind] * ::cos(sigma_point_measurements[sigma_ind](i));
        roll_sum_y += state_weights_[sigma_ind] * ::sin(sigma_point_measurements[sigma_ind](i));
      } else if (update_indices[i] == StateMemberPitch) {
        pitch_sum_x += state_weights_[sigma_ind] * ::cos(sigma_point_measurements[sigma_ind](i));
        pitch_sum_y += state_weights_[sigma_ind] * ::sin(sigma_point_measurements[sigma_ind](i));
      } else if (update_indices[i] == StateMemberYaw) {
        yaw_sum_x += state_weights_[sigma_ind] * ::cos(sigma_point_measurements[sigma_ind](i));
        yaw_sum_y += state_weights_[sigma_ind] * ::sin(sigma_point_measurements[sigma_ind](i));
      }
    }

    // Wrap angles in the innovation
    for (size_t i = 0; i < update_size; ++i) {
      if (update_indices[i] == StateMemberRoll) {
        predicted_measurement(i) = ::atan2(roll_sum_y, roll_sum_x);
      } else if (update_indices[i] == StateMemberPitch) {
        predicted_measurement(i) = ::atan2(pitch_sum_y, pitch_sum_x);
      } else if (update_indices[i] == StateMemberYaw) {
        predicted_measurement(i) = ::atan2(yaw_sum_y, yaw_sum_x);
      }
    }
  }

  // (2) Use the sigma point measurements and predicted measurement to compute a
  // predicted measurement covariance matrix P_zz and a state/measurement
  // cross-covariance matrix P_xz.
  for (size_t sigma_ind = 0; sigma_ind < sigma_points_.size(); ++sigma_ind) {
    sigma_diff = sigma_point_measurements[sigma_ind] - predicted_measurement;
    Eigen::VectorXd sigma_state_diff = sigma_points_[sigma_ind] - state;

    for (size_t i = 0; i < update_size; ++i) {
      if (update_indices[i] == StateMemberRoll ||
        update_indices[i] == StateMemberPitch ||
        update_indices[i] == StateMemberYaw)
      {
        sigma_diff(i) = angles::normalize_angle(sigma_diff(i));
        sigma_state_diff(i) = angles::normalize_angle(sigma_state_diff(i));
      }
    }

    predicted_meas_covar.noalias() +=
      covar_weights_[sigma_ind] * (sigma_diff * sigma_diff.transpose());
    cross_covar.noalias() += covar_weights_[sigma_ind] *
      (sigma_state_diff * sigma_diff.transpose());
  }

  // (3) Compute the Kalman gain, making sure to use the actual measurement
  // covariance: K = P_xz * (P_zz + R)^-1
  Eigen::MatrixXd inv_innov_cov =
    (predicted_meas_covar + measurement_covariance_subset).inverse();
  kalman_gain_subset = cross_covar * inv_innov_cov;

  // (4) Apply the gain to the difference between the actual and predicted
  // measurements: x = x + K(z - z_hat)
  innovation_subset = (measurement_subset - predicted_measurement);

  // Wrap angles in the innovation
  for (size_t i = 0; i < update_size; ++i) {
    if (update_indices[i] == StateMemberRoll ||
      update_indices[i] == StateMemberPitch ||
      update_indices[i] == StateMemberYaw)
    {
      innovation_subset(i) = ::angles::normalize_angle(innovation_subset(i));
    }
  }

  // (5) Check Mahalanobis distance of innovation
  if (filter_utilities::check_mahalanobis_threshold(
      innovation_subset, inv_innov_cov,
      measurement.mahalanobis_thresh_))
  {
    state.noalias() += kalman_gain_subset * innovation_subset;

    // (6) Compute the new estimate error covariance P = P - (K * P_zz * K')
    state_covariance.noalias() -=
      (kalman_gain_subset * predicted_meas_covar *
      kalman_gain_subset.transpose());

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

void Ukf::predict(
  const TimestampNs reference_time,
  const DurationNs delta)
{
  Eigen::VectorXd state = _model_as_base->get_state();
  FB_DEBUG(
    "---------------------- Ukf::predict ----------------------\n" <<
      "delta is " << nanoseconds_to_seconds(delta) << "\nstate is " << state << "\n");

  Eigen::MatrixXd& state_covariance = _model_as_base->get_state_covariance_unsafe();
  generate_sigma_points(state, state_covariance);
  double roll_sum_x {};
  double roll_sum_y {};
  double pitch_sum_x {};
  double pitch_sum_y {};
  double yaw_sum_x {};
  double yaw_sum_y {};

  // Sum the weighted sigma points to generate a new state prediction
  state.setZero();
  for (size_t sigma_ind = 0; sigma_ind < sigma_points_.size(); ++sigma_ind) {
    // Apply the state transition function to this sigma point
    project_sigma_point(reference_time, sigma_points_[sigma_ind], delta);
    state.noalias() += state_weights_[sigma_ind] * sigma_points_[sigma_ind];

    // Euler angle averaging requires special care
    roll_sum_x += state_weights_[sigma_ind] * ::cos(sigma_points_[sigma_ind](StateMemberRoll));
    roll_sum_y += state_weights_[sigma_ind] * ::sin(sigma_points_[sigma_ind](StateMemberRoll));
    pitch_sum_x += state_weights_[sigma_ind] * ::cos(sigma_points_[sigma_ind](StateMemberPitch));
    pitch_sum_y += state_weights_[sigma_ind] * ::sin(sigma_points_[sigma_ind](StateMemberPitch));
    yaw_sum_x += state_weights_[sigma_ind] * ::cos(sigma_points_[sigma_ind](StateMemberYaw));
    yaw_sum_y += state_weights_[sigma_ind] * ::sin(sigma_points_[sigma_ind](StateMemberYaw));
  }

  // Recover average Euler angles
  state(StateMemberRoll) = ::atan2(roll_sum_y, roll_sum_x);
  state(StateMemberPitch) = ::atan2(pitch_sum_y, pitch_sum_x);
  state(StateMemberYaw) = ::atan2(yaw_sum_y, yaw_sum_x);

  // Now use the sigma points and the predicted state to compute a predicted covariance
  state_covariance.setZero();
  Eigen::VectorXd sigma_diff(STATE_SIZE);
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
  _model_as_base->set_state(std::move(state));
  FB_DEBUG(
    "Predicted state is:\n" << state <<
      "\nPredicted estimate error covariance is:\n" << state_covariance <<
      "\n\n--------------------- /Ukf::predict ----------------------\n");
}

void Ukf::generate_sigma_points(const Eigen::VectorXd& state, const Eigen::MatrixXd& state_covariance)
{

  // Take the square root of a small fraction of the state_covariance using LL'
  // decomposition
  weighted_covar_sqrt_ =
    ((static_cast<double>(STATE_SIZE) + lambda_) * state_covariance).llt().matrixL();

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

void Ukf::project_sigma_point(
  const TimestampNs reference_time,
  Eigen::VectorXd& sigma_point,
  const DurationNs delta)
{
  _model_as_base->predict(sigma_point, reference_time, delta);
}


void Ukf::compute_dynamic_process_noise_covariance(
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
  Eigen::MatrixXd velocity_matrix(TWIST_SIZE, TWIST_SIZE);
  velocity_matrix.setIdentity();
  velocity_matrix.diagonal() *=
    state.segment(POSITION_V_OFFSET, TWIST_SIZE).norm();

  covariance.block<TWIST_SIZE, TWIST_SIZE>(
    POSITION_OFFSET, POSITION_OFFSET) =
    velocity_matrix *
    process_noise_covariance_.block<TWIST_SIZE, TWIST_SIZE>(
    POSITION_OFFSET,
    POSITION_OFFSET) *
    velocity_matrix.transpose();
}

}  // namespace rpp_localization
