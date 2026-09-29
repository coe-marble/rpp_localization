/*
 * SPDX-FileCopyrightText: (c) 2014, 2015, 2016 Charles River Analytics, Inc.
 * SPDX-FileCopyrightText: (c) 2017, Locus Robotics, Inc.
 * SPDX-License-Identifier: BSD-3-Clause
 */
#include "rpp_localization/filters/ekf.hpp"

#include <cmath>
#include <vector>

#include "angles/angles.h"
#include "Eigen/Dense"
#include "rclcpp/time.hpp"
#include "rpp_localization/core/filter_base.hpp"
#include "rpp_localization/core/measurement.hpp"
#include "rpp_localization/core/filter_common.hpp"
#include "rpp_localization/core/filter_utilities.hpp"
#include "rpp_localization/models/constant_acc_model.hpp"




namespace rpp_localization
{

template<class T>
Ekf<T>::Ekf(int state_space_dim)
: FilterBase(state_space_dim),
  _model(state_space_dim),
  _debug(false)
{
  set_model_base(_model);
}

template<class T>
Ekf<T>::~Ekf() {}

template<class T>
void Ekf<T>::init(std::shared_ptr<rclcpp::Node> node)
{
  _node = node;
  _identity.resize(STATE_SIZE, STATE_SIZE);
  _identity.setIdentity();
  _model.init(node);
}

template<class T>
void Ekf<T>::correct(const Measurement & measurement)
{
  Eigen::VectorXd& state = _model_as_base->get_state_unsafe();
  FB_DEBUG(
    "---------------------- Ekf::correct ----------------------\n" <<
      "State is:\n" <<
      state <<
      "\n"
      "Topic is:\n" <<
      measurement.topic_name_ <<
      "\n"
      "Measurement is:\n" <<
      measurement.measurement_ <<
      "\n"
      "Measurement topic name is:\n" <<
      measurement.topic_name_ <<
      "\n\n"
      "Measurement covariance is:\n" <<
      measurement.covariance_ << "\n");

  // We don't want to update everything, so we need to build matrices that only
  // update the measured parts of our state vector. Throughout prediction and
  // correction, we attempt to maximize efficiency in Eigen.

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
  Eigen::MatrixXd state_to_measurement_subset(update_size, state.rows());  // H
  Eigen::MatrixXd kalman_gain_subset(state.rows(), update_size);          // K
  Eigen::VectorXd innovation_subset(update_size);  // z - Hx

  state_subset.setZero();
  measurement_subset.setZero();
  measurement_covariance_subset.setZero();
  state_to_measurement_subset.setZero();
  kalman_gain_subset.setZero();
  innovation_subset.setZero();

  // Now build the sub-matrices from the full-sized matrices
  for (size_t i = 0; i < update_size; ++i) {
    measurement_subset(i) = measurement.measurement_(update_indices[i]);
    state_subset(i) = state(update_indices[i]);

    for (size_t j = 0; j < update_size; ++j) {
      measurement_covariance_subset(i, j) =
        measurement.covariance_(update_indices[i], update_indices[j]);
    }

    // Handle negative (read: bad) covariances in the measurement. Rather
    // than exclude the measurement or make up a covariance, just take
    // the absolute value.
    if (measurement_covariance_subset(i, i) < 0) {
      FB_DEBUG(
        "WARNING: Negative covariance for index " <<
          i << " of measurement (value is" <<
          measurement_covariance_subset(i, i) <<
          "). Using absolute value...\n");

      measurement_covariance_subset(i, i) =
        ::fabs(measurement_covariance_subset(i, i));
    }

    // If the measurement variance for a given variable is very
    // near 0 (as in e-50 or so) and the variance for that
    // variable in the covariance matrix is also near zero, then
    // the Kalman gain computation will blow up. Really, no
    // measurement can be completely without error, so add a small
    // amount in that case.
    if (measurement_covariance_subset(i, i) < 1e-9) {
      FB_DEBUG(
        "WARNING: measurement had very small error covariance for index " <<
          update_indices[i] <<
          ". Adding some noise to maintain filter stability.\n");

      measurement_covariance_subset(i, i) = 1e-9;
    }
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

  // (1) Compute the Kalman gain: K = (PH') / (HPH' + R)
  Eigen::MatrixXd& state_error_covariance = _model.get_state_covariance_unsafe();
  Eigen::MatrixXd pht =
    state_error_covariance * state_to_measurement_subset.transpose();
  Eigen::MatrixXd hphr_inverse =
    (state_to_measurement_subset * pht + measurement_covariance_subset)
    .inverse();
  kalman_gain_subset.noalias() = pht * hphr_inverse;

  innovation_subset = (measurement_subset - state_subset);

  // Wrap angles in the innovation
  for (size_t i = 0; i < update_size; ++i) {
    if (update_indices[i] == StateMemberRoll ||
      update_indices[i] == StateMemberPitch ||
      update_indices[i] == StateMemberYaw)
    {
      innovation_subset(i) = ::angles::normalize_angle(innovation_subset(i));
    }
  }

  //(2) Check Mahalanobis distance between mapped measurement and state.
  if (filter_utilities::check_mahalanobis_threshold(
      innovation_subset, hphr_inverse,
      measurement.mahalanobis_thresh_))
  {
    // (3) Apply the gain to the difference between the state and measurement: x
    // = x + K(z - Hx)
    state.noalias() += kalman_gain_subset * innovation_subset;

    // (4) Update the estimate error covariance using the Joseph form: (I -
    // KH)P(I - KH)' + KRK'
    Eigen::MatrixXd gain_residual = _identity;
    gain_residual.noalias() -= kalman_gain_subset * state_to_measurement_subset;
    state_error_covariance =
      gain_residual * state_error_covariance * gain_residual.transpose();
    state_error_covariance.noalias() += kalman_gain_subset *
      measurement_covariance_subset *
      kalman_gain_subset.transpose();

    // Handle wrapping of angles
    filter_utilities::wrapStateAngles(state);
    _model.set_state_covariance(state_error_covariance);
    FB_DEBUG(
      "Kalman gain subset is:\n" <<
        kalman_gain_subset << "\nInnovation is:\n" <<
        innovation_subset << "\nCorrected full state is:\n" <<
        state << "\nCorrected full estimate error covariance is:\n" <<
        state_error_covariance <<
        "\n\n---------------------- /Ekf::correct ----------------------\n");
  }
  else
  {
    FB_DEBUG(
      "Innovation mahalanobis distance test failed.");
  }
}


template<class T>
void Ekf<T>::predict(
  const rclcpp::Time & reference_time,
  const rclcpp::Duration & delta)
{
  const double delta_sec = filter_utilities::toSec(delta);
  // very interesting, without ModelBase it does not work...
  _model.ModelBase::step(reference_time, delta_sec);
}  // namespace rpp_localization


template<class T>
bool Ekf<T>::get_debug() {return _debug;}


template<class T>
void Ekf<T>::set_debug(const bool debug, std::ostream * out_stream)
{
  if (debug) {
    if (out_stream != NULL) {
      _debug_stream = out_stream;
      _debug = true;
    } else {
      _debug = false;
    }
  } else {
    _debug = false;
  }
}

}

template class rpp_localization::Ekf<rpp_localization::ConstantAccelerationModel>;