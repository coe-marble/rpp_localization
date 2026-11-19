/*
 * SPDX-FileCopyrightText: (c) 2014, 2015, 2016 Charles River Analytics, Inc.
 * SPDX-FileCopyrightText: (c) 2017, Locus Robotics, Inc.
 * SPDX-License-Identifier: BSD-3-Clause
 */
#ifndef RPP_LOCALIZATION__FILTERS__EXTENDED_KALMAN_FILTER_HPP_
#define RPP_LOCALIZATION__FILTERS__EXTENDED_KALMAN_FILTER_HPP_

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <stdexcept>

#include "Eigen/Dense"
#include "angles/angles.h"
#include "rpp_localization/core/filter_base.hpp"
#include "rpp_localization/core/filter_common.hpp"
#include "rpp_localization/core/filter_utilities.hpp"
#include "rpp_localization/core/validation.hpp"

namespace rpp_localization
{

/**
 * @brief Extended Kalman filter class
 *
 * Implementation of an extended Kalman filter (EKF). This class derives from
 * FilterBase and overrides the predict() and correct() methods in keeping with
 * the discrete time EKF algorithm.
 */

class Ekf : public FilterBase
{
  public:

    explicit Ekf(ModelBase& model);
    ~Ekf();

    std::unique_ptr<FilterBase> clone() const override
    {
      return std::make_unique<Ekf>(*this);
    }


    void correct(const Measurement& measurement) override;

    void predict(TimestampNs reference_time, DurationNs delta) override;
};

}  // namespace rpp_localization

namespace rpp_localization
{

inline Ekf::Ekf(ModelBase& model)
: FilterBase(static_cast<int>(model.get_state().size()))
{
  if (model.get_state().size() != STATE_SIZE) {
    throw std::invalid_argument("EKF runtime model must have 15 state elements");
  }

  set_model_base(model);
}

inline Ekf::~Ekf() {}

inline void Ekf::correct(const Measurement & measurement)
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

  // Fixed-capacity storage keeps the correction free of heap allocations.
  using SubsetVector = Eigen::Matrix<double, Eigen::Dynamic, 1, 0, STATE_SIZE, 1>;
  using SubsetMatrix =
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, 0, STATE_SIZE, STATE_SIZE>;
  using GainMatrix = Eigen::Matrix<double, STATE_SIZE, Eigen::Dynamic, 0, STATE_SIZE, STATE_SIZE>;
  using StateMatrix = Eigen::Matrix<double, STATE_SIZE, STATE_SIZE>;

  // First, determine how many state vector values we're updating
  std::array<Eigen::Index, STATE_SIZE> update_indices{};
  Eigen::Index update_size = 0;
  const auto mask_size = std::min<std::size_t>(measurement.update_vector_.size(), STATE_SIZE);
  for (std::size_t i = 0; i < mask_size; ++i) {
    if (measurement.update_vector_[i]) {
      const auto index = static_cast<Eigen::Index>(i);
      // Handle nan and inf values in measurements
      if (std::isnan(measurement.measurement_(index))) {
        FB_DEBUG(
          "Value at index " << i <<
            " was nan. Excluding from update.\n");
      } else if (std::isinf(measurement.measurement_(index))) {
        FB_DEBUG(
          "Value at index " << i <<
            " was inf. Excluding from update.\n");
      } else {
        update_indices[static_cast<std::size_t>(update_size++)] = index;
      }
    }
  }

  using IndexRow = Eigen::Matrix<Eigen::Index, 1, Eigen::Dynamic, Eigen::RowMajor, 1, STATE_SIZE>;
  FB_DEBUG(
    "Update indices are:\n" <<
      IndexRow::Map(update_indices.data(), update_size) << "\n");

  // Now set up the relevant matrices
  SubsetVector state_subset(update_size);       // x (in most literature)
  SubsetVector measurement_subset(update_size);  // z
  SubsetMatrix measurement_covariance_subset(update_size, update_size);  // R
  SubsetMatrix state_to_measurement_subset(update_size, STATE_SIZE);  // H
  GainMatrix kalman_gain_subset(STATE_SIZE, update_size);          // K
  SubsetVector innovation_subset(update_size);  // z - Hx

  state_to_measurement_subset.setZero();

  // Now build the sub-matrices from the full-sized matrices
  for (Eigen::Index i = 0; i < update_size; ++i) {
    const auto row = update_indices[static_cast<std::size_t>(i)];
    measurement_subset(i) = measurement.measurement_(row);
    state_subset(i) = state(row);

    for (Eigen::Index j = 0; j < update_size; ++j) {
      measurement_covariance_subset(i, j) =
        measurement.covariance_(row, update_indices[static_cast<std::size_t>(j)]);
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
          row <<
          ". Adding some noise to maintain filter stability.\n");
    }

    measurement_covariance_subset(i, i) = normalized_covariance.value;
  }

  // The state-to-measurement function, h, will now be a measurement_size x
  // full_state_size matrix, with ones in the (i, i) locations of the values to
  // be updated
  for (Eigen::Index i = 0; i < update_size; ++i) {
    state_to_measurement_subset(i, update_indices[static_cast<std::size_t>(i)]) = 1;
  }

  FB_DEBUG(
    "Current state subset is:\n" <<
      state_subset << "\nMeasurement subset is:\n" <<
      measurement_subset << "\nMeasurement covariance subset is:\n" <<
      measurement_covariance_subset <<
      "\nState-to-measurement subset is:\n" <<
      state_to_measurement_subset << "\n");

  // (1) Compute the Kalman gain: K = (PH') / (HPH' + R)
  Eigen::MatrixXd& state_error_covariance = _model_as_base->get_state_covariance_unsafe();
  GainMatrix pht(STATE_SIZE, update_size);
  pht.noalias() = state_error_covariance * state_to_measurement_subset.transpose();
  SubsetMatrix innovation_covariance(update_size, update_size);
  innovation_covariance.noalias() = state_to_measurement_subset * pht;
  innovation_covariance += measurement_covariance_subset;
  const Eigen::LDLT<SubsetMatrix> innovation_covariance_ldlt(innovation_covariance);
  if (innovation_covariance_ldlt.info() != Eigen::Success) {
    FB_DEBUG("Innovation covariance factorization failed.");
    return;
  }
  // K' = (HPH' + R)^-1 HP', since HPH' + R is symmetric.
  const SubsetMatrix kalman_gain_transpose = innovation_covariance_ldlt.solve(pht.transpose());
  kalman_gain_subset = kalman_gain_transpose.transpose();

  innovation_subset = (measurement_subset - state_subset);

  // Wrap angles in the innovation
  for (Eigen::Index i = 0; i < update_size; ++i) {
    const auto row = update_indices[static_cast<std::size_t>(i)];
    if (row == StateMemberRoll ||
      row == StateMemberPitch ||
      row == StateMemberYaw)
    {
      innovation_subset(i) = ::angles::normalize_angle(innovation_subset(i));
    }
  }

  //(2) Check Mahalanobis distance between mapped measurement and state.
  const SubsetVector weighted_innovation = innovation_covariance_ldlt.solve(innovation_subset);
  if (filter_utilities::check_squared_mahalanobis_threshold(
      innovation_subset.dot(weighted_innovation),
      measurement.mahalanobis_thresh_))
  {
    // (3) Apply the gain to the difference between the state and measurement: x
    // = x + K(z - Hx)
    state.noalias() += kalman_gain_subset * innovation_subset;

    // (4) Update the estimate error covariance using the Joseph form: (I -
    // KH)P(I - KH)' + KRK'
    StateMatrix gain_residual = StateMatrix::Identity();
    gain_residual.noalias() -= kalman_gain_subset * state_to_measurement_subset;
    StateMatrix projected_covariance;
    projected_covariance.noalias() = gain_residual * state_error_covariance;
    state_error_covariance.noalias() = projected_covariance * gain_residual.transpose();
    GainMatrix gain_measurement_covariance(STATE_SIZE, update_size);
    gain_measurement_covariance.noalias() = kalman_gain_subset * measurement_covariance_subset;
    state_error_covariance.noalias() +=
      gain_measurement_covariance * kalman_gain_subset.transpose();

    // Handle wrapping of angles
    filter_utilities::wrap_state_angles(state);
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


inline void Ekf::predict(
  const TimestampNs reference_time,
  const DurationNs delta)
{
  _model_as_base->predict(reference_time, delta);
}

}  // namespace rpp_localization

#endif  // RPP_LOCALIZATION__FILTERS__EXTENDED_KALMAN_FILTER_HPP_
