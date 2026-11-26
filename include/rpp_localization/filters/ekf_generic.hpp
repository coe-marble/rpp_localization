/*
 * SPDX-FileCopyrightText: (c) 2014, 2015, 2016 Charles River Analytics, Inc.
 * SPDX-FileCopyrightText: (c) 2017, Locus Robotics, Inc.
 * SPDX-License-Identifier: BSD-3-Clause
 */
#ifndef RPP_LOCALIZATION__FILTERS__EKF_GENERIC_HPP_
#define RPP_LOCALIZATION__FILTERS__EKF_GENERIC_HPP_

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <memory>
#include <stdexcept>
#include <utility>
#include <vector>

#include "Eigen/Dense"
#include "angles/angles.h"
#include "rpp_localization/core/filter_base.hpp"
#include "rpp_localization/core/filter_common.hpp"
#include "rpp_localization/core/filter_utilities.hpp"
#include "rpp_localization/core/validation.hpp"

namespace rpp_localization
{

/**
 * @brief Experimental EKF with runtime-bound dimensions and scalar angle coordinates.
 *
 * Keeps the legacy masked-state observation and ModelBase prediction boundary
 * for a direct numerical performance comparison with Ekf. This is not yet an
 * adapter for the shared Estimator/Model interfaces. State and covariance remain
 * in the injected model. Bindings stay fixed for the lifetime of the filter.
 * The model must retain its configured state and covariance dimensions.
 */

class EkfGeneric : public FilterBase
{
  public:

    explicit EkfGeneric(ModelBase& model, std::vector<Eigen::Index> angle_indices = {});
    ~EkfGeneric();

    // Both comparison bindings call the same algorithm on the same backing
    // buffers. Only their Eigen view types differ. The filter must outlive them.
    class CorrectionBinding
    {
      public:
        void correct(const Measurement& measurement) const
        {
          kernel_(*filter_, measurement);
        }

      private:
        friend class EkfGeneric;
        using Kernel = void (*)(EkfGeneric&, const Measurement&);
        CorrectionBinding(EkfGeneric& filter, Kernel kernel)
        : filter_(&filter), kernel_(kernel) {}

        EkfGeneric* filter_;
        Kernel kernel_;
    };

    template<int StateSize>
    CorrectionBinding prepare_correction();

    std::unique_ptr<FilterBase> clone() const override
    {
      return std::make_unique<EkfGeneric>(*this);
    }


    void correct(const Measurement& measurement) override;

    void predict(TimestampNs reference_time, DurationNs delta) override;

  private:
    template<int StateSize>
    void correct_impl(const Measurement& measurement);

    template<int StateSize>
    static void correct_bound(EkfGeneric& filter, const Measurement& measurement)
    {
      filter.correct_impl<StateSize>(measurement);
    }

    struct CorrectionWorkspace
    {
      CorrectionWorkspace(Eigen::Index state_size, Eigen::Index measurement_size)
      : state_subset(measurement_size),
        measurement_subset(measurement_size),
        measurement_covariance_subset(measurement_size, measurement_size),
        state_to_measurement_subset(measurement_size, state_size),
        kalman_gain_subset(state_size, measurement_size),
        innovation_subset(measurement_size),
        pht(state_size, measurement_size),
        innovation_covariance(measurement_size, measurement_size),
        innovation_covariance_ldlt(measurement_size),
        kalman_gain_transpose(measurement_size, state_size),
        weighted_innovation(measurement_size),
        gain_measurement_covariance(state_size, measurement_size)
      {
      }

      Eigen::VectorXd state_subset;
      Eigen::VectorXd measurement_subset;
      Eigen::MatrixXd measurement_covariance_subset;
      Eigen::MatrixXd state_to_measurement_subset;
      Eigen::MatrixXd kalman_gain_subset;
      Eigen::VectorXd innovation_subset;
      Eigen::MatrixXd pht;
      Eigen::MatrixXd innovation_covariance;
      Eigen::LDLT<Eigen::MatrixXd> innovation_covariance_ldlt;
      Eigen::MatrixXd kalman_gain_transpose;
      Eigen::VectorXd weighted_innovation;
      Eigen::MatrixXd gain_measurement_covariance;
    };

    struct Workspace
    {
      explicit Workspace(Eigen::Index state_size)
      : update_indices(static_cast<std::size_t>(state_size)),
        gain_residual(state_size, state_size),
        projected_covariance(state_size, state_size)
      {
        corrections.reserve(static_cast<std::size_t>(state_size) + 1);
        for (Eigen::Index size = 0; size <= state_size; ++size) {
          corrections.emplace_back(state_size, size);
        }
      }

      std::vector<Eigen::Index> update_indices;
      std::vector<CorrectionWorkspace> corrections;
      Eigen::MatrixXd gain_residual;
      Eigen::MatrixXd projected_covariance;
    };

    // Replay clones use the same model and are called serially. Scratch can be
    // shared without copying it into every numerical checkpoint.
    std::shared_ptr<Workspace> workspace_;
    std::vector<Eigen::Index> angle_indices_;
    std::vector<unsigned char> angle_states_;
};

}  // namespace rpp_localization

namespace rpp_localization
{

inline EkfGeneric::EkfGeneric(ModelBase& model, std::vector<Eigen::Index> angle_indices)
: FilterBase(static_cast<int>(model.get_state().size())),
  angle_indices_(std::move(angle_indices)),
  angle_states_(static_cast<std::size_t>(_state_dim), 0)
{
  if (model.get_state_covariance().rows() != _state_dim ||
    model.get_state_covariance().cols() != _state_dim)
  {
    throw std::invalid_argument("EKF generic model covariance must match its state dimension");
  }
  for (const auto index : angle_indices_) {
    if (index < 0 || index >= _state_dim || angle_states_[static_cast<std::size_t>(index)]) {
      throw std::invalid_argument("EKF generic angle indices must be unique state coordinates");
    }
    angle_states_[static_cast<std::size_t>(index)] = 1;
  }

  set_model_base(model);
  workspace_ = std::make_shared<Workspace>(_state_dim);
}

inline EkfGeneric::~EkfGeneric() {}

template<int StateSize>
inline EkfGeneric::CorrectionBinding EkfGeneric::prepare_correction()
{
  static_assert(StateSize == Eigen::Dynamic || StateSize > 0, "Invalid EKF state size");
  if constexpr (StateSize != Eigen::Dynamic) {
    if (_state_dim != StateSize) {
      throw std::invalid_argument("EKF comparison binding must match the configured state dimension");
    }
  }
  return CorrectionBinding(*this, &correct_bound<StateSize>);
}

inline void EkfGeneric::correct(const Measurement & measurement)
{
  correct_impl<Eigen::Dynamic>(measurement);
}

template<int StateSize>
inline void EkfGeneric::correct_impl(const Measurement& measurement)
{
  using StateVectorType = Eigen::Matrix<double, StateSize, 1>;
  using StateMatrixType = Eigen::Matrix<double, StateSize, StateSize>;
  using ObservationMatrixType = Eigen::Matrix<double, Eigen::Dynamic, StateSize>;
  using GainMatrixType = Eigen::Matrix<double, StateSize, Eigen::Dynamic>;
  const Eigen::Index state_size = _state_dim;
  Eigen::Map<StateVectorType> state(_model_as_base->get_state_unsafe().data(), state_size);
  FB_DEBUG(
    "---------------------- EkfGeneric::correct ----------------------\n" <<
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

  auto& update_indices = workspace_->update_indices;
  Eigen::Index update_size = 0;
  const auto mask_size = std::min<std::size_t>(
    measurement.update_vector_.size(), static_cast<std::size_t>(_state_dim));
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

  using IndexRow = Eigen::Matrix<Eigen::Index, 1, Eigen::Dynamic, Eigen::RowMajor>;
  FB_DEBUG(
    "Update indices are:\n" <<
      IndexRow::Map(update_indices.data(), update_size) << "\n");

  auto& correction = workspace_->corrections[static_cast<std::size_t>(update_size)];
  auto& state_subset = correction.state_subset;
  auto& measurement_subset = correction.measurement_subset;
  auto& measurement_covariance_subset = correction.measurement_covariance_subset;
  Eigen::Map<ObservationMatrixType> state_to_measurement_subset(
    correction.state_to_measurement_subset.data(), update_size, state_size);
  Eigen::Map<GainMatrixType> kalman_gain_subset(
    correction.kalman_gain_subset.data(), state_size, update_size);
  auto& innovation_subset = correction.innovation_subset;

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
  Eigen::Map<StateMatrixType> state_error_covariance(
    _model_as_base->get_state_covariance_unsafe().data(), state_size, state_size);
  Eigen::Map<GainMatrixType> pht(correction.pht.data(), state_size, update_size);
  pht.noalias() = state_error_covariance * state_to_measurement_subset.transpose();
  auto& innovation_covariance = correction.innovation_covariance;
  innovation_covariance.noalias() = state_to_measurement_subset * pht;
  innovation_covariance += measurement_covariance_subset;
  auto& innovation_covariance_ldlt = correction.innovation_covariance_ldlt;
  innovation_covariance_ldlt.compute(innovation_covariance);
  if (innovation_covariance_ldlt.info() != Eigen::Success) {
    FB_DEBUG("Innovation covariance factorization failed.");
    return;
  }
  // K' = (HPH' + R)^-1 HP', since HPH' + R is symmetric.
  Eigen::Map<ObservationMatrixType> kalman_gain_transpose(
    correction.kalman_gain_transpose.data(), update_size, state_size);
  kalman_gain_transpose = innovation_covariance_ldlt.solve(pht.transpose());
  kalman_gain_subset = kalman_gain_transpose.transpose();

  innovation_subset = (measurement_subset - state_subset);

  // Wrap angles in the innovation
  for (Eigen::Index i = 0; i < update_size; ++i) {
    const auto row = update_indices[static_cast<std::size_t>(i)];
    if (angle_states_[static_cast<std::size_t>(row)])
    {
      innovation_subset(i) = ::angles::normalize_angle(innovation_subset(i));
    }
  }

  //(2) Check Mahalanobis distance between mapped measurement and state.
  auto& weighted_innovation = correction.weighted_innovation;
  weighted_innovation = innovation_covariance_ldlt.solve(innovation_subset);
  if (filter_utilities::check_squared_mahalanobis_threshold(
      innovation_subset.dot(weighted_innovation),
      measurement.mahalanobis_thresh_))
  {
    // (3) Apply the gain to the difference between the state and measurement: x
    // = x + K(z - Hx)
    state.noalias() += kalman_gain_subset * innovation_subset;

    // (4) Update the estimate error covariance using the Joseph form: (I -
    // KH)P(I - KH)' + KRK'
    Eigen::Map<StateMatrixType> gain_residual(
      workspace_->gain_residual.data(), state_size, state_size);
    gain_residual.setIdentity();
    gain_residual.noalias() -= kalman_gain_subset * state_to_measurement_subset;
    Eigen::Map<StateMatrixType> projected_covariance(
      workspace_->projected_covariance.data(), state_size, state_size);
    projected_covariance.noalias() = gain_residual * state_error_covariance;
    state_error_covariance.noalias() = projected_covariance * gain_residual.transpose();
    Eigen::Map<GainMatrixType> gain_measurement_covariance(
      correction.gain_measurement_covariance.data(), state_size, update_size);
    gain_measurement_covariance.noalias() = kalman_gain_subset * measurement_covariance_subset;
    state_error_covariance.noalias() +=
      gain_measurement_covariance * kalman_gain_subset.transpose();

    // Handle wrapping of angles
    for (const auto index : angle_indices_) {
      state(index) = angles::normalize_angle(state(index));
    }
    FB_DEBUG(
      "Kalman gain subset is:\n" <<
        kalman_gain_subset << "\nInnovation is:\n" <<
        innovation_subset << "\nCorrected full state is:\n" <<
        state << "\nCorrected full estimate error covariance is:\n" <<
        state_error_covariance <<
        "\n\n---------------------- /EkfGeneric::correct ----------------------\n");
  }
  else
  {
    FB_DEBUG(
      "Innovation mahalanobis distance test failed.");
  }
}


inline void EkfGeneric::predict(
  const TimestampNs reference_time,
  const DurationNs delta)
{
  _model_as_base->predict(reference_time, delta);
}

}  // namespace rpp_localization

#endif  // RPP_LOCALIZATION__FILTERS__EKF_GENERIC_HPP_
