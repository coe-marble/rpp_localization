/*
 * SPDX-FileCopyrightText: (c) 2016, TNO IVS Helmond.
 * SPDX-License-Identifier: BSD-3-Clause
 */
#include "rpp_localization/filters/estimator.hpp"

#include <memory>
#include <vector>

#include "boost/circular_buffer.hpp"
#include "Eigen/Dense"
#include "rpp_localization/filters/ekf.hpp"
#include "rpp_localization/filters/ukf.hpp"

namespace rpp_localization
{
// TODO(anyone): Port to ROS 2 Ukf constructor way, where 3 separate parameters
// are accepted for alpha, kappa, beta, not just a vector, for filter_args
RppLocalizationEstimator::RppLocalizationEstimator(
  unsigned int buffer_capacity,
  FilterTypes::FilterType filter_type,
  const Eigen::MatrixXd & process_noise_covariance,
  const std::vector<double> & filter_args)
{
  state_buffer_.set_capacity(buffer_capacity);

  // Set up the filter that is used for predictions
  if (filter_type == FilterTypes::EKF) {
    filter_ = std::make_unique<Ekf>();
  } else if (filter_type == FilterTypes::UKF) {
    if (filter_args.size() < 3) {
      filter_ = std::make_unique<Ukf>();
    } else {
      filter_ = std::make_unique<Ukf>();
      dynamic_cast<Ukf *>(filter_.get())->setConstants(
        filter_args[0], filter_args[1],
        filter_args[2]);
    }
  }

  filter_->get_process_noise_covariance(process_noise_covariance);
}

RppLocalizationEstimator::~RppLocalizationEstimator()
{
}

void RppLocalizationEstimator::set_state(const EstimatorState & state)
{
  // If newly received state is newer than any in the buffer, push back
  if (state_buffer_.empty() || state.time_stamp >
    state_buffer_.back().time_stamp)
  {
    state_buffer_.push_back(state);
    // If it is older, put it in the right position
  } else {
    for (boost::circular_buffer<EstimatorState>::iterator it =
      state_buffer_.begin(); it != state_buffer_.end(); ++it)
    {
      if (state.time_stamp < it->time_stamp) {
        state_buffer_.insert(it, state);
        return;
      }
    }
  }
}

EstimatorResults::EstimatorResult RppLocalizationEstimator::get_state(
  const double time,
  EstimatorState & state) const
{
  // If there's nothing in the buffer, there's nothing to give.
  if (state_buffer_.size() == 0) {
    return EstimatorResults::EmptyBuffer;
  }

  // Set state to the most recent one for now
  state = state_buffer_.back();

  // Go through buffer from new to old
  EstimatorState last_state_before_time = state_buffer_.front();
  EstimatorState next_state_after_time = state_buffer_.back();
  bool previous_state_found = false;
  bool next_state_found = false;

  for (boost::circular_buffer<EstimatorState>::const_reverse_iterator it =
    state_buffer_.rbegin(); it != state_buffer_.rend(); ++it)
  {
    /* If the time stamp of the current state from the buffer is
     * older than the requested time, store it as the last state
     * before the requested time. If it is younger, save it as the
     * next one after, and go on to find the last one before.
     */
    if (it->time_stamp == time) {
      state = *it;
      return EstimatorResults::Exact;
    } else if (it->time_stamp <= time) {
      last_state_before_time = *it;
      previous_state_found = true;
      break;
    } else {
      next_state_after_time = *it;
      next_state_found = true;
    }
  }

  // If we found a previous state and a next state, we can do interpolation
  if (previous_state_found && next_state_found) {
    interpolate(last_state_before_time, next_state_after_time, time, state);
    return EstimatorResults::Interpolation;
    // If only a previous state is found, we can do extrapolation into the future
  } else if (previous_state_found) {
    extrapolate(last_state_before_time, time, state);
    return EstimatorResults::ExtrapolationIntoFuture;
    // If only a next state is found, we'll have to extrapolate into the past.
  } else if (next_state_found) {
    extrapolate(next_state_after_time, time, state);
    return EstimatorResults::ExtrapolationIntoPast;
  } else {
    return EstimatorResults::Failed;
  }
}

void RppLocalizationEstimator::setBufferCapacity(const int capacity)
{
  state_buffer_.set_capacity(capacity);
}

void RppLocalizationEstimator::clearBuffer()
{
  state_buffer_.clear();
}

unsigned int RppLocalizationEstimator::getBufferCapacity() const
{
  return state_buffer_.capacity();
}

unsigned int RppLocalizationEstimator::getSize() const
{
  return state_buffer_.size();
}

void RppLocalizationEstimator::extrapolate(
  const EstimatorState & boundary_state,
  const double requested_time,
  EstimatorState & state_at_req_time) const
{
  // Set up the filter with the boundary state
  filter_->set_state(boundary_state.state);
  filter_->set_estimate_error_covariance(boundary_state.covariance);

  // Calculate how much time we need to extrapolate into the future
  double delta = requested_time - boundary_state.time_stamp;

  // Use the filter to predict
  rclcpp::Time time_stamp = rclcpp::Time(
    boundary_state.time_stamp *
    1000000000);
  rclcpp::Duration delta_duration = rclcpp::Duration::from_seconds(delta);
  filter_->predict(time_stamp, delta_duration);

  state_at_req_time.time_stamp = requested_time;
  state_at_req_time.state = filter_->get_state();
  state_at_req_time.covariance = filter_->get_estimate_error_covariance();
}

void RppLocalizationEstimator::interpolate(
  const EstimatorState & given_state_1,
  const EstimatorState & /*given_state_2*/,
  const double requested_time,
  EstimatorState & state_at_req_time) const
{
  /*
   * TODO(anyone): Right now, we only extrapolate from the last known state
   * before the requested time. But as the state after the requested time is
   * also known, we may want to perform interpolation between states.
   */
  extrapolate(given_state_1, requested_time, state_at_req_time);
}

}  // namespace rpp_localization
