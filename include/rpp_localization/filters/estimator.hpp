/*
 * SPDX-FileCopyrightText: (c) 2016, TNO IVS Helmond.
 * SPDX-License-Identifier: BSD-3-Clause
 */
#ifndef RPP_LOCALIZATION__RPP_LOCALIZATION_ESTIMATOR_HPP_
#define RPP_LOCALIZATION__RPP_LOCALIZATION_ESTIMATOR_HPP_

#include <ostream>
#include <memory>
#include <vector>

#include "boost/circular_buffer.hpp"
#include "Eigen/Dense"
#include "rpp_localization/core/filter_base.hpp"
#include "rpp_localization/core/filter_common.hpp"

namespace rpp_localization
{

struct Twist
{
  Eigen::Vector3d linear;
  Eigen::Vector3d angular;
};

//! @brief Robot Localization Estimator State
//!
//! The Estimator State data structure bundles the state information of the
//! estimator.
//!
struct EstimatorState
{
  EstimatorState()
  : time_stamp(0.0),
    state(STATE_SIZE),
    covariance(STATE_SIZE, STATE_SIZE)
  {
    state.setZero();
    covariance.setZero();
  }

  //! @brief Time at which this state is/was achieved
  double time_stamp;

  //! @brief System state at time = time_stamp
  Eigen::VectorXd state;

  //! @brief System state covariance at time = time_stamp
  Eigen::MatrixXd covariance;

  friend std::ostream & operator<<(std::ostream & os, const EstimatorState & state)
  {
    return os << "state:\n - time_stamp: " << state.time_stamp <<
           "\n - state: \n" << state.state <<
           " - covariance: \n" << state.covariance;
  }
};

namespace EstimatorResults
{
enum EstimatorResult
{
  ExtrapolationIntoFuture = 0,
  Interpolation,
  ExtrapolationIntoPast,
  Exact,
  EmptyBuffer,
  Failed
};
}  // namespace EstimatorResults

namespace FilterTypes
{
enum FilterType
{
  EKF = 0,
  UKF,
  NotDefined
};
}  // namespace FilterTypes

//! @brief Robot Localization Listener class
//!
//! The Robot Localization Estimator class buffers states of and inputs to a
//! system and can interpolate and extrapolate based on a given system model.
//!
class RppLocalizationEstimator
{
public:
  //! @brief Constructor for the RppLocalizationListener class
  //!
  //! @param[in] args - Generic argument container (not used here, but needed
  //! so that the ROS filters can pass arbitrary arguments to templated filter
  //! types).
  //!
  explicit RppLocalizationEstimator(
    unsigned int buffer_capacity,
    FilterTypes::FilterType filter_type,
    const Eigen::MatrixXd & process_noise_covariance,
    const std::vector<double> & filter_args = std::vector<double>());

  //! @brief Destructor for the RppLocalizationListener class
  //!
  virtual ~RppLocalizationEstimator();

  //! @brief Sets the current internal state of the listener.
  //!
  //! @param[in] state - The new state vector to set the internal state to
  //!
  void set_state(const EstimatorState & state);

  //! @brief Returns the state at a given time
  //!
  //! Projects the current state and error matrices forward using a model of the robot's motion.
  //!
  //! @param[in] time - The time to which the prediction is being made
  //! @param[out] state - The returned state at the given time
  //!
  //! @return GetStateResult enum
  //!
  EstimatorResults::EstimatorResult get_state(
    const double time,
    EstimatorState & state) const;

  //! @brief Clears the internal state buffer
  //!
  void clearBuffer();

  //! @brief Sets the buffer capacity
  //!
  //! @param[in] capacity - The new buffer capacity
  //!
  void setBufferCapacity(const int capacity);

  //! @brief Returns the buffer capacity
  //!
  //! Returns the number of EstimatorState objects that can be pushed to the
  //! buffer before old ones are dropped. (The capacity of the buffer).
  //!
  //! @return buffer capacity
  //!
  unsigned int getBufferCapacity() const;

  //! @brief Returns the current buffer size
  //!
  //! Returns the number of EstimatorState objects currently in the buffer.
  //!
  //! @return current buffer size
  //!
  unsigned int getSize() const;

private:
  friend std::ostream & operator<<(
    std::ostream & os,
    const RppLocalizationEstimator & rle)
  {
    for (boost::circular_buffer<EstimatorState>::const_iterator it =
      rle.state_buffer_.begin(); it != rle.state_buffer_.end(); ++it)
    {
      os << *it << "\n";
    }
    return os;
  }

  //! @brief Extrapolates the given state to a requested time stamp
  //!
  //! @param[in] boundary_state - state from which to extrapolate
  //! @param[in] requested_time - time stamp to extrapolate to
  //! @param[out] state_at_req_time - predicted state at requested time
  //!
  void extrapolate(
    const EstimatorState & boundary_state,
    const double requested_time,
    EstimatorState & state_at_req_time) const;

  //! @brief Interpolates the given state to a requested time stamp
  //!
  //! @param[in] given_state_1 - last state update before requested time
  //! @param[in] given_state_2 - next state update after requested time
  //! @param[in] requested_time - time stamp to extrapolate to
  //! @param[out] state_at_req_time - predicted state at requested time
  //!
  void interpolate(
    const EstimatorState & given_state_1,
    const EstimatorState & /*given_state_2*/,
    const double requested_time, EstimatorState & state_at_req_time) const;

  //!
  //! @brief The buffer holding the system states that have come in.
  //! Interpolation and extrapolation is done starting from these states.
  //!
  boost::circular_buffer<EstimatorState> state_buffer_;

  //!
  //! @brief A pointer to the filter instance that is used for extrapolation
  //!
  std::unique_ptr<FilterBase> filter_;
};

}  // namespace rpp_localization

#endif  // RPP_LOCALIZATION__RPP_LOCALIZATION_ESTIMATOR_HPP_
