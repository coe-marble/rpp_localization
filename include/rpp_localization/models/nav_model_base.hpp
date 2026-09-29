#ifndef NAV_MODEL_BASE_H
#define NAV_MODEL_BASE_H

// SPDX-FileCopyrightText: 2021 Laboratory for Underwater Systems and Technologies (LABUST)
// SPDX-License-Identifier: Apache-2.0

#include <rpp_localization/core/model_base.hpp>
#include <rpp_localization/core/measurement.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tuple>
#include <iostream>

#define MB_DEBUG(msg) \
  if (get_debug()) { \
    *_debug_stream << msg; \
  }

namespace rpp_localization
{

class NavModelBase : public ModelBase
{
public:
    void init(std::shared_ptr<rclcpp::Node> node) override;
    void reset();

    virtual void step(Eigen::VectorXd& state, Eigen::MatrixXd& state_covariance, const rclcpp::Time & reference_time, const double dT) = 0;
    virtual ~NavModelBase();

    bool get_debug();


    bool get_initialized_status();
    const Eigen::MatrixXd & get_process_noise_covariance();
    const std::vector<bool>& get_control_update_vector();

    void compute_dynamic_process_noise_covariance(
      const Eigen::VectorXd & state, Eigen::MatrixXd& covariance);

    void set_control_params(
        const std::vector<bool> & update_vector,
        const rclcpp::Duration & control_timeout,
        const std::vector<double> & acceleration_limits,
        const std::vector<double> & acceleration_gains,
        const std::vector<double> & deceleration_limits,
        const std::vector<double> & deceleration_gains);

    void set_debug(const bool debug, std::ostream * out_stream = NULL);


    void set_process_noise_covariance(const Eigen::MatrixXd & process_noise_covariance);

    void validate_delta(rclcpp::Duration & delta);
protected:
    NavModelBase(int state_dim);

    void load_params();
    /**
    * @brief Method for settings bounds on acceleration values derived from
    * controls
    * @param[in] state - The current state variable (e.g., linear X velocity)
    * @param[in] control - The current control commanded velocity corresponding
    * to the state variable
    * @param[in] acceleration_limit - Limit for acceleration (regardless of
    * driving direction)
    * @param[in] acceleration_gain - Gain applied to acceleration control error
    * @param[in] deceleration_limit - Limit for deceleration (moving towards
    * zero, regardless of driving direction)
    * @param[in] deceleration_gain - Gain applied to deceleration control error
    * @return a usable acceleration estimate for the control vector
    */
    double computeControlAcceleration(
      const double state,
      const double control,
      const double acceleration_limit,
      const double acceleration_gain,
      const double deceleration_limit,
      const double deceleration_gain);

    /**
    * @brief Converts the control term to an acceleration to be applied in the
    * prediction step
    * @param[in] reference_time - The time of the update (measurement used in the
    * prediction step)
    */
    void prepareControl(
      const rclcpp::Time & reference_time,
      const double );

    /**
    * @brief Whether or not we've received any measurements
    */
    bool _initialized;

    /**
    * @brief Timeout value, in seconds, after which a control is considered stale
    */
    rclcpp::Duration _control_timeout;

    /**
    * @brief If true, uses the robot's vehicle state and the static process noise
    * covariance matrix to generate a dynamic process noise covariance matrix
    */
    bool _use_dynamic_process_noise_covariance;

    /**
    * @brief Gains applied to acceleration derived from control term
    */
    std::vector<double> acceleration_gains_;

    /**
    * @brief Caps the acceleration we apply from control input
    */
    std::vector<double> acceleration_limits_;

    /**
    * @brief Gains applied to deceleration derived from control term
    */
    std::vector<double> deceleration_gains_;

    /**
    * @brief Caps the deceleration we apply from control input
    */
    std::vector<double> deceleration_limits_;

    /**
    * @brief Which control variables are being used (e.g., not every vehicle is
    * controllable in Y or Z)
    */
    std::vector<bool> _control_update_vector;

    /**
    * @brief Variable that gets updated every time we process a measurement and
    * we have a valid control
    */
    Eigen::VectorXd control_acceleration_;

    /**
    * @brief Covariance matrices can be incredibly unstable. We can add a small
    * value to it at each iteration to help maintain its positive-definite
    * property.
    */
    Eigen::MatrixXd _covariance_epsilon;

    /**
    * @brief Gets updated when useDynamicProcessNoise_ is true
    */
    Eigen::MatrixXd dynamic_process_noise_covariance_;


    /**
    * @brief We need the identity for a few operations. Better to store it.
    */
    Eigen::MatrixXd _identity;

    /**
    * @brief As we move through the world, we follow a predict/update cycle. If
    * one were to imagine a scenario where all we did was make predictions
    * without correcting, the error in our position estimate would grow without
    * bound. This error is stored in the stateEstimateCovariance_ matrix.
    * However, this matrix doesn't answer the question of *how much* our error
    * should grow for each time step. That's where the processNoiseCovariance
    * matrix comes in. When we make a prediction using the transfer function, we
    * add this matrix (times delta_t) to the state estimate covariance matrix.
    */
    Eigen::MatrixXd process_noise_covariance_;

    /**
    * @brief Whether or not the filter is in debug mode
    */
    bool _debug;
    std::shared_ptr<rclcpp::Node> _node;
    std::ostream* _debug_stream;
};

}  // namespace rpp_localization


/* NAV_MODEL_BASE_H */
#endif