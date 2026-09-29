/*
 * SPDX-FileCopyrightText: (c) 2014, 2015, 2016 Charles River Analytics, Inc.
 * SPDX-License-Identifier: BSD-3-Clause
 */
#ifndef RPP_LOCALIZATION__UKF_HPP_
#define RPP_LOCALIZATION__UKF_HPP_

#include <vector>

#include "Eigen/Dense"
#include "rclcpp/time.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rpp_localization/core/filter_base.hpp"
#include "rpp_localization/core/measurement.hpp"

#include "rpp_localization/models/constant_acc_model.hpp"
#include "rpp_localization/core/filter_utilities.hpp"

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

template<class T> // model type
class Ukf : public FilterBase
{
public:
  /**
   * @brief  Constructor for the Ukf class
   *
   * @param[in] args - Generic argument container. It is assumed that args[0]
   * constains the alpha parameter, args[1] contains the kappa parameter, and
   * args[2] contains the beta parameter.
   */
  Ukf(int state_space_dim);

  /**
   * @brief  Destructor for the Ukf class
   */
  ~Ukf();

  void init(rclcpp::Node& node);

  void correct(const Measurement& measurement) override;

  void predict(TimestampNs reference_time, DurationNs delta) override;

  void setConstants(double alpha, double kappa, double beta);

  bool get_debug();
  void set_debug(const bool debug, std::ostream * out_stream);

protected:
  /**
   * @brief  Computes the weighted covariance and sigma points
   */
  void generateSigmaPoints(const Eigen::VectorXd& state, const Eigen::MatrixXd& covariance);

  /**
   * @brief  Carries out the predict step for the posteriori state of a sigma point
   * @param[in,out] sigma_point - The sigma point (state vector) to project
   * @param[in] delta - The time step over which to project
   */
  void projectSigmaPoint(
    TimestampNs reference_time,
    Eigen::VectorXd& sigma_point,
    DurationNs delta);


  void load_params(rclcpp::Node& node);

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
  bool _use_dynamic_process_noise_covariance;

  Eigen::MatrixXd process_noise_covariance_;

  bool _debug;
  std::ostream* _debug_stream;
  T _model;
};

}  // namespace rpp_localization

#endif  // RPP_LOCALIZATION__UKF_HPP_
