/* ----------------------------------------------------------------------------
 * Copyright 2023, Luigi Scarfone <luigi.scarfone@unical.it>
 * All Rights Reserved 
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 *  @file   inertial_process.h
 *  @author Luigi Scarfone
 *  @brief  Header file for Inertial Process with bias measurements (thread-safe)
 *          Requries "Imperfect InEKF" since biases don't fit into Lie Group structure
 *  @date   April 18, 2023
 **/

#ifndef INERTIAL_PROCESS_
#define INERTIAL_PROCESS_

#include <Eigen/Core>
#include <unsupported/Eigen/MatrixFunctions>
#include "rclcpp/time.hpp"
#include "rpp_localization/core/measurement.hpp"
#include "rpp_localization/core/model_base.hpp"
#include "rpp_localization/models/nav_model_base.hpp"
#include "rpp_localization/core/filter_utilities.hpp"
#include "rpp_localization/core/filter_common.hpp"
#include "rpp_localization/inekf/so3.hpp"
#include "rpp_localization/inekf/se3.hpp"
#include "rpp_localization/inekf/lie_group.hpp"

typedef Eigen::Matrix<double,6,1> Vector6d;
typedef Eigen::Matrix<double,15,15> MatrixCov;

namespace rpp_localization::InEKF{

class InertialProcess : public NavModelBase, SE3<2,6>
{
public:
    InertialProcess(int dim);       // dim is 15 but the inekf takes care only about R, v and p vectors
    ~InertialProcess();

    // Overriding from NavModelBase
    void init(rclcpp::Node& node);
    void step(Eigen::VectorXd& state, Eigen::MatrixXd& state_covariance, const rclcpp::Time & reference_time, const double dT) override;

    // Setting Input Control Noise and Biases
    void setGyroNoise(Eigen::Matrix3d M);
    void setAccelNoise(Eigen::Matrix3d M);
    void setGyroBiasNoise(double std);
    void setAccelBiasNoise(double std);

    // Manage Lie Group SE3 State
    void set_lie_state (const SE3<2,6>& state_lie) { state_lie_ = state_lie; };
    void set_lie_state (const Eigen::VectorXd& state);
    SE3<2,6>& get_lie_state() { return state_lie_; };
    void set_lie_covariance(const Eigen::MatrixXd& cov, const Eigen::MatrixXd& R);


private:
    void load_params(rclcpp::Node& node);

    // Define gravitational vector (optional since rpp_localization has this function)
    Eigen::Vector3d g_ = (Eigen::Vector3d() << 0,0,-9.81).finished();

    // Error type of the filter
    ERROR _error;

    /**
     * @brief Create Process Model Covariance
     *        Set and Get Matrix Process Model Covariance
     */
    MatrixCov Q_;
    MatrixCov getQ() const {return Q_;};
    void setQ(MatrixCov Q) {Q_ = Q;};

    /**
     * @brief Integrates IMU measurements.
     * 
     * @param u Control. First 3 are angular velocity, last 3 are linear acceleration.
     * @param dt Delta time
     * @param state Current state
     * @return Integrated state
     */
    void propagateState(const Vector6d& u, double dt, SE3<2,6>& state);

    /**
     * @brief Make a discrete time linearized process model matrix, with \f$\Phi = \exp(A\Delta t) \f$.
     *        Since this is used in an "Imperfect InEKF", both left and right versions are slightly state dependent.
     *
     * @param u Control - IMU Measurements
     * @param dt Delta time
     * @param state Current state estimate (shouldn't be needed unless doing an "Imperfect InEKF")
     * @param error Right or left error. Function should be implemented to handle both.
     * @return Phi
     */
    MatrixCov makePhi(const Eigen::Matrix<double, 6, 1>& u, double dt, const SE3<2,6>& state, InEKF::ERROR error);

    /**
     * @brief create new state Matrix Group Lie
    */
    SE3<2,6> state_lie_;
};

} // namespace rpp_localization::InEKF

#endif  // INERTIAL_PROCESS_
