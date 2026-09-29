
#include "rpp_localization/inekf/inekf.hpp"

#include <cmath>
#include <vector>

#include "angles/angles.h"
#include "Eigen/Dense"
#include "rclcpp/time.hpp"
#include "rpp_localization/core/filter_base.hpp"
#include "rpp_localization/core/measurement.hpp"
#include "rpp_localization/core/filter_common.hpp"
#include "rpp_localization/core/filter_utilities.hpp"




namespace rpp_localization
{

template<class T>
InEkf<T>::InEkf(int state_space_dim)
: FilterBase(state_space_dim),
  _model(state_space_dim),
  _debug(false)             // Why set false by default???
{
  set_model_base(_model);
  _identity.resize(STATE_SIZE, STATE_SIZE);
  _identity.setIdentity();
}

template<class T>
InEkf<T>::~InEkf() {}

template<class T>
void InEkf<T>::init(std::shared_ptr<rclcpp::Node> node)
{
  _node = node;
  _model.init(node);

  // GET ERROR TYPE FROM CONFIGURATION FILE
  _node->declare_parameter("error", "RIGHT");
  use_pseudomeasurements_ = _node->declare_parameter("use_pseudomeasurements", false);
  pseudomeasurement_cov_lat_ = _node->declare_parameter<double>("pseudomeasurement_variance_lat", 1.0);
  pseudomeasurement_cov_alt_ = _node->declare_parameter<double>("pseudomeasurement_variance_alt", 10.0);
  std::string error{""};
  if(_node->get_parameter("error", error))
  {
    if (error == "RIGHT") error_ = InEKF::RIGHT;
    else if (error == "LEFT") error_ = InEKF::LEFT;
    else
    {
      //RCLCPP_ERROR("ERROR TYPE NOT SET PROPERLY!");
    }
  }
}

template<class T>
void InEkf<T>::correct(const Measurement & measurement)
{
  // GET STATE VECTOR AND STATE COVARIANCE MATRIX
  Eigen::VectorXd& state = _model.get_state_unsafe();
  Eigen::MatrixXd& state_error_covariance = _model.get_state_covariance_unsafe();

  FB_DEBUG(
    "---------------------- InEkf::correct ----------------------\n" <<
      "State is:\n" <<
      state <<
      "\n"
      "Measurement is:\n" <<
      measurement.measurement_ <<
      "\n"
      "Measurement topic name is:\n" <<
      measurement.topic_name_ <<
      "\n\n"
      "Measurement covariance is:\n" <<
      measurement.covariance_ << "\n");

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
      }
      else {
        update_indices.push_back(i);
      }
    }
  }

  FB_DEBUG("Update indices are:\n" << update_indices << "\n");
  auto ekf_update_indices = [] (const Measurement& measurement) -> std::vector<int>
  {
    std::vector<int> indices;
    for (size_t i = 0; i < ORIENTATION_SIZE; ++i)
    {
      if (measurement.update_vector_[ORIENTATION_OFFSET + i]) indices.push_back(i + ORIENTATION_OFFSET);
    }
    for (size_t i = 0; i < ANGULAR_VELOCITY_SIZE; ++i)
    {
      if (measurement.update_vector_[ORIENTATION_V_OFFSET + i]) indices.push_back(i + ORIENTATION_V_OFFSET);
    }
    for (size_t i = 0; i < ACCELERATION_SIZE; ++i)
    {
      if (measurement.update_vector_[POSITION_A_OFFSET + i]) indices.push_back(i + POSITION_A_OFFSET);
    }
    return indices;
  };

  // EKF UPDATE ACCELERATION, ANGULAR VELOCITY AND ANGLES
  auto indices = ekf_update_indices(measurement);
  auto update_size = indices.size();
  bool generate_pseudomeasurement = false;
  if (update_size > 0)
  {
    Eigen::VectorXd state_subset(update_size);       // x (in most literature)
    Eigen::VectorXd measurement_subset(update_size);       // x (in most literature)
    Eigen::MatrixXd measurement_covariance_subset(update_size, update_size);  // R
    Eigen::MatrixXd state_to_measurement_subset(update_size, state.rows());  // H
    Eigen::MatrixXd kalman_gain_subset(state.rows(), update_size);          // K
    Eigen::VectorXd innovation_subset(update_size);  // z - Hx
    state_subset.setZero();
    measurement_subset.setZero();
    state_to_measurement_subset.setZero();
    measurement_covariance_subset.setZero();
    kalman_gain_subset.setZero();
    innovation_subset.setZero();
    for (size_t i = 0; i < update_size; ++i)
    {
      measurement_subset[i] = measurement.measurement_(indices[i]);
      state_subset[i] = state(indices[i]);
      for (size_t j = 0; j < update_size; ++j)
      {
        measurement_covariance_subset(i, j) = measurement.covariance_(indices[i], indices[j]);
      }

      if (measurement_covariance_subset(i, i) < 0)
      {
        FB_DEBUG(
          "WARNING: Negative covariance for index " <<
            i << " of measurement (value is" <<
            measurement_covariance_subset(i, i) <<
            "). Using absolute value...\n");

        measurement_covariance_subset(i, i) =
          ::fabs(measurement_covariance_subset(i, i));
      }

      if (measurement_covariance_subset(i, i) < 1e-9)
      {
        FB_DEBUG(
          "WARNING: measurement had very small error covariance for index " <<
            update_indices[i] <<
            ". Adding some noise to maintain filter stability.\n");

        measurement_covariance_subset(i, i) = 1e-9;
      }
    }

    for (size_t i = 0; i < indices.size(); ++i) {
      state_to_measurement_subset(i, indices[i]) = 1;
    }

    Eigen::MatrixXd pht =
      state_error_covariance * state_to_measurement_subset.transpose();
    Eigen::MatrixXd hphr_inverse =
      (state_to_measurement_subset * pht + measurement_covariance_subset)
      .inverse();
    kalman_gain_subset.noalias() = pht * hphr_inverse;

    innovation_subset = (measurement_subset - state_subset);

    // Wrap angles in the innovation
    for (size_t i = 0; i < indices.size(); ++i) {
      if (indices[i] == StateMemberRoll ||
        indices[i] == StateMemberPitch ||
        indices[i] == StateMemberYaw)
      {
        generate_pseudomeasurement = use_pseudomeasurements_;
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
      // _model.set_state_covariance(state_error_covariance);
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
  _model.set_lie_state(state);
  auto& current_lie_state = _model.get_lie_state();
  auto R = current_lie_state.R()();
  _model.set_lie_covariance(state_error_covariance, R);
  Eigen::VectorXd z;
  Eigen::MatrixXd z_;
  Eigen::MatrixXd H;
  Eigen::MatrixXd V;
  Eigen::MatrixXd Sinv;
  Eigen::MatrixXd K;

  // Determine the typeof Measurement Model to be used
  // For doing the update properly it is mandatory to send all vector components in order to follow Lie Algebra
  // If you have a depth sensor, you can create as measurement vector like [x_state, y_state, z_measurement]
  // Then you have to set sigma_x and sigma_y really really high (you can do this from your measurement model)

  // UPDATE LINEAR TWIST VELOCITIES
  if (generate_pseudomeasurement || measurement.update_vector_[6] || measurement.update_vector_[7] || measurement.update_vector_[8])
  {
    z = Eigen::VectorXd(3);
    Eigen::VectorXd z_cov = Eigen::VectorXd(3);
    if (generate_pseudomeasurement)
    {
      z[0] = state(StateMemberVx);
      z[1] = 0;
      z[2] = 0;
      z_cov[0] = 1e9;
      z_cov[1] = this->pseudomeasurement_cov_lat_;
      z_cov[2] = this->pseudomeasurement_cov_alt_;
    }
    else
    {
      z[0] = (measurement.update_vector_[6] ? measurement.measurement_(6) : state(StateMemberVx));
      z[1] = (measurement.update_vector_[7] ? measurement.measurement_(7) : state(StateMemberVy));
      z[2] = (measurement.update_vector_[8] ? measurement.measurement_(8) : state(StateMemberVz));
      z_cov[0] = (measurement.update_vector_[6] ? measurement.covariance_.coeff(6, 6) + 1e-9 : 1e9);
      z_cov[1] = (measurement.update_vector_[7] ? measurement.covariance_.coeff(7, 7) + 1e-9 : 1e9);
      z_cov[2] = (measurement.update_vector_[8] ? measurement.covariance_.coeff(8, 8) + 1e-9 : 1e9);
    }

    Eigen::MatrixXd M = z_cov.asDiagonal();
    twistSensor.setNoise(M);
    z_  = twistSensor.processZ(z, current_lie_state);

    H = twistSensor.makeHError(current_lie_state, error_);
    V = twistSensor.calcV(z_, current_lie_state);
    Sinv = twistSensor.calcSInverse(current_lie_state);

    Eigen::MatrixXd cov = current_lie_state.cov();
    // Compute K + dX
    K = cov * (H.transpose() * Sinv);

    InEKF::SE3<2,6>::TangentVector K_V = K*V;

    // Apply to states
    if (error_ == InEKF::ERROR::RIGHT)
    {
      current_lie_state = InEKF::SE3<2,6>::exp(K_V) * current_lie_state;
    }
    else
    {
      current_lie_state = current_lie_state * InEKF::SE3<2,6>::exp(K_V);
    }
    current_lie_state.setCov(cov - K * (H * cov));

    Eigen::Vector3d updated_local_vel;
    updated_local_vel = R.inverse() * current_lie_state[0];

    // Update Local Velocity
    state(StateMemberVx) = updated_local_vel[0];
    state(StateMemberVy) = updated_local_vel[1];
    state(StateMemberVz) = updated_local_vel[2];

    // Remap state covariance
    // rotation, position, velocity, angular velocity, acceleration
    Eigen::Matrix<double, 15, 15> lie_covariance = current_lie_state.cov();

    state_error_covariance.block(3, 3, 3, 3) = lie_covariance.block(0, 0, 3, 3);
    state_error_covariance.block(6, 6, 3, 3) = R.transpose() * lie_covariance.block(3, 3, 3, 3) * R;
    state_error_covariance.block(0, 0, 3, 3) = lie_covariance.block(6, 6, 3, 3);
  }

  // UPDATE POSE
  if (measurement.update_vector_[0] || measurement.update_vector_[1] || measurement.update_vector_[2])
  {
    z = Eigen::VectorXd(3);
    z[0] = (measurement.update_vector_[0] ? measurement.measurement_(0) : state(StateMemberX));
    z[1] = (measurement.update_vector_[1] ? measurement.measurement_(1) : state(StateMemberY));
    z[2] = (measurement.update_vector_[2] ? measurement.measurement_(2) : state(StateMemberZ));

    Eigen::VectorXd z_cov = Eigen::VectorXd(3);
    // Measurement Matrix Inverse (LEFT ERROR)
    z_cov[0] = (measurement.update_vector_[0] ? measurement.covariance_.coeff(0, 0) + 1e-9 : 1e9);
    z_cov[1] = (measurement.update_vector_[1] ? measurement.covariance_.coeff(1, 1) + 1e-9 : 1e9);
    z_cov[2] = (measurement.update_vector_[2] ? measurement.covariance_.coeff(2, 2) + 1e-9 : 1e9);

    Eigen::MatrixXd M = z_cov.asDiagonal();
    poseSensor.setNoise(M);
    z_ = poseSensor.processZ(z, current_lie_state);
    H = poseSensor.makeHError(current_lie_state, error_);

    V = poseSensor.calcV(z_, current_lie_state);
    Sinv = poseSensor.calcSInverse(current_lie_state);

    // Compute K + dX
    K = current_lie_state.cov() * (H.transpose() * Sinv);
    auto cov = current_lie_state.cov();

    InEKF::SE3<2,6>::TangentVector K_V = K*V;
    // Apply to states
    if (error_ == InEKF::ERROR::RIGHT)
    {
      current_lie_state = InEKF::SE3<2,6>::exp(K_V) * current_lie_state;
    }
    else
    {
      current_lie_state = current_lie_state * InEKF::SE3<2,6>::exp(K_V);
    }

    current_lie_state.setCov(cov - K * (H * cov));

    // Update rpp_localization state
    // New state predicted, now map into rpp_localization state
    Eigen::Vector3d predicted_pos = current_lie_state[1];
    // Now Compute new state
    // Position
    state(StateMemberX) = predicted_pos[0];
    state(StateMemberY) = predicted_pos[1];
    state(StateMemberZ) = predicted_pos[2];

    // Remap state covariance
    // rotation, position, velocity, angular velocity, acceleration
    Eigen::Matrix<double, 15, 15> lie_covariance = current_lie_state.cov();

    state_error_covariance.block(3, 3, 3, 3) = lie_covariance.block(0, 0, 3, 3);
    state_error_covariance.block(6, 6, 3, 3) = R.transpose() * lie_covariance.block(3, 3, 3, 3) * R;
    state_error_covariance.block(0, 0, 3, 3) = lie_covariance.block(6, 6, 3, 3);
  }
}

template<class T>
void InEkf<T>::predict(
  const rclcpp::Time & reference_time,
  const rclcpp::Duration & delta)
{
  const double delta_sec = filter_utilities::toSec(delta);
  // very interesting, without ModelBase it does not work...TBD!!!!
  _model.ModelBase::step(reference_time, delta_sec);
  FB_DEBUG("PREDICTED MEASUREMENTS ARE :\n" << _model.get_state() << "\n");
}  // namespace rpp_localization


template<class T>
bool InEkf<T>::get_debug() {return _debug;}


template<class T>
void InEkf<T>::set_debug(const bool debug, std::ostream * out_stream)
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

// TODO
template class rpp_localization::InEkf<rpp_localization::InEKF::InertialProcess>;