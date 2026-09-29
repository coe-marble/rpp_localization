
#ifndef RPP_LOCALIZATION__INEKF_HPP_
#define RPP_LOCALIZATION__INEKF_HPP_

#include "rclcpp/time.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rpp_localization/core/filter_base.hpp"
#include "rpp_localization/core/measurement.hpp"
#include "rpp_localization/core/filter_utilities.hpp"

// ADDING InEKF LIBRARY
#include "rpp_localization/inekf/inertial_process.hpp"
#include "rpp_localization/inekf/twist_sensor.hpp"
#include "rpp_localization/inekf/pose_sensor.hpp"

#define INFO RCUTILS_LOG_INFO

namespace rpp_localization
{

template<class T> // model type
class InEkf : public FilterBase
{
  public:

    InEkf(int state_space_dim);
    ~InEkf();


    void init(std::shared_ptr<rclcpp::Node> node) override;

    void correct(const Measurement& measurement) override;

    void predict(
      const rclcpp::Time& reference_time,
      const rclcpp::Duration& delta) override;

    bool get_debug();
    void set_debug(const bool debug, std::ostream * out_stream);

  private:
    bool _debug;
    bool use_pseudomeasurements_;
    double pseudomeasurement_cov_lat_, pseudomeasurement_cov_alt_;
    std::ostream* _debug_stream;
    Eigen::MatrixXd _identity;
    T _model;

    InEKF::ERROR error_;
    // Sensor General Model
    // For now Assuming no rotation or translation between Input Control (IMU) and sensors
    InEKF::PoseSensor poseSensor;
    InEKF::TwistSensor twistSensor;
};

}  // namespace rpp_localization

#endif  // RPP_LOCALIZATION__EKF_HPP_