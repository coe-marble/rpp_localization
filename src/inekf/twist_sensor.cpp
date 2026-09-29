#include "rpp_localization/inekf/twist_sensor.hpp"

namespace rpp_localization::InEKF {

TwistSensor::TwistSensor(SO3<> dvlR, Eigen::Vector3d dvlT)
    : dvlR_(dvlR), dvlT_(SO3<>::wedge(dvlT)){
    error_ = ERROR::RIGHT;

    M_ = Eigen::Matrix3d::Zero();
    H_ = Eigen::Matrix<double, 3, 15>::Zero();
    H_.block<3,3>(0,3) = Eigen::Matrix3d::Identity();
}

TwistSensor::TwistSensor(SE3<> dvlH)
    : TwistSensor(dvlH.R(), dvlH[0]) {}

TwistSensor::TwistSensor(Eigen::Matrix3d dvlR, Eigen::Vector3d dvlT)
    : TwistSensor(SO3<>(dvlR), dvlT) {}

TwistSensor::TwistSensor() 
    : TwistSensor(SO3<>(), Eigen::Vector3d::Zero()) {
    M_ = Eigen::Matrix3d::Zero();
    M_ = Eigen::Matrix3d::Identity() * 0.02;
    H_ = Eigen::Matrix<double, 3, 15>::Zero();
    H_.block<3,3>(0,3) = Eigen::Matrix3d::Identity();
}

void TwistSensor::setNoise(double std_dvl, double std_imu){
    M_ = Eigen::Matrix3d::Identity() * std_dvl*std_dvl;
}

void TwistSensor::setNoise(Eigen::MatrixXd std_cov)
{
    M_ = std_cov;
}

TwistSensor::VectorB TwistSensor::processZ(const Eigen::VectorXd& z, const SE3<2,6>& state){
    // Fill up Z
    Eigen::Matrix<double,5,1> z_full;
    z_full << z[0], z[1], z[2], -1, 0;

    return z_full;
}

}