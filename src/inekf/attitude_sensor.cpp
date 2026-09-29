#include "rpp_localization/inekf/attitude_sensor.hpp"

namespace rpp_localization::InEKF {

AttitudeSensor::AttitudeSensor(double std) {
    error_ = ERROR::LEFT;

    M_ = Eigen::Matrix3d::Zero();
    H_ = Eigen::Matrix<double, 3, 15>::Zero();

    H_ = Eigen::Matrix<double, 3, 15>::Zero();
    H_.block<3,3>(0,0) = Eigen::Matrix3d::Identity();
}

void AttitudeSensor::setNoise(double std){
    // Actually storing M.inverse() here
    M_ = Eigen::Matrix3d::Zero();
    M_(2,2) = 1 / (std*std);
}


AttitudeSensor::VectorB AttitudeSensor::processZ(const Eigen::Matrix3d& z, const SE3<2,6>& state) {
    Eigen::Matrix<double,5,1> z_full;
    Eigen::Vector3d p = state[1];
    z_full << z[0], z[1], z[2], 0, 1;
    return z_full;
}

}