#include "rpp_localization/inekf/pose_sensor.hpp"

namespace rpp_localization::InEKF {

PoseSensor::PoseSensor(double std) {
    error_ = ERROR::LEFT;

    M_ = Eigen::Matrix3d::Zero();
    M_(2,2) = 1 / (std*std);

    H_ = Eigen::Matrix<double, 3, 15>::Zero();
    H_.block<3,3>(0,6) = Eigen::Matrix3d::Identity();
}

void PoseSensor::setNoise(double std){
    // Actually storing M.inverse() here
    M_ = Eigen::Matrix3d::Zero();
    M_(2,2) = 1 / (std*std);
}

void PoseSensor::setNoise(Eigen::MatrixXd std_cov){
    // Actually storing M.inverse() here
    M_ = Eigen::Matrix3d::Zero();
    M_ = std_cov;
}

void PoseSensor::setLocalNoise(double std, int i){
    M_(i, i) = std;
}

PoseSensor::VectorB PoseSensor::processZ(const Eigen::VectorXd& z, const SE3<2,6>& state) {
    Eigen::Matrix<double,5,1> z_full;
    z_full << z[0], z[1], z[2], 0, 1;
    return z_full;
}

PoseSensor::MatrixS PoseSensor::calcSInverse(const SE3<2,6>& state){
    // Calculate Sinv
    Eigen::Matrix3d R = state.R()();
    Eigen::Matrix3d Sig = (H_error_ * state.cov() * H_error_.transpose()).inverse();
    return Sig - Sig*( R.transpose()*M_*R + Sig ).inverse()*Sig;
}

}