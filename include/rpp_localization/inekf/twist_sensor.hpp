#ifndef TWIST_SENSOR_
#define TWIST_SENSOR_

#include <Eigen/Core>
#include "rpp_localization/inekf/se3.hpp"
#include "rpp_localization/inekf/so3.hpp"
#include "rpp_localization/inekf/measurement_model.hpp"

namespace rpp_localization::InEKF {

/**
 * @brief Twist linear velocity sensor measurement model for use with inertial process model.
 * 
 */
class TwistSensor : public MeasureModel<SE3<2,6>> {
    
    public:
        using typename MeasureModel<SE3<2,6>>::MatrixS;
        using typename MeasureModel<SE3<2,6>>::MatrixH;
        using typename MeasureModel<SE3<2,6>>::VectorV;
        using typename MeasureModel<SE3<2,6>>::VectorB;

        /**
         * @brief Construct a new TwistSensor object. Assumes no rotation or translation between this and IMU frame.
         * 
         */
        TwistSensor();

        /**
         * @brief Construct a new TwistSensor object with offset from IMU frame.
         * 
         * @param dvlR 3x3 Rotation matrix encoding rotation from DVL to IMU frame.
         * @param dvlT 3x1 Vector of translation from IMU to DVL in IMU frame.
         */
        TwistSensor(Eigen::Matrix3d dvlR, Eigen::Vector3d dvlT);

        /**
         * @brief Construct a new TwistSensor object with offset from IMU frame.
         * 
         * @param dvlR SO3 object encoding rotationf rom DVL to IMU frame.
         * @param dvlT 3x1 Vector of translation from IMU to DVL in IMU frame.
         */
        TwistSensor(SO3<> dvlR, Eigen::Vector3d dvlT);

        /**
         * @brief Construct a new TwistSensor object with offset from IMU frame.
         * 
         * @param dvlH SE3 object encoding transformation from DVL to IMU frame.
         */
        TwistSensor(SE3<> dvlH);

        /**
         * @brief Destroy the TwistSensor object
         * 
         */
        ~TwistSensor(){ }

        /**
         * @brief Set the noise covariances.
         * 
         * @param std_dvl Standard deviation of DVL measurement.
         * @param std_imu Standard deviation of gyropscope measurement (needed b/c we transform frames).
         */
        void setNoise(double std_dvl, double std_imu);

        void setNoise(Eigen::MatrixXd std_cov);
        /**
         * @brief Overriden from base class. Takes in a 6 vector with DVL measurement as first 3 elements and IMU as last three 
         and converts DVL to IMU, then makes it the right size and passes it on.
         * 
         * @param z DVL/IMU measurement.
         * @param state Current state estimate.
         * @return Processed measurement.
         */
        VectorB processZ(const Eigen::VectorXd& z, const SE3<2,6>& state) override;

    private:
        Eigen::Matrix3d dvlT_;
        SO3<> dvlR_;
};

}

#endif // TWIST_SENSOR_