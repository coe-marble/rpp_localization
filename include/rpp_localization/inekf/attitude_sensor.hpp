#ifndef ATTITUDE_SENSOR_
#define ATTITUDE_SENSOR_

#include <Eigen/Core>
#include "rpp_localization/inekf/se3.hpp"
#include "rpp_localization/inekf/so3.hpp"
#include "rpp_localization/inekf/measurement_model.hpp"

namespace rpp_localization::InEKF {

/**
 * @brief Attitude sensor measurement model for use with inertial process model. Uses 
    pseudo-measurements to fit into a left invariant measurement model.
 * 
 */
class AttitudeSensor : public MeasureModel<SO3<>> {
    
    public:
        using typename MeasureModel<SO3<>>::MatrixS;
        using typename MeasureModel<SO3<>>::MatrixH;
        using typename MeasureModel<SO3<>>::VectorV;
        using typename MeasureModel<SO3<>>::VectorB;

        /**
         * @brief Construct a new Depth Sensor object
         * 
         * @param std The standard deviation of the measurement.
         */
        AttitudeSensor(double std=1);

        /**
         * @brief Destroy the Depth Sensor object
         * 
         */
        ~AttitudeSensor(){ }

        /**
         * @brief Overriden from the base class. Inserts psuedo measurements for the x and y value to fit the invariant measurement.
         * 
         * @param z Measurement
         * @param state Current state estimate.
         * @return Processed measurement.
         */
        VectorB processZ(const Eigen::VectorXd& z, const SE3<2,6>& state) override;

        /**
         * @brief Overriden from base class. Calculate inverse of measurement noise S, using the Woodbury Matrix Identity
         * 
         * @param state Current state estimate.
         * @return Inverse of measurement noise. 
         */
        MatrixS calcSInverse(const SE3<2,6>& state) override;

        /**
         * @brief Set the measurement noise
         * 
         * @param std The standard deviation of the measurement.
         */
        void setNoise(double std);
};

}

#endif // ATTITUDE_SENSOR_