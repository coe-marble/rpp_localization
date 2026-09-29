#ifndef RPP_LOCALIZATION__PYTHON_MODEL_HPP_
#define RPP_LOCALIZATION__PYTHON_MODEL_HPP_

#include <ostream>
#include <vector>
#include <stdexcept>

#include "Eigen/Dense"
#include "rclcpp/time.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rpp_localization/core/measurement.hpp"
#include "rpp_localization/core/model_base.hpp"
#include "pybind11/pybind11.h"
#include <pybind11/embed.h>

namespace rpp_localization
{

class PythonModel : public ModelBase
{
    public:
        PythonModel(int state_dim);
        ~PythonModel();

        void init(rclcpp::Node& node);
        void step(const rclcpp::Time& reference_time, double dT);
        void predict(
          StateVector& state,
          CovarianceMatrix& state_covariance,
          TimestampNs reference_time,
          DurationNs delta) override;

    private:
        void load_params(rclcpp::Node& node);

        void import_py_module();

        bool _initialized;
        std::string _param_namespace;
        std::string _py_package_name;
        std::string _py_module_name;
        std::string _py_class_name;
        pybind11::scoped_interpreter _interpreter_guard;
};

}  // namespace rpp_localization

#endif  // RPP_LOCALIZATION__PYTHON_MODEL_HPP_
