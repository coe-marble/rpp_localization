#ifndef CONSTANT_ACC_MODEL_
#define CONSTANT_ACC_MODEL_

#include "rclcpp/time.hpp"
#include "rpp_localization/core/measurement.hpp"
#include "rpp_localization/core/model_base.hpp"
#include "rpp_localization/models/nav_model_base.hpp"
#include "rpp_localization/core/filter_utilities.hpp"
#include "rpp_localization/core/filter_common.hpp"

namespace rpp_localization
{

class ConstantAccelerationModel : public NavModelBase
{
public:
  ConstantAccelerationModel(int state_dim);

  ~ConstantAccelerationModel();

  void init(std::shared_ptr<rclcpp::Node> node) override;

  /// @brief Accepts current state and state_covariance and forwards them in time
  /// @param state 
  /// @param reference_time 
  /// @param dT 
  /// @param state_covariance 
  void step(Eigen::VectorXd& state, Eigen::MatrixXd& state_covariance, const rclcpp::Time & reference_time, const double dT) override;

private:

  void load_params();
  Eigen::MatrixXd _transfer_function;
  Eigen::MatrixXd _transfer_function_jacobian;

};

}  // namespace rpp_localization

#endif  // CONSTANT_ACC_MODEL_
