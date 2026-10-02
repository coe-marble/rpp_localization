#ifndef RPP_LOCALIZATION__MODELS__CONSTANT_ACCELERATION_MODEL_HPP_
#define RPP_LOCALIZATION__MODELS__CONSTANT_ACCELERATION_MODEL_HPP_

#include "rpp_localization/core/filter_common.hpp"
#include "rpp_localization/core/model_base.hpp"
#include "rpp_localization/models/navigation_model_base.hpp"
#include "rpp_localization/core/filter_utilities.hpp"

namespace rpp_localization
{

class ConstantAccelerationModel : public NavigationModelBase
{
public:
  ConstantAccelerationModel(int state_dim);

  ~ConstantAccelerationModel();

  // Initializes transfer-matrix state after plugin configuration.
  void initialize_runtime();

  /// @brief Accepts current state and state_covariance and forwards them in time
  /// @param state 
  /// @param reference_time 
  /// @param dT 
  /// @param state_covariance 
  void predict(
    StateVector& state,
    CovarianceMatrix& state_covariance,
    TimestampNs reference_time,
    DurationNs delta) override;

private:
  Eigen::MatrixXd _transfer_function;
  Eigen::MatrixXd _transfer_function_jacobian;

};

}  // namespace rpp_localization

#endif  // RPP_LOCALIZATION__MODELS__CONSTANT_ACCELERATION_MODEL_HPP_
