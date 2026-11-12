#include <memory>

#include "rpp_localization/filters/unscented_kalman_filter.hpp"
#include "rpp_localization/models/constant_acceleration_model.hpp"

namespace rpp_localization::test
{

std::unique_ptr<ModelBase> make_peer_model()
{
  auto model = std::make_unique<ConstantAccelerationModel>(STATE_SIZE);
  model->initialize_runtime();
  return model;
}

std::unique_ptr<FilterBase> make_peer_filter(ModelBase & model)
{
  return std::make_unique<Ukf>(model);
}

using WrapStateAngles = void (*)(Eigen::VectorXd &);

WrapStateAngles peer_wrap_function()
{
  return &filter_utilities::wrap_state_angles;
}

}  // namespace rpp_localization::test
