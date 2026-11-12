#include <memory>

#include <gtest/gtest.h>

#include "rpp_localization/filters/extended_kalman_filter.hpp"
#include "rpp_localization/filters/unscented_kalman_filter.hpp"
#include "rpp_localization/models/constant_acceleration_model.hpp"

namespace rpp_localization::test
{

std::unique_ptr<ModelBase> make_peer_model();
std::unique_ptr<FilterBase> make_peer_filter(ModelBase & model);
using WrapStateAngles = void (*)(Eigen::VectorXd &);
WrapStateAngles peer_wrap_function();

TEST(NumericsLinkage, LinksAcrossTranslationUnitsWithoutTheRosAdapter)
{
  auto peer_model = make_peer_model();
  auto peer_filter = make_peer_filter(*peer_model);

  ConstantAccelerationModel local_model(STATE_SIZE);
  local_model.initialize_runtime();
  Ekf ekf(local_model);
  Ukf ukf(local_model);

  EXPECT_EQ(peer_filter->get_model(), peer_model.get());
  EXPECT_EQ(ekf.get_model(), &local_model);
  EXPECT_EQ(ukf.get_model(), &local_model);
  EXPECT_EQ(peer_wrap_function(), &filter_utilities::wrap_state_angles);
}

}  // namespace rpp_localization::test
