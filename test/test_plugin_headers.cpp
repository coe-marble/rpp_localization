#include <gtest/gtest.h>

#include <type_traits>

#include "ekf15_plugin.hpp"
#include "ukf15_plugin.hpp"
#include "constant_acceleration_model15.hpp"
#include "vehicle_model3d_nav_model15.hpp"

TEST(LocalizationPluginHeaders, ExposeConcreteRppPluginEntryPoints)
{
  static_assert(std::is_base_of_v<rpp_localization::LocalizationFilter15, Ekf15>);
  static_assert(std::is_base_of_v<rpp_localization::LocalizationFilter15, Ukf15>);
  static_assert(std::is_base_of_v<rpp_localization::NavModel15, ConstantAccelerationModel15>);
  static_assert(std::is_base_of_v<rpp_localization::NavModel15, VehicleModel3DNavModel15>);
  SUCCEED();
}
