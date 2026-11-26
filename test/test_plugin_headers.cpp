#include <gtest/gtest.h>

#include <type_traits>

#include "ekf15_plugin.hpp"
#include "ukf15_plugin.hpp"
#include "constant_acceleration_model15.hpp"
#include "navigation_model.hpp"
#include "ekf_estimator.hpp"
#include "ukf_estimator.hpp"

TEST(LocalizationPluginHeaders, ExposeConcreteRppPluginEntryPoints)
{
  static_assert(std::is_base_of_v<rpp_localization::LocalizationFilter15, Ekf15>);
  static_assert(std::is_base_of_v<rpp_localization::LocalizationFilter15, Ukf15>);
  static_assert(std::is_base_of_v<rpp_localization::NavModel15, ConstantAccelerationModel15>);
  static_assert(std::is_base_of_v<rpp_interfaces::Model, NavigationModel>);
  static_assert(std::is_base_of_v<rpp_interfaces::Estimator, ::Ekf>);
  static_assert(std::is_base_of_v<rpp_interfaces::Estimator, ::Ukf>);
  static_assert(!std::is_abstract_v<NavigationModel>);
  static_assert(!std::is_abstract_v<::Ekf>);
  static_assert(!std::is_abstract_v<::Ukf>);
  SUCCEED();
}
