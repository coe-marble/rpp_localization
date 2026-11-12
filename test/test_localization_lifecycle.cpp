#include <cstddef>
#include <initializer_list>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include <rpp_cpp/component_runtime_builder.hpp>
#include <rpp_plugin_types/rpp_localization/LocalizationFilter15.hpp>
#include <rpp_plugin_types/rpp_localization/NavModel15.hpp>
#include <rpp_localization/core/filter_common.hpp>
#include <rpp_localization/ros/localization_script.hpp>

namespace
{

constexpr std::size_t k_state_size = 15;

rpp_localization::LocalizationFilter15::Estimate15 make_estimate()
{
  rpp_localization::LocalizationFilter15::Estimate15 estimate;
  auto state = estimate.state().values();
  state.resize(k_state_size);
  auto covariance = estimate.covariance().values();
  covariance.resize(k_state_size * k_state_size);
  for (std::size_t row = 0; row < k_state_size; ++row)
  {
    state[row] = 0.0;
    for (std::size_t column = 0; column < k_state_size; ++column)
    {
      covariance[row * k_state_size + column] = row == column ? 1.0 : 0.0;
    }
  }
  estimate.referenceTimeNs() = 0;
  return estimate;
}

rpp_localization::LocalizationFilter15::LocalizationPredictInput15 make_prediction()
{
  rpp_localization::LocalizationFilter15::LocalizationPredictInput15 prediction;
  prediction.referenceTimeNs() = 1;
  prediction.deltaNs() = 1;
  prediction.control().present() = false;
  return prediction;
}

TEST(LocalizationLifecycle, ConfiguresActivatesAndDeactivatesTheFilterTree)
{
  rpp::RppDataManager data_manager(rpp::RPP_HOME, RPP_LOCALIZATION_SOURCE_DIR);
  rpp::ComponentRuntimeBuilder builder(
    data_manager, rpp::ClockOptions(rpp::RPP_CLOCK_MOCK));
  auto runtime = builder.build_script({
    rpp::LibraryScript{"rpp_localization", "localization"},
    std::optional<std::string>{"ekf"}, std::nullopt});

  ASSERT_TRUE(runtime.configure().success);
  const auto & script_context = runtime.get_context();
  const auto & filter_context = script_context.get_subcomponent_context("filter");
  EXPECT_FALSE(filter_context.is_active());

  ASSERT_TRUE(filter_context.activate().success);
  EXPECT_TRUE(filter_context.is_active());

  const auto filter = filter_context.get_instance<rpp_localization::LocalizationFilter15>();
  ASSERT_NE(filter, nullptr);
  EXPECT_EQ(filter->initialize(std::move(make_estimate())).code(), 0);
  EXPECT_EQ(filter->predict(std::move(make_prediction())).status().code(), 0);

  const auto stopped = filter_context.deactivate();
  ASSERT_TRUE(stopped.result.success);
  EXPECT_FALSE(filter_context.is_active());
  EXPECT_EQ(filter->getEstimate().status().code(), 5);

  ASSERT_TRUE(filter_context.activate().success);
  EXPECT_EQ(filter->initialize(std::move(make_estimate())).code(), 0);

  const auto stopped_again = filter_context.deactivate();
  ASSERT_TRUE(stopped_again.result.success);
  ASSERT_TRUE(runtime.cleanup().success);
}

TEST(LocalizationLifecycle, ScriptCanWaitForExternalActivationOrActivateByDefault)
{
  for (const bool activate_on_start : {false, true})
  {
    SCOPED_TRACE(activate_on_start);
    rpp::RppDataManager data_manager(rpp::RPP_HOME, RPP_LOCALIZATION_SOURCE_DIR);
    rpp::ComponentRuntimeBuilder builder(
      data_manager, rpp::ClockOptions(rpp::RPP_CLOCK_MOCK));
    auto runtime = builder.build_script({
      rpp::LibraryScript{"rpp_localization", "localization"},
      std::optional<std::string>{"ekf"}, std::nullopt});
    ASSERT_TRUE(runtime.configure().success);

    rpp_localization::LocalizationScript script(runtime.get_context());
    if (activate_on_start)
    {
      script.initialize();
    }
    else
    {
      script.initialize(false);
    }

    ASSERT_NE(script.filter(), nullptr);
    EXPECT_EQ(script.is_active(), activate_on_start);
    const auto & filter_context = runtime.get_context().get_subcomponent_context("filter");
    if (!activate_on_start)
    {
      ASSERT_TRUE(filter_context.activate().success);
      EXPECT_TRUE(script.is_active());
    }

    EXPECT_EQ(script.filter()->initialize(std::move(make_estimate())).code(), 0);
    EXPECT_EQ(script.filter()->predict(std::move(make_prediction())).status().code(), 0);
    ASSERT_TRUE(filter_context.deactivate().result.success);
    EXPECT_FALSE(script.is_active());
    ASSERT_TRUE(runtime.cleanup().success);
  }
}

TEST(LocalizationLifecycle, DisabledControlAxesPreserveMeasuredAcceleration)
{
  rpp::RppDataManager data_manager(rpp::RPP_HOME, RPP_LOCALIZATION_SOURCE_DIR);
  rpp::ComponentRuntimeBuilder builder(data_manager);
  builder.set_parameter_overrides({{"rpp_localization::ConstantAccelerationModel15", {
    {"use_control", true},
    {"control_config", rpp::params::ParameterConverter<std::vector<bool>>::to(
      std::vector<bool>(6, true))}
  }}});
  auto runtime = builder.build_component({
    std::string(RPP_LOCALIZATION_SOURCE_DIR) +
      "/.rppws/parts/rpp_localization__ekf15/11111111-1111-4111-8111-111111111111/"
      "subcomponents/11111111-1111-4111-8111-111111111112",
    "rpp_localization::ConstantAccelerationModel15"});
  ASSERT_TRUE(runtime.configure().success);
  ASSERT_TRUE(runtime.activate().success);
  auto model = runtime.get_instance<rpp_localization::NavModel15>();
  for (const bool present : {true, false}) {
    rpp_localization::NavModel15::LocalizationModelPredictInput15 input;
    auto initial = make_estimate();
    auto state = input.state().values();
    auto covariance = input.covariance().values();
    state.resize(k_state_size);
    covariance.resize(k_state_size * k_state_size);
    for (std::size_t index = 0; index < k_state_size; ++index) {
      state[index] = static_cast<double>(initial.state().values()[index]);
      for (std::size_t column = 0; column < k_state_size; ++column) {
        covariance[index * k_state_size + column] =
          static_cast<double>(initial.covariance().values()[index * k_state_size + column]);
      }
    }
    input.state().values()[rpp_localization::StateMemberAx] = 2.0;
    input.control().present() = present;
    input.control().values().resize(6);
    input.control().enabled().resize(6);
    for (std::size_t index = 0; index < 6; ++index) {
      input.control().values()[index] = 0.0;
      input.control().enabled()[index] = false;
    }
    input.referenceTimeNs() = 100'000'000;
    input.deltaNs() = 100'000'000;
    const auto result = model->predict(std::move(input));
    ASSERT_EQ(result.status().code(), 0);
    EXPECT_DOUBLE_EQ(result.state().values()[rpp_localization::StateMemberAx], 2.0);
  }
}

}  // namespace
