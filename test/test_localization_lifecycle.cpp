#include <cstddef>
#include <cstdint>
#include <initializer_list>
#include <limits>
#include <memory>
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

using Estimate15 = rpp_localization::LocalizationFilter15::Estimate15;

struct FilterScript
{
  explicit FilterScript(const std::string& configuration)
  : data_manager(rpp::RPP_HOME, RPP_LOCALIZATION_SOURCE_DIR),
    builder(data_manager, rpp::ClockOptions(rpp::RPP_CLOCK_MOCK)),
    runtime(builder.build_script({
      rpp::LibraryScript{"rpp_localization", "localization"},
      std::optional<std::string>{configuration}, std::nullopt}))
  {
  }

  const rpp::SubcomponentContext& filter() const
  {
    return runtime.get_context().get_subcomponent_context("filter");
  }

  std::shared_ptr<rpp_localization::LocalizationFilter15> instance() const
  {
    return filter().get_instance<rpp_localization::LocalizationFilter15>();
  }

  rpp::RppDataManager data_manager;
  rpp::ComponentRuntimeBuilder builder;
  rpp::ScriptRuntime<> runtime;
};

Estimate15 make_moving_estimate()
{
  auto estimate = make_estimate();
  auto state = estimate.state().values();
  auto covariance = estimate.covariance().values();
  for (std::size_t row = 0; row < k_state_size; ++row)
  {
    state[row] = 0.1 * static_cast<double>(row + 1);
    covariance[row * k_state_size + row] = 0.5;
  }
  estimate.referenceTimeNs() = 100'000'000;
  return estimate;
}

rpp_localization::LocalizationFilter15::LocalizationPredictInput15 make_prediction_at(
  const std::int64_t reference_time)
{
  rpp_localization::LocalizationFilter15::LocalizationPredictInput15 prediction;
  prediction.referenceTimeNs() = reference_time;
  prediction.deltaNs() = 50'000'000;
  prediction.control().present() = false;
  return prediction;
}

rpp::HandoverState handover(Estimate15 estimate)
{
  return rpp::HandoverState(Estimate15::Const(std::move(estimate)));
}

TEST(LocalizationLifecycle, HandsOverTheEstimateBetweenEkfAndUkf)
{
  const std::vector<std::pair<std::string, std::string>> switches{{"ekf", "ukf"}, {"ukf", "ekf"}};
  for (const auto& [source_name, target_name] : switches)
  {
    SCOPED_TRACE(source_name + " to " + target_name);
    FilterScript source(source_name);
    FilterScript target(target_name);
    ASSERT_TRUE(source.runtime.configure().success);
    ASSERT_TRUE(target.runtime.configure().success);

    ASSERT_TRUE(source.filter().activate().success);
    ASSERT_EQ(source.instance()->initialize(make_moving_estimate()).code(), 0);
    ASSERT_EQ(source.instance()->predict(make_prediction_at(150'000'000)).status().code(), 0);
    const auto expected = source.instance()->getEstimate();
    ASSERT_EQ(expected.status().code(), 0);

    auto stopped = source.filter().deactivate();
    ASSERT_TRUE(stopped.result.success);
    ASSERT_TRUE(stopped.state.has_value());
    ASSERT_TRUE(target.filter().activate(std::move(*stopped.state)).success);

    const auto restored = target.instance()->getEstimate();
    ASSERT_EQ(restored.status().code(), 0);
    EXPECT_EQ(restored.estimate().referenceTimeNs(), 150'000'000);
    EXPECT_EQ(restored.estimate().referenceTimeNs(), expected.estimate().referenceTimeNs());
    for (std::size_t index = 0; index < k_state_size; ++index)
    {
      EXPECT_EQ(
        restored.estimate().state().values()[index],
        expected.estimate().state().values()[index]);
    }
    for (std::size_t index = 0; index < k_state_size * k_state_size; ++index)
    {
      EXPECT_EQ(
        restored.estimate().covariance().values()[index],
        expected.estimate().covariance().values()[index]);
    }
    EXPECT_EQ(target.instance()->predict(make_prediction_at(200'000'000)).status().code(), 0);

    ASSERT_TRUE(target.filter().deactivate().result.success);
    ASSERT_TRUE(source.runtime.cleanup().success);
    ASSERT_TRUE(target.runtime.cleanup().success);
  }
}

TEST(LocalizationLifecycle, ColdActivationWaitsForInitialization)
{
  FilterScript script("ekf");
  ASSERT_TRUE(script.runtime.configure().success);

  ASSERT_TRUE(script.filter().activate().success);
  EXPECT_EQ(script.instance()->getEstimate().status().code(), 5);
  const auto stopped = script.filter().deactivate();
  ASSERT_TRUE(stopped.result.success);
  EXPECT_FALSE(stopped.state.has_value());

  ASSERT_TRUE(script.filter().activate().success);
  EXPECT_EQ(script.instance()->getEstimate().status().code(), 5);
  EXPECT_EQ(script.instance()->initialize(make_moving_estimate()).code(), 0);
  EXPECT_EQ(script.instance()->getEstimate().status().code(), 0);
  ASSERT_TRUE(script.filter().deactivate().result.success);
  ASSERT_TRUE(script.runtime.cleanup().success);
}

TEST(LocalizationLifecycle, RejectsAnInvalidHandoverEstimate)
{
  auto non_finite = make_moving_estimate();
  non_finite.state().values()[0] = std::numeric_limits<double>::quiet_NaN();
  auto wrong_size = make_moving_estimate();
  wrong_size.covariance().values().resize(k_state_size);
  std::vector<Estimate15> invalid;
  invalid.push_back(std::move(non_finite));
  invalid.push_back(std::move(wrong_size));

  for (auto& estimate : invalid)
  {
    FilterScript script("ekf");
    ASSERT_TRUE(script.runtime.configure().success);

    const auto result = script.filter().activate(handover(std::move(estimate)));
    EXPECT_FALSE(result.success);
    EXPECT_NE(result.message.find("handover estimate"), std::string::npos) << result.message;
    EXPECT_FALSE(script.filter().is_active());

    // A failed activation leaves the slot in ERROR until it is deactivated.
    const auto recovered = script.filter().deactivate();
    ASSERT_TRUE(recovered.result.success) << recovered.result.message;
    EXPECT_FALSE(recovered.state.has_value());
    ASSERT_TRUE(script.filter().activate().success);
    EXPECT_EQ(script.instance()->getEstimate().status().code(), 5);
    ASSERT_TRUE(script.filter().deactivate().result.success);
    ASSERT_TRUE(script.runtime.cleanup().success);
  }
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

    EXPECT_EQ(script.is_active(), activate_on_start);
    const auto & filter_context = runtime.get_context().get_subcomponent_context("filter");
    if (!activate_on_start)
    {
      ASSERT_TRUE(filter_context.activate().success);
      EXPECT_TRUE(script.is_active());
    }

    {
      // Obtain the active filter only when it is needed. LocalizationScript no
      // longer retains a component instance across slot lifecycle operations.
      const auto filter = runtime.get_context().get_active_component<
        rpp_localization::LocalizationFilter15>("filter");
      ASSERT_TRUE(static_cast<bool>(filter));
      EXPECT_EQ(filter->initialize(std::move(make_estimate())).code(), 0);
      EXPECT_EQ(filter->predict(std::move(make_prediction())).status().code(), 0);
    }
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
