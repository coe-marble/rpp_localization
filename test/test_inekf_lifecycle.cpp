#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include <rpp_cpp/component_runtime_builder.hpp>
#include <rpp_plugin_types/rpp_localization/InvariantFilter.hpp>
#include <rpp_localization/ros/invariant_localization_script.hpp>

namespace
{

using Filter = rpp_localization::InvariantFilter;
using InertialEstimate = Filter::InertialEstimate;

constexpr std::size_t k_error_size = 15;
constexpr double k_gravity = 9.80665;

struct InvariantScript
{
  InvariantScript()
  : data_manager(rpp::RPP_HOME, RPP_LOCALIZATION_SOURCE_DIR),
    builder(data_manager, rpp::ClockOptions(rpp::RPP_CLOCK_MOCK)),
    runtime(builder.build_script({
      rpp::LibraryScript{"rpp_localization", "invariant_localization"},
      std::optional<std::string>{"inekf"}, std::nullopt}))
  {
  }

  const rpp::SubcomponentContext& filter() const
  {
    return runtime.get_context().get_subcomponent_context("filter");
  }

  std::shared_ptr<Filter> instance() const
  {
    return filter().get_instance<Filter>();
  }

  rpp::RppDataManager data_manager;
  rpp::ComponentRuntimeBuilder builder;
  rpp::ScriptRuntime<> runtime;
};

template<typename Values>
void fill(Values values, const std::vector<double>& input)
{
  values.resize(static_cast<unsigned>(input.size()));
  for (std::size_t index = 0; index < input.size(); ++index)
  {
    values[index] = input[index];
  }
}

InertialEstimate make_estimate(const std::int64_t reference_time = 100'000'000)
{
  InertialEstimate estimate;
  auto state = estimate.state();
  fill(state.rotation(), {1, 0, 0, 0, 1, 0, 0, 0, 1});
  fill(state.velocity(), {0.5, 0.0, 0.0});
  fill(state.position(), {1.0, 2.0, 3.0});
  fill(state.gyroBias(), {0.0, 0.0, 0.0});
  fill(state.accelBias(), {0.0, 0.0, 0.0});
  std::vector<double> covariance(k_error_size * k_error_size, 0.0);
  for (std::size_t index = 0; index < k_error_size; ++index)
  {
    covariance[index * k_error_size + index] = 0.01;
  }
  fill(state.covariance(), covariance);
  state.rightInvariant() = true;
  fill(estimate.angularVelocity(), {0.0, 0.0, 0.0});
  estimate.referenceTimeNs() = reference_time;
  return estimate;
}

// A level IMU at rest, held over 10 ms.
Filter::InvariantPredictInput make_prediction(const std::int64_t reference_time)
{
  Filter::InvariantPredictInput input;
  fill(input.imu().angularVelocity(), {0.0, 0.0, 0.1});
  fill(input.imu().specificForce(), {0.0, 0.0, k_gravity});
  input.referenceTimeNs() = reference_time;
  input.deltaNs() = 10'000'000;
  return input;
}

rpp::HandoverState handover(InertialEstimate estimate)
{
  return rpp::HandoverState(InertialEstimate::Const(std::move(estimate)));
}

TEST(InekfLifecycle, ColdActivationWaitsForInitializationAndPredicts)
{
  InvariantScript script;
  ASSERT_TRUE(script.runtime.configure().success);

  ASSERT_TRUE(script.filter().activate().success);
  EXPECT_EQ(script.instance()->getEstimate().status().code(), 5);
  ASSERT_EQ(script.instance()->initialize(make_estimate()).code(), 0);

  const auto result = script.instance()->predict(make_prediction(110'000'000));
  ASSERT_EQ(result.status().code(), 0) << result.status().message();
  const auto state = result.estimate().state();
  // Gravity is compensated, so only the initial velocity moves the position.
  EXPECT_NEAR(state.position()[0], 1.005, 1e-9);
  EXPECT_NEAR(state.position()[2], 3.0, 1e-9);
  EXPECT_NEAR(state.velocity()[2], 0.0, 1e-9);
  EXPECT_NEAR(result.estimate().angularVelocity()[2], 0.1, 1e-12);
  EXPECT_EQ(result.estimate().referenceTimeNs(), 110'000'000);

  // Time must advance by exactly deltaNs.
  EXPECT_EQ(script.instance()->predict(make_prediction(130'000'000)).status().code(), 2);

  const auto stopped = script.filter().deactivate();
  ASSERT_TRUE(stopped.result.success);
  EXPECT_TRUE(stopped.state.has_value());
  EXPECT_EQ(script.instance()->getEstimate().status().code(), 5);
  ASSERT_TRUE(script.runtime.cleanup().success);
}

TEST(InekfLifecycle, HandsOverTheEstimateBetweenInstances)
{
  InvariantScript source;
  InvariantScript target;
  ASSERT_TRUE(source.runtime.configure().success);
  ASSERT_TRUE(target.runtime.configure().success);

  ASSERT_TRUE(source.filter().activate().success);
  ASSERT_EQ(source.instance()->initialize(make_estimate()).code(), 0);
  ASSERT_EQ(source.instance()->predict(make_prediction(110'000'000)).status().code(), 0);
  const auto expected = source.instance()->getEstimate();
  ASSERT_EQ(expected.status().code(), 0);

  auto stopped = source.filter().deactivate();
  ASSERT_TRUE(stopped.result.success);
  ASSERT_TRUE(stopped.state.has_value());
  ASSERT_TRUE(target.filter().activate(std::move(*stopped.state)).success);

  const auto restored = target.instance()->getEstimate();
  ASSERT_EQ(restored.status().code(), 0);
  EXPECT_EQ(restored.estimate().referenceTimeNs(), 110'000'000);
  const auto restored_state = restored.estimate().state();
  const auto expected_state = expected.estimate().state();
  for (std::size_t index = 0; index < 9; ++index)
  {
    EXPECT_EQ(restored_state.rotation()[index], expected_state.rotation()[index]);
  }
  for (std::size_t index = 0; index < 3; ++index)
  {
    EXPECT_EQ(restored_state.velocity()[index], expected_state.velocity()[index]);
    EXPECT_EQ(restored_state.position()[index], expected_state.position()[index]);
    EXPECT_EQ(
      restored.estimate().angularVelocity()[index],
      expected.estimate().angularVelocity()[index]);
  }
  for (std::size_t index = 0; index < k_error_size * k_error_size; ++index)
  {
    EXPECT_EQ(restored_state.covariance()[index], expected_state.covariance()[index]);
  }
  EXPECT_EQ(target.instance()->predict(make_prediction(120'000'000)).status().code(), 0);

  ASSERT_TRUE(target.filter().deactivate().result.success);
  ASSERT_TRUE(source.runtime.cleanup().success);
  ASSERT_TRUE(target.runtime.cleanup().success);
}

TEST(InekfLifecycle, RejectsAnInvalidHandoverEstimate)
{
  auto non_finite = make_estimate();
  non_finite.state().position()[0] = std::numeric_limits<double>::quiet_NaN();
  auto wrong_size = make_estimate();
  wrong_size.state().covariance().resize(k_error_size);
  auto left_invariant = make_estimate();
  left_invariant.state().rightInvariant() = false;
  std::vector<InertialEstimate> invalid;
  invalid.push_back(std::move(non_finite));
  invalid.push_back(std::move(wrong_size));
  invalid.push_back(std::move(left_invariant));

  for (auto& estimate : invalid)
  {
    InvariantScript script;
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

TEST(InekfLifecycle, ScriptActivatesTheFilterByDefault)
{
  InvariantScript script;
  ASSERT_TRUE(script.runtime.configure().success);
  rpp_localization::InvariantLocalizationScript invariant(script.runtime.get_context());
  invariant.initialize();
  EXPECT_TRUE(invariant.is_active());
  ASSERT_TRUE(script.filter().deactivate().result.success);
  EXPECT_FALSE(invariant.is_active());
  ASSERT_TRUE(script.runtime.cleanup().success);
}

}  // namespace
