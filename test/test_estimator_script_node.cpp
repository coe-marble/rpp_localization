#include <gtest/gtest.h>
#include <rpp_localization/ros/estimator_script_node.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rpp_ros_interfaces/msg/numeric_interface.hpp>
#include <rpp_ros2/srv/reload_component.hpp>
#include <rcl/time.h>

#include <chrono>
#include <cstdint>
#include <functional>
#include <future>
#include <mutex>
#include <thread>

namespace
{
using namespace std::chrono_literals;
namespace c = rpp_interfaces::contract;

class EstimatorNodeTest : public ::testing::Test
{
  void SetUp() override {rclcpp::init(0, nullptr);}
  void TearDown() override {rclcpp::shutdown();}
};

TEST_F(EstimatorNodeTest, ReplayCatchesUpAndPublishesTheAdoptedStateAfterSimulationTimeStops)
{
  for (const std::string configuration : {"ekf", "ukf"}) {
    SCOPED_TRACE(configuration);
    rclcpp::NodeOptions options;
    options.parameter_overrides({
      {"rpp_workspace", std::string(RPP_LOCALIZATION_SOURCE_DIR)},
      {"script_name", std::string("localization_generic")}, {"configuration", configuration},
      {"use_sim_time", true}, {"replay_execution", std::string("ros")},
      {"predict_to_current_time", false}, {"expose_navigation_topics", false},
      {"initial_covariance", std::vector<double>(15, 0.01)},
      {"frequency", 200.0}, {"publish_frequency", 100.0}});
    auto estimator = std::make_shared<rpp_localization::EstimatorScriptNode>(options);
    auto peer = std::make_shared<rclcpp::Node>("replay_clock_test_peer");
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(estimator); executor.add_node(peer);
    using Estimate = rpp_ros_interfaces::msg::EstimatorEstimate;
    std::optional<Estimate> estimate;
    auto subscription = peer->create_subscription<Estimate>("/estimator_script/estimate", 10,
      [&](Estimate::ConstSharedPtr value) {estimate = *value;});
    ASSERT_TRUE(estimator->get_clock()->ros_time_is_active());
    std::int64_t clock = 2'000'000'000;
    const auto set_time = [&] {
      std::lock_guard<std::mutex> lock(estimator->get_clock()->get_clock_mutex());
      EXPECT_EQ(rcl_set_ros_time_override(estimator->get_clock()->get_clock_handle(), clock), RCL_RET_OK);
    };
    const auto spin_until = [&](const std::function<bool()>& ready, bool advance_clock) {
      const auto end = std::chrono::steady_clock::now() + 10s;
      while (!ready() && std::chrono::steady_clock::now() < end) {
        if (advance_clock) {clock += 10'000'000; set_time();}
        executor.spin_some(); std::this_thread::sleep_for(5ms);
      }
      return ready();
    };
    set_time();
    const std::vector<double> empty, q(225, 0.0);
    ASSERT_EQ(estimator->submit_prediction(empty, q, 1'000'000'000, 1'000'000'000, 0), c::CODE_OK);
    ASSERT_TRUE(spin_until([&] {return estimate && estimate->stamp.sec == 1;}, true));
    ASSERT_EQ(estimator->submit_prediction(empty, q, 2'000'000'000, 1'000'000'000, 0), c::CODE_OK);
    ASSERT_TRUE(spin_until([&] {return estimate && estimate->stamp.sec == 2;}, true));
    ASSERT_EQ(estimate->state.size(), 15u); EXPECT_NEAR(estimate->state[0], 0.0, 1e-8);
    std::vector<double> measurement(15, 0.0), r(225, 0.0);
    measurement[0] = 1.0;
    for (std::size_t axis = 0; axis < 15; ++axis) {r[axis * 15 + axis] = 0.1;}
    const auto frozen_clock = estimator->now().nanoseconds();
    ASSERT_EQ(estimator->submit_measurement(0, measurement, r, 1'000'000'000), c::CODE_OK);
    ASSERT_TRUE(spin_until([&] {return estimate && estimate->layout_revision == estimator->layout_revision() &&
      estimate->state[0] > 0.01;}, false));
    EXPECT_EQ(estimator->now().nanoseconds(), frozen_clock);
    EXPECT_EQ(estimate->stamp.sec, 2); EXPECT_EQ(estimate->stamp.nanosec, 0u);
    const auto revision = estimator->layout_revision();
    auto reload = peer->create_client<rpp_ros2::srv::ReloadComponent>("/estimator_script/estimator/reload");
    ASSERT_TRUE(spin_until([&] {return reload->service_is_ready();}, false));
    auto request = std::make_shared<rpp_ros2::srv::ReloadComponent::Request>();
    request->name = configuration == "ekf" ? "DefaultEkfEstimator" : "DefaultUkfEstimator";
    auto response = reload->async_send_request(request);
    ASSERT_TRUE(spin_until([&] {return response.wait_for(0s) == std::future_status::ready;}, false));
    const auto result = response.get();
    ASSERT_TRUE(result->success) << result->message;
    ASSERT_TRUE(spin_until([&] {return estimator->layout_revision() > revision;}, false));
    estimate.reset();
    ASSERT_EQ(estimator->submit_prediction(empty, q, 2'000'000'000, 1'000'000'000, 0), c::CODE_OK);
    ASSERT_EQ(estimator->submit_measurement(0, measurement, r, 1'000'000'000), c::CODE_OK);
    ASSERT_TRUE(spin_until([&] {return estimate && estimate->layout_revision == estimator->layout_revision() &&
      estimate->state[0] > 0.01;}, false));
    EXPECT_EQ(estimator->now().nanoseconds(), frozen_clock);
    executor.remove_node(estimator); executor.remove_node(peer);
  }
}

TEST_F(EstimatorNodeTest, WorkerPublishesNavigationAndControlProjectionAndAcceptsReset)
{
  rclcpp::NodeOptions options;
  options.parameter_overrides({
    {"rpp_workspace", std::string(RPP_LOCALIZATION_SOURCE_DIR)},
    {"script_name", std::string("navigation_estimation")},
    {"configuration", std::string("ekf")},
    {"expose_navigation_topics", true},
    {"navigation.publish_control_state", true},
    {"predict_to_current_time", false},
    {"frequency", 100.0}, {"publish_frequency", 100.0}});
  auto estimator = std::make_shared<rpp_localization::EstimatorScriptNode>(options);
  auto peer = std::make_shared<rclcpp::Node>("estimator_test_peer");
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(estimator); executor.add_node(peer);
  std::optional<nav_msgs::msg::Odometry> odometry;
  std::optional<rpp_ros_interfaces::msg::NumericSample> projection;
  std::optional<rpp_ros_interfaces::msg::NumericInterface> layout;
  auto estimate_subscription = peer->create_subscription<nav_msgs::msg::Odometry>("odometry/filtered", 10,
    [&](nav_msgs::msg::Odometry::ConstSharedPtr value) {odometry = *value;});
  auto state_subscription = peer->create_subscription<rpp_ros_interfaces::msg::NumericSample>(
    "/estimator_script/navigation/state", 10,
    [&](rpp_ros_interfaces::msg::NumericSample::ConstSharedPtr value) {projection = *value;});
  auto layout_subscription = peer->create_subscription<rpp_ros_interfaces::msg::NumericInterface>(
    "/estimator_script/navigation/layout", rclcpp::QoS(1).transient_local(),
    [&](rpp_ros_interfaces::msg::NumericInterface::ConstSharedPtr value) {layout = *value;});
  auto publisher = peer->create_publisher<nav_msgs::msg::Odometry>("odometry/input", 10);
  const auto spin_until = [&](const std::function<bool()>& ready, const std::function<void()>& send) {
    const auto end = std::chrono::steady_clock::now() + 10s;
    while (!ready() && std::chrono::steady_clock::now() < end) {
      send(); executor.spin_some(); std::this_thread::sleep_for(10ms);
    }
    return ready();
  };
  nav_msgs::msg::Odometry measurement;
  measurement.header.frame_id = "map"; measurement.child_frame_id = "base_link";
  measurement.pose.pose.position.x = 2.0; measurement.pose.pose.position.y = -1.0;
  measurement.pose.pose.orientation.w = 1.0;
  for (std::size_t axis = 0; axis < 6; ++axis) {
    measurement.pose.covariance[axis * 6 + axis] = 0.05;
    measurement.twist.covariance[axis * 6 + axis] = 0.1;
  }
  ASSERT_TRUE(spin_until([&] {return odometry && projection && layout;}, [&] {
    measurement.header.stamp = peer->now(); publisher->publish(measurement);
  }));
  EXPECT_EQ(odometry->header.frame_id, "map");
  EXPECT_EQ(odometry->child_frame_id, "base_link");
  EXPECT_NEAR(odometry->pose.pose.position.x, 2.0, 1e-8);
  EXPECT_NEAR(odometry->pose.pose.position.y, -1.0, 1e-8);
  ASSERT_EQ(projection->values.size(), 6u);
  EXPECT_EQ(projection->interface_name, "base");
  EXPECT_NEAR(projection->values[0], 2.0, 1e-8);
  ASSERT_EQ(layout->layout.fields.size(), 4u);
  EXPECT_EQ(layout->layout.fields[1].name, "base.orientation");
  EXPECT_EQ(layout->layout.fields[1].coordinate_kind, c::COORDINATE_CONTINUOUS_ANGLE);

  auto reset = peer->create_client<rpp_ros_interfaces::srv::ResetEstimate>("/estimator_script/reset");
  ASSERT_TRUE(spin_until([&] {return reset->service_is_ready();}, [] {}));
  auto request = std::make_shared<rpp_ros_interfaces::srv::ResetEstimate::Request>();
  request->stamp = peer->now(); request->activate = true;
  request->state.assign(15, 0.0); request->state[0] = 9.0; request->state[1] = 4.0;
  request->covariance.assign(225, 0.0);
  for (std::size_t axis = 0; axis < 15; ++axis) {request->covariance[axis * 15 + axis] = 0.1;}
  auto response = reset->async_send_request(request);
  ASSERT_TRUE(spin_until([&] {return response.wait_for(0s) == std::future_status::ready;}, [] {}));
  EXPECT_EQ(response.get()->status, c::CODE_OK);
  ASSERT_TRUE(spin_until([&] {return odometry && odometry->pose.pose.position.x == 9.0 &&
      projection && projection->values[0] == 9.0;}, [] {}));
  EXPECT_DOUBLE_EQ(odometry->pose.pose.position.y, 4.0);
  executor.remove_node(estimator); executor.remove_node(peer);
}

TEST_F(EstimatorNodeTest, NavigationWaitsForClockAndInitializesFromEarliestTimestamp)
{
  rclcpp::NodeOptions options;
  options.parameter_overrides({
    {"rpp_workspace", std::string(RPP_LOCALIZATION_SOURCE_DIR)},
    {"script_name", std::string("navigation_estimation")},
    {"configuration", std::string("ekf")},
    {"expose_navigation_topics", true}, {"use_sim_time", true},
    {"predict_to_current_time", false},
    {"frequency", 100.0}, {"publish_frequency", 100.0}});
  auto estimator = std::make_shared<rpp_localization::EstimatorScriptNode>(options);
  auto peer = std::make_shared<rclcpp::Node>("navigation_clock_test_peer");
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(estimator); executor.add_node(peer);
  const auto set_time = [&](std::int64_t time) {
    std::lock_guard<std::mutex> lock(estimator->get_clock()->get_clock_mutex());
    EXPECT_EQ(rcl_set_ros_time_override(estimator->get_clock()->get_clock_handle(), time), RCL_RET_OK);
  };
  ASSERT_TRUE(estimator->get_clock()->ros_time_is_active());
  set_time(1'000'000'000);

  std::optional<nav_msgs::msg::Odometry> odometry;
  auto subscription = peer->create_subscription<nav_msgs::msg::Odometry>("odometry/filtered", 10,
    [&](nav_msgs::msg::Odometry::ConstSharedPtr value) {odometry = *value;});
  auto publisher = peer->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("pose/input", 10);
  const auto spin_until = [&](const std::function<bool()>& ready, const std::function<void()>& send) {
    const auto end = std::chrono::steady_clock::now() + 10s;
    while (!ready() && std::chrono::steady_clock::now() < end) {
      send(); executor.spin_some(); std::this_thread::sleep_for(10ms);
    }
    return ready();
  };
  ASSERT_TRUE(spin_until([&] {
    return publisher->get_subscription_count() > 0 && subscription->get_publisher_count() > 0;
  }, [] {}));

  geometry_msgs::msg::PoseWithCovarianceStamped earlier;
  earlier.header.frame_id = "map"; earlier.header.stamp.sec = 1; earlier.header.stamp.nanosec = 10'000'000;
  earlier.pose.pose.position.x = 2.0; earlier.pose.pose.orientation.w = 1.0;
  for (std::size_t axis = 0; axis < 6; ++axis) {earlier.pose.covariance[axis * 6 + axis] = 0.1;}
  auto later = earlier;
  later.header.stamp.nanosec = 20'000'000; later.pose.pose.position.x = 9.0;
  for (std::size_t axis = 0; axis < 6; ++axis) {later.pose.covariance[axis * 6 + axis] = 1e12;}
  // Both messages arrive ahead of /clock, with the later timestamp arriving first.
  publisher->publish(later); publisher->publish(earlier);
  for (std::size_t iteration = 0; iteration < 10; ++iteration) {
    executor.spin_some(); std::this_thread::sleep_for(10ms);
  }
  EXPECT_FALSE(odometry.has_value());

  std::int64_t current_time = 1'020'000'000;
  ASSERT_TRUE(spin_until([&] {
    return odometry && odometry->header.stamp.sec == 1 && odometry->header.stamp.nanosec == 20'000'000u;
  }, [&] {
    current_time += 10'000'000; set_time(current_time);
  }));
  // The precise earlier pose must initialize the filter; the later pose has negligible weight.
  EXPECT_NEAR(odometry->pose.pose.position.x, 2.0, 1e-6);
  EXPECT_EQ(odometry->header.stamp.sec, 1);
  EXPECT_EQ(odometry->header.stamp.nanosec, 20'000'000u);
  executor.remove_node(estimator); executor.remove_node(peer);
}
}  // namespace
