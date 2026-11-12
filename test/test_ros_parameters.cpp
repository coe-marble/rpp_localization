#include <limits>
#include <vector>

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include "rpp_localization/ros/ros_filter_utilities.hpp"

namespace rpp_localization
{
namespace
{

class RosParameters : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    rclcpp::init(0, nullptr);
  }

  static void TearDownTestSuite()
  {
    rclcpp::shutdown();
  }
};

TEST_F(RosParameters, MissingCovariancePreservesTheDefault)
{
  rclcpp::Node node("covariance_defaults");
  Eigen::MatrixXd covariance = Eigen::MatrixXd::Identity(STATE_SIZE, STATE_SIZE) * 1e-9;
  const Eigen::MatrixXd expected = covariance;
  ros_filter_utilities::load_covariance_parameter(node, "covariance", covariance);
  EXPECT_TRUE(covariance.isApprox(expected));
}

TEST_F(RosParameters, CovarianceAcceptsDiagonalAndRowMajorMatrices)
{
  rclcpp::Node node("covariance_layout");
  node.declare_parameter("diagonal", std::vector<double>(STATE_SIZE, 2.0));
  Eigen::MatrixXd covariance;
  ros_filter_utilities::load_covariance_parameter(node, "diagonal", covariance);
  EXPECT_TRUE(covariance.isApprox(Eigen::MatrixXd::Identity(STATE_SIZE, STATE_SIZE) * 2.0));

  std::vector<double> values(STATE_SIZE * STATE_SIZE, 0.0);
  values[1] = 0.25;
  node.declare_parameter("full", values);
  ros_filter_utilities::load_covariance_parameter(node, "full", covariance);
  EXPECT_DOUBLE_EQ(covariance(0, 1), 0.25);
  EXPECT_DOUBLE_EQ(covariance(1, 0), 0.0);
}

TEST_F(RosParameters, MalformedCovarianceIsRejectedWithoutChangingTheDestination)
{
  for (const std::size_t length : {0, 1, 14, 16, 224, 226}) {
    SCOPED_TRACE(length);
    rclcpp::Node node("covariance_length");
    node.declare_parameter("covariance", std::vector<double>(length, 1.0));
    Eigen::MatrixXd covariance = Eigen::MatrixXd::Identity(STATE_SIZE, STATE_SIZE);
    EXPECT_THROW(ros_filter_utilities::load_covariance_parameter(node, "covariance", covariance),
      std::invalid_argument);
    EXPECT_TRUE(covariance.isIdentity());
  }
  for (const double value : {-1.0, std::numeric_limits<double>::infinity()}) {
    rclcpp::Node node("covariance_values");
    node.declare_parameter("covariance", std::vector<double>(STATE_SIZE, value));
    Eigen::MatrixXd covariance = Eigen::MatrixXd::Identity(STATE_SIZE, STATE_SIZE);
    EXPECT_THROW(ros_filter_utilities::load_covariance_parameter(node, "covariance", covariance),
      std::invalid_argument);
    EXPECT_TRUE(covariance.isIdentity());
  }
}

TEST_F(RosParameters, SensorMaskMustHaveExactlyFifteenEntries)
{
  for (const std::size_t length : {0, 1, 14, 16}) {
    SCOPED_TRACE(length);
    rclcpp::NodeOptions options;
    options.parameter_overrides({rclcpp::Parameter("imu0_config", std::vector<bool>(length, true))});
    rclcpp::Node node("sensor_mask_length", options);
    EXPECT_THROW(ros_filter_utilities::load_update_config(node, "imu0"), std::invalid_argument);
  }
  rclcpp::Node node("sensor_mask_defaults");
  EXPECT_EQ(ros_filter_utilities::load_update_config(node, "imu0"),
    std::vector<bool>(STATE_SIZE, false));
}

}  // namespace
}  // namespace rpp_localization
