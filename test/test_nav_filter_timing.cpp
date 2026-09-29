#include <memory>
#include <vector>

#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"
#include "rpp_localization/core/filter_common.hpp"
#include "rpp_localization/core/filter_base.hpp"
#include "rpp_localization/filters/nav_filter.hpp"
#include "rpp_localization/models/nav_model_base.hpp"

#include "filters/nav_filter.cpp"

namespace rpp_localization
{
namespace
{

class TimingModel final : public NavModelBase
{
public:
  TimingModel()
  : NavModelBase(STATE_SIZE)
  {
  }

  void step(
    Eigen::VectorXd&,
    Eigen::MatrixXd&,
    const rclcpp::Time&,
    const double) override
  {
  }
};

class TimingFilter final : public FilterBase
{
public:
  explicit TimingFilter(const int state_dim)
  : FilterBase(state_dim),
    model()
  {
    set_model_base(model);
  }

  void init(rclcpp::Node&)
  {
  }

  void correct(const Measurement&) override
  {
    ++correction_count;
  }

  void predict(const TimestampNs reference_time, const DurationNs delta) override
  {
    ++prediction_count;
    last_prediction_time = reference_time;
    last_prediction_delta = delta;
  }

  TimingModel model;
  int correction_count{};
  int prediction_count{};
  TimestampNs last_prediction_time{};
  DurationNs last_prediction_delta{};
};

Measurement makeMeasurement(const TimestampNs time)
{
  Measurement measurement;
  measurement.time_ = time;
  measurement.topic_name_ = "timing";
  measurement.update_vector_ = std::vector<bool>(STATE_SIZE, false);
  return measurement;
}

}  // namespace

class NavFilterTimingParityTest : public testing::Test
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

TEST_F(NavFilterTimingParityTest, PreservesLegacyStaleAndForwardMeasurementHandling)
{
  NavFilter<TimingFilter> filter;
  auto node = rclcpp::Node::make_shared("nav_filter_timing_parity");
  filter.init(*node);

  filter.process_measurement(makeMeasurement(100));
  EXPECT_EQ(filter.get_last_measurement_time(), 100);
  EXPECT_EQ(filter.get_filter().prediction_count, 0);
  EXPECT_EQ(filter.get_filter().correction_count, 0);

  filter.process_measurement(makeMeasurement(90));
  EXPECT_EQ(filter.get_last_measurement_time(), 100);
  EXPECT_EQ(filter.get_filter().prediction_count, 0);
  EXPECT_EQ(filter.get_filter().correction_count, 1);

  filter.process_measurement(makeMeasurement(100));
  EXPECT_EQ(filter.get_last_measurement_time(), 100);
  EXPECT_EQ(filter.get_filter().prediction_count, 0);
  EXPECT_EQ(filter.get_filter().correction_count, 2);

  filter.process_measurement(makeMeasurement(110));
  EXPECT_EQ(filter.get_last_measurement_time(), 110);
  EXPECT_EQ(filter.get_filter().prediction_count, 1);
  EXPECT_EQ(filter.get_filter().last_prediction_time, 110);
  EXPECT_EQ(filter.get_filter().last_prediction_delta, 10);
  EXPECT_EQ(filter.get_filter().correction_count, 3);
}

}  // namespace rpp_localization
