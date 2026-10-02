#include <type_traits>

#include "gtest/gtest.h"
#include "rpp_localization/inekf/inertial_prediction_model.hpp"
#include "rpp_localization/inekf/inertial_process.hpp"

namespace rpp_localization::InEKF
{
namespace
{

class RecordingInertialPredictionModel final : public InertialPredictionModel
{
public:
  void predict(
    InertialLieState&,
    const InertialPredictionInput& input) override
  {
    reference_time = input.reference_time;
    delta = input.delta;
    imu = input.imu;
  }

  TimestampNs reference_time{};
  DurationNs delta{};
  Eigen::Matrix<double, 6, 1> imu = Eigen::Matrix<double, 6, 1>::Zero();
};

static_assert(std::is_base_of_v<InertialPredictionModel, InertialProcess>);

}  // namespace

TEST(InertialPredictionModelTest, KeepsLieStateAndImuInputSeparateFromState15)
{
  InertialLieState state;
  InertialPredictionInput input;
  input.reference_time = 42;
  input.delta = 250'000'000;
  input.imu << 1.0, 2.0, 3.0, 4.0, 5.0, 6.0;

  RecordingInertialPredictionModel model;
  InertialPredictionModel& runtime_model = model;
  runtime_model.predict(state, input);

  EXPECT_EQ(model.reference_time, 42);
  EXPECT_EQ(model.delta, 250'000'000);
  EXPECT_EQ(model.imu, input.imu);
  EXPECT_EQ(state.cov().rows(), 15);
}

}  // namespace rpp_localization::InEKF
