#define EIGEN_RUNTIME_NO_MALLOC
#include <stdexcept>
#define eigen_assert(x) \
  do { if (!(x)) { throw std::runtime_error("Eigen assertion failed: " #x); } } while (false)

#include <cmath>
#include <cstddef>
#include <cstdlib>
#include <limits>
#include <new>
#include <vector>

#include "gtest/gtest.h"
#include "rpp_localization/filters/ekf_generic.hpp"
#include "rpp_localization/filters/extended_kalman_filter.hpp"
#include "rpp_localization/models/constant_acceleration_model.hpp"
#include "model_matrix_comparison.hpp"

namespace
{

bool g_count_allocations = false;
std::size_t g_allocations = 0;

class NoAllocationScope
{
public:
  NoAllocationScope()
  {
    g_allocations = 0;
    g_count_allocations = true;
    Eigen::internal::set_is_malloc_allowed(false);
  }

  ~NoAllocationScope()
  {
    Eigen::internal::set_is_malloc_allowed(true);
    g_count_allocations = false;
  }
};

}  // namespace

// Keep replacement allocation functions opaque to call-site diagnostics: GCC
// otherwise inlines delete's free() and reports it as a mismatched new/free.
#if defined(__GNUC__) || defined(__clang__)
#define RPP_TEST_NOINLINE __attribute__((noinline))
#else
#define RPP_TEST_NOINLINE
#endif

RPP_TEST_NOINLINE void* operator new(const std::size_t size)
{
  if (g_count_allocations) {
    ++g_allocations;
  }
  if (void* pointer = std::malloc(size == 0 ? 1 : size)) {
    return pointer;
  }
  throw std::bad_alloc();
}

RPP_TEST_NOINLINE void operator delete(void* pointer) noexcept { std::free(pointer); }
RPP_TEST_NOINLINE void operator delete(void* pointer, std::size_t) noexcept { std::free(pointer); }

#undef RPP_TEST_NOINLINE

namespace rpp_localization
{
namespace
{

class RuntimeModel final : public ModelBase
{
public:
  explicit RuntimeModel(int state_size)
  : ModelBase(state_size)
  {
  }

  void predict(
    StateVector& state, CovarianceMatrix& covariance,
    TimestampNs reference_time, DurationNs delta) override
  {
    last_reference_time = reference_time;
    last_delta = delta;
    state.array() += nanoseconds_to_seconds(delta);
    covariance.diagonal().array() += 0.01 * nanoseconds_to_seconds(delta);
  }

  TimestampNs last_reference_time{0};
  DurationNs last_delta{0};
};

Measurement make_measurement(int state_size, int width, bool reverse = false)
{
  Measurement measurement;
  measurement.update_vector_.assign(static_cast<std::size_t>(state_size), false);
  for (int index = 0; index < width; ++index) {
    const int axis = reverse ? state_size - index - 1 : index;
    measurement.update_vector_[static_cast<std::size_t>(axis)] = true;
  }
  measurement.measurement_ = Eigen::VectorXd::LinSpaced(state_size, -0.7, 0.7);
  measurement.covariance_ =
    0.05 * Eigen::MatrixXd::Identity(state_size, state_size) +
    0.005 * Eigen::MatrixXd::Ones(state_size, state_size);
  return measurement;
}

void expect_legacy_parity(const Measurement& measurement)
{
  RuntimeModel legacy_model(STATE_SIZE);
  RuntimeModel generic_model(STATE_SIZE);
  Eigen::VectorXd state = Eigen::VectorXd::LinSpaced(STATE_SIZE, 0.3, -0.3);
  state(StateMemberYaw) = 3.1;
  const Eigen::MatrixXd covariance =
    0.5 * Eigen::MatrixXd::Identity(STATE_SIZE, STATE_SIZE) +
    0.01 * Eigen::MatrixXd::Ones(STATE_SIZE, STATE_SIZE);
  legacy_model.set_state(state);
  generic_model.set_state(state);
  legacy_model.set_state_covariance(covariance);
  generic_model.set_state_covariance(covariance);
  Ekf legacy(legacy_model);
  EkfGeneric generic(generic_model, {StateMemberRoll, StateMemberPitch, StateMemberYaw});

  legacy.correct(measurement);
  generic.correct(measurement);

  EXPECT_TRUE(generic_model.get_state().isApprox(legacy_model.get_state(), 1e-10));
  EXPECT_TRUE(generic_model.get_state_covariance().isApprox(
    legacy_model.get_state_covariance(), 1e-10));

  generic_model.set_state(state);
  generic_model.set_state_covariance(covariance);
  auto fixed = generic.prepare_correction<STATE_SIZE>();
  fixed.correct(measurement);
  EXPECT_TRUE(generic_model.get_state().isApprox(legacy_model.get_state(), 1e-10));
  EXPECT_TRUE(generic_model.get_state_covariance().isApprox(
    legacy_model.get_state_covariance(), 1e-10));
}

TEST(EkfGeneric, MatchesOriginalAcrossAllMeasurementWidths)
{
  for (int width = 0; width <= STATE_SIZE; ++width) {
    for (const bool reverse : {false, true}) {
      auto measurement = make_measurement(STATE_SIZE, width, reverse);
      measurement.measurement_(StateMemberYaw) = -3.1;
      expect_legacy_parity(measurement);
    }
  }
}

TEST(EkfGeneric, MatchesOriginalForInvalidValuesCovarianceNormalizationAndGating)
{
  auto measurement = make_measurement(STATE_SIZE, STATE_SIZE);
  measurement.measurement_(StateMemberYaw) = -3.1;
  measurement.measurement_(1) = std::numeric_limits<double>::quiet_NaN();
  measurement.measurement_(4) = std::numeric_limits<double>::infinity();
  measurement.covariance_(0, 0) = -0.05;
  measurement.covariance_(2, 2) = 0.0;
  expect_legacy_parity(measurement);
  measurement.mahalanobis_thresh_ = 1e-6;
  expect_legacy_parity(measurement);
}

TEST(EkfGeneric, SupportsRuntimeDimensionsBelowAndAboveFifteen)
{
  for (const int state_size : {1, 4, 21}) {
    RuntimeModel model(state_size);
    model.set_state(Eigen::VectorXd::Constant(state_size, 0.2));
    model.set_state_covariance(0.5 * Eigen::MatrixXd::Identity(state_size, state_size));
    EkfGeneric filter(model);
    auto measurement = make_measurement(state_size, state_size);
    measurement.measurement_.setOnes();
    measurement.covariance_ = 0.5 * Eigen::MatrixXd::Identity(state_size, state_size);

    filter.correct(measurement);

    EXPECT_TRUE(model.get_state().isApprox(Eigen::VectorXd::Constant(state_size, 0.6), 1e-12));
    EXPECT_TRUE(model.get_state_covariance().isApprox(
      0.25 * Eigen::MatrixXd::Identity(state_size, state_size), 1e-12));
  }
}

TEST(EkfGeneric, WrapsOnlyConfiguredAngleCoordinates)
{
  RuntimeModel model(4);
  model.set_state(Eigen::VectorXd::Constant(4, 3.1));
  model.set_state_covariance(0.5 * Eigen::MatrixXd::Identity(4, 4));
  EkfGeneric filter(model, {3});
  auto measurement = make_measurement(4, 4);
  measurement.measurement_.setConstant(-3.1);
  measurement.covariance_ = 0.5 * Eigen::MatrixXd::Identity(4, 4);

  filter.correct(measurement);

  EXPECT_NEAR(model.get_state()(0), 0.0, 1e-12);
  EXPECT_NEAR(model.get_state()(1), 0.0, 1e-12);
  EXPECT_NEAR(model.get_state()(2), 0.0, 1e-12);
  EXPECT_NEAR(std::abs(model.get_state()(3)), PI, 1e-12);
}

TEST(EkfGeneric, PredictsThroughTheBoundRuntimeModel)
{
  RuntimeModel model(4);
  EkfGeneric filter(model);

  filter.predict(1'000'000'000, 250'000'000);

  EXPECT_EQ(model.last_reference_time, 1'000'000'000);
  EXPECT_EQ(model.last_delta, 250'000'000);
  EXPECT_TRUE(model.get_state().isApprox(Eigen::VectorXd::Constant(4, 0.25)));
  EXPECT_TRUE(model.get_state_covariance().isApprox(0.0025 * Eigen::MatrixXd::Identity(4, 4)));
}

TEST(EkfGeneric, RejectsInvalidAngleBindings)
{
  RuntimeModel model(4);
  EXPECT_THROW((EkfGeneric(model, {-1})), std::invalid_argument);
  EXPECT_THROW((EkfGeneric(model, {4})), std::invalid_argument);
  EXPECT_THROW((EkfGeneric(model, {1, 1})), std::invalid_argument);
}

TEST(EkfGeneric, RejectsFixedBindingForDifferentStateDimension)
{
  RuntimeModel model(4);
  EkfGeneric filter(model);
  EXPECT_THROW(filter.prepare_correction<15>(), std::invalid_argument);
  EXPECT_NO_THROW(filter.prepare_correction<4>());
  EXPECT_NO_THROW(filter.prepare_correction<Eigen::Dynamic>());
}

TEST(EkfGeneric, ReusesPreparedStorageOnFirstUseAndAcrossMeasurementWidths)
{
  for (const int state_size : {4, 15, 21}) {
    RuntimeModel model(state_size);
    EkfGeneric filter(model, {state_size - 1});
    auto fixed = state_size == 4 ? filter.prepare_correction<4>() :
      (state_size == 15 ? filter.prepare_correction<15>() : filter.prepare_correction<21>());
    auto dynamic = filter.prepare_correction<Eigen::Dynamic>();
    const Eigen::VectorXd state = Eigen::VectorXd::Constant(state_size, 0.2);
    const Eigen::MatrixXd covariance = Eigen::MatrixXd::Identity(state_size, state_size);
    std::vector<Measurement> measurements;
    for (int width = 0; width <= state_size; ++width) {
      measurements.push_back(make_measurement(state_size, width));
    }
    for (int width = state_size; width >= 0; --width) {
      measurements.push_back(make_measurement(state_size, width, true));
    }

    for (const auto& measurement : measurements) {
      for (const auto binding : {fixed, dynamic}) {
        model.set_state(state);
        model.set_state_covariance(covariance);
        {
          NoAllocationScope scope;
          EXPECT_NO_THROW(binding.correct(measurement));
        }
        EXPECT_EQ(g_allocations, 0u);
      }
    }
  }
}

TEST(ModelMatrixComparison, BothViewsMatchOriginalAndReusePreparedStorage)
{
  StateVector state = StateVector::LinSpaced(STATE_SIZE, 0.3, -0.3);
  state(StateMemberYaw) = 3.1;
  state(StateMemberVx) = 2.0;
  state(StateMemberAx) = 0.4;
  const CovarianceMatrix covariance = 0.5 * CovarianceMatrix::Identity(STATE_SIZE, STATE_SIZE) +
    0.01 * CovarianceMatrix::Ones(STATE_SIZE, STATE_SIZE);

  for (int mode = 0; mode < 5; ++mode) {
    ConstantAccelerationModel original(STATE_SIZE);
    original.initialize_runtime();
    rpp_localization::test::ModelMatrixComparison compared(STATE_SIZE);
    for (NavigationModelBase* model : {static_cast<NavigationModelBase*>(&original),
        static_cast<NavigationModelBase*>(&compared)})
    {
      model->set_computes_jacobian(mode >= 1);
      model->set_computes_covariance(mode >= 2);
      model->set_dynamic_process_noise_covariance(mode == 3);
      if (mode == 4) {
        model->set_control_params(std::vector<bool>(TWIST_SIZE, true), 1'000'000'000,
          std::vector<double>(TWIST_SIZE, 1.0), std::vector<double>(TWIST_SIZE, 0.5),
          std::vector<double>(TWIST_SIZE, 1.0), std::vector<double>(TWIST_SIZE, 0.5));
        ControlCommand command;
        command.stamp = 990'000'000;
        command.control.setConstant(0.4);
        model->set_control(command);
      }
    }
    auto fixed = compared.prepare_prediction<STATE_SIZE>();
    auto dynamic = compared.prepare_prediction<Eigen::Dynamic>();
    for (const TimestampNs reference_time : {1'000'000'000LL, 3'000'000'000LL}) {
      original.set_state(state);
      original.set_state_covariance(covariance);
      original.predict(original.get_state_unsafe(), original.get_state_covariance_unsafe(),
        reference_time, 33'333'333);
      for (const auto binding : {fixed, dynamic}) {
        compared.set_state(state);
        compared.set_state_covariance(covariance);
        {
          NoAllocationScope scope;
          EXPECT_NO_THROW(binding.predict(reference_time, 33'333'333));
        }
        EXPECT_EQ(g_allocations, 0u);
        EXPECT_TRUE(compared.get_state().isApprox(original.get_state(), 1e-10));
        EXPECT_TRUE(compared.get_state_covariance().isApprox(
          original.get_state_covariance(), 1e-10));
      }
    }
  }
}

TEST(ModelMatrixComparison, RejectsDifferentNavigationAndBindingDimensions)
{
  EXPECT_THROW((rpp_localization::test::ModelMatrixComparison(4)), std::invalid_argument);
  rpp_localization::test::ModelMatrixComparison model(STATE_SIZE);
  EXPECT_THROW(model.prepare_prediction<4>(), std::invalid_argument);
}

}  // namespace
}  // namespace rpp_localization
