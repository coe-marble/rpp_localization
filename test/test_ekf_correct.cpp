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
#include "angles/angles.h"
#include "rpp_localization/core/filter_common.hpp"
#include "rpp_localization/core/filter_utilities.hpp"
#include "rpp_localization/core/model_base.hpp"
#include "rpp_localization/core/validation.hpp"
#include "rpp_localization/filters/extended_kalman_filter.hpp"

namespace
{

bool g_count_allocations = false;
std::size_t g_allocations = 0;

}  // namespace

void* operator new(const std::size_t size)
{
  if (g_count_allocations) {
    ++g_allocations;
  }
  if (void* pointer = std::malloc(size == 0 ? 1 : size)) {
    return pointer;
  }
  throw std::bad_alloc();
}

namespace rpp_localization
{
namespace
{

class StaticModel final : public ModelBase
{
public:
  StaticModel()
  : ModelBase(STATE_SIZE)
  {
  }

  void predict(StateVector&, CovarianceMatrix&, TimestampNs, DurationNs) override {}
};

std::vector<bool> mask(const std::vector<int>& indices)
{
  std::vector<bool> result(STATE_SIZE, false);
  for (const int index : indices) {
    result[static_cast<std::size_t>(index)] = true;
  }
  return result;
}

const std::vector<std::vector<bool>>& masks()
{
  static const std::vector<std::vector<bool>> result{
    mask({0, 1, 2, 3, 4, 5}),
    mask({3, 4, 5, 9, 10, 11, 12, 13, 14}),
    mask({2, 6, 7, 11}),
    mask({0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14})};
  return result;
}

Eigen::MatrixXd spd(const double scale, const int seed)
{
  std::srand(static_cast<unsigned>(seed));
  const Eigen::MatrixXd a = Eigen::MatrixXd::Random(STATE_SIZE, STATE_SIZE);
  return scale * (a * a.transpose()) + 0.1 * Eigen::MatrixXd::Identity(STATE_SIZE, STATE_SIZE);
}

Measurement make_measurement(const std::vector<bool>& update_vector)
{
  Measurement measurement;
  measurement.update_vector_ = update_vector;
  measurement.measurement_ = Eigen::VectorXd::LinSpaced(STATE_SIZE, -0.7, 0.7);
  measurement.measurement_(StateMemberYaw) = -3.1;
  measurement.covariance_ = spd(0.05, 7);
  return measurement;
}

void prepare(StaticModel& model)
{
  model.get_state_unsafe() = Eigen::VectorXd::LinSpaced(STATE_SIZE, 0.3, -0.3);
  model.get_state_unsafe()(StateMemberYaw) = 3.1;
  model.get_state_covariance_unsafe() = spd(0.2, 3);
}

// Textbook EKF update with explicit inverse, as a reference.
void reference_correct(
  const Measurement& measurement, Eigen::VectorXd& state, Eigen::MatrixXd& covariance)
{
  std::vector<int> indices;
  for (int i = 0; i < STATE_SIZE; ++i) {
    if (measurement.update_vector_[static_cast<std::size_t>(i)] &&
      std::isfinite(measurement.measurement_(i)))
    {
      indices.push_back(i);
    }
  }
  const auto m = static_cast<Eigen::Index>(indices.size());
  Eigen::MatrixXd h = Eigen::MatrixXd::Zero(m, STATE_SIZE);
  Eigen::MatrixXd r(m, m);
  Eigen::VectorXd innovation(m);
  for (Eigen::Index i = 0; i < m; ++i) {
    const int row = indices[static_cast<std::size_t>(i)];
    h(i, row) = 1.0;
    innovation(i) = measurement.measurement_(row) - state(row);
    if (row == StateMemberRoll || row == StateMemberPitch || row == StateMemberYaw) {
      innovation(i) = ::angles::normalize_angle(innovation(i));
    }
    for (Eigen::Index j = 0; j < m; ++j) {
      r(i, j) = measurement.covariance_(row, indices[static_cast<std::size_t>(j)]);
    }
    r(i, i) = validation::normalize_measurement_covariance(r(i, i)).value;
  }
  const Eigen::MatrixXd s_inverse = (h * covariance * h.transpose() + r).inverse();
  if (!filter_utilities::check_mahalanobis_threshold(
      innovation, s_inverse, measurement.mahalanobis_thresh_))
  {
    return;
  }
  const Eigen::MatrixXd gain = covariance * h.transpose() * s_inverse;
  state += gain * innovation;
  const Eigen::MatrixXd residual =
    Eigen::MatrixXd::Identity(STATE_SIZE, STATE_SIZE) - gain * h;
  covariance = residual * covariance * residual.transpose() + gain * r * gain.transpose();
  filter_utilities::wrap_state_angles(state);
}

void expect_matches_reference(const Measurement& measurement)
{
  StaticModel model;
  prepare(model);
  Ekf filter(model);
  Eigen::VectorXd expected_state = model.get_state();
  Eigen::MatrixXd expected_covariance = model.get_state_covariance();
  reference_correct(measurement, expected_state, expected_covariance);

  filter.correct(measurement);

  EXPECT_TRUE(model.get_state().isApprox(expected_state, 1e-10));
  EXPECT_TRUE(model.get_state_covariance().isApprox(expected_covariance, 1e-10));
}

TEST(EkfCorrect, PerformsNoHeapAllocations)
{
  for (const auto& update_vector : masks()) {
    StaticModel model;
    prepare(model);
    Ekf filter(model);
    const Measurement measurement = make_measurement(update_vector);
    const Eigen::VectorXd before = model.get_state();

    g_allocations = 0;
    g_count_allocations = true;
    Eigen::internal::set_is_malloc_allowed(false);
    EXPECT_NO_THROW(filter.correct(measurement));
    Eigen::internal::set_is_malloc_allowed(true);
    g_count_allocations = false;

    EXPECT_EQ(g_allocations, 0u);
    EXPECT_FALSE(model.get_state().isApprox(before));
  }
}

TEST(EkfCorrect, MatchesTextbookUpdateForTypicalMasks)
{
  for (const auto& update_vector : masks()) {
    expect_matches_reference(make_measurement(update_vector));
  }
}

TEST(EkfCorrect, ExcludesNonFiniteMeasurementValues)
{
  auto measurement = make_measurement(masks()[0]);
  measurement.measurement_(1) = std::numeric_limits<double>::quiet_NaN();
  measurement.measurement_(4) = std::numeric_limits<double>::infinity();
  expect_matches_reference(measurement);
}

TEST(EkfCorrect, RejectsOutliersWithoutChangingTheEstimate)
{
  StaticModel model;
  prepare(model);
  Ekf filter(model);
  auto measurement = make_measurement(masks()[0]);
  measurement.mahalanobis_thresh_ = 1e-3;
  const Eigen::VectorXd state = model.get_state();
  const Eigen::MatrixXd covariance = model.get_state_covariance();

  filter.correct(measurement);

  EXPECT_EQ(model.get_state(), state);
  EXPECT_EQ(model.get_state_covariance(), covariance);
}

}  // namespace
}  // namespace rpp_localization
