// Ekf::correct benchmark: allocations per call, mean and p99 duration per update mask.
#include <algorithm>
#include <chrono>
#include <cstddef>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>

#include "rpp_localization/core/filter_common.hpp"
#include "rpp_localization/core/model_base.hpp"
#include "rpp_localization/filters/extended_kalman_filter.hpp"

#if defined(__GLIBC__)
namespace
{
bool g_count_allocations = false;
std::size_t g_allocations = 0;
}  // namespace

extern "C" {
void* __libc_malloc(std::size_t size);
void* __libc_calloc(std::size_t count, std::size_t size);
void* __libc_realloc(void* pointer, std::size_t size);

void* malloc(std::size_t size) noexcept
{
  g_allocations += g_count_allocations ? 1 : 0;
  return __libc_malloc(size);
}

void* calloc(std::size_t count, std::size_t size) noexcept
{
  g_allocations += g_count_allocations ? 1 : 0;
  return __libc_calloc(count, size);
}

void* realloc(void* pointer, std::size_t size) noexcept
{
  g_allocations += g_count_allocations ? 1 : 0;
  return __libc_realloc(pointer, size);
}
}
#endif

namespace
{

using rpp_localization::STATE_SIZE;

class StaticModel final : public rpp_localization::ModelBase
{
public:
  StaticModel()
  : ModelBase(STATE_SIZE)
  {
  }

  void predict(
    rpp_localization::StateVector&, rpp_localization::CovarianceMatrix&,
    rpp_localization::TimestampNs, rpp_localization::DurationNs) override
  {
  }
};

struct Case
{
  std::string name;
  std::vector<int> indices;
};

}  // namespace

int main(int argc, char** argv)
{
  const int iterations = argc > 1 ? std::atoi(argv[1]) : 200000;
  const std::vector<Case> cases{
    {"position (3)", {0, 1, 2}},
    {"pose (6)", {0, 1, 2, 3, 4, 5}},
    {"imu (9)", {3, 4, 5, 9, 10, 11, 12, 13, 14}},
    {"full (15)", {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14}}};

  std::srand(1);
  const Eigen::MatrixXd a = Eigen::MatrixXd::Random(STATE_SIZE, STATE_SIZE);
  const Eigen::MatrixXd covariance =
    0.2 * a * a.transpose() + 0.1 * Eigen::MatrixXd::Identity(STATE_SIZE, STATE_SIZE);
  const Eigen::VectorXd state = Eigen::VectorXd::LinSpaced(STATE_SIZE, 0.3, -0.3);

  std::printf("%-14s %12s %10s %10s\n", "mask", "allocs/call", "mean [us]", "p99 [us]");
  for (const auto& test_case : cases) {
    StaticModel model;
    rpp_localization::Ekf filter(model);
    rpp_localization::Measurement measurement;
    measurement.update_vector_.assign(STATE_SIZE, false);
    for (const int index : test_case.indices) {
      measurement.update_vector_[static_cast<std::size_t>(index)] = true;
    }
    measurement.measurement_ = Eigen::VectorXd::LinSpaced(STATE_SIZE, -0.2, 0.2);
    measurement.covariance_ = 0.05 * Eigen::MatrixXd::Identity(STATE_SIZE, STATE_SIZE);

    std::vector<double> durations(static_cast<std::size_t>(iterations));
    std::size_t allocations = 0;
    for (int i = 0; i < iterations; ++i) {
      model.get_state_unsafe() = state;
      model.get_state_covariance_unsafe() = covariance;
#if defined(__GLIBC__)
      g_allocations = 0;
      g_count_allocations = true;
#endif
      const auto start = std::chrono::steady_clock::now();
      filter.correct(measurement);
      const auto stop = std::chrono::steady_clock::now();
#if defined(__GLIBC__)
      g_count_allocations = false;
      allocations += g_allocations;
#endif
      durations[static_cast<std::size_t>(i)] =
        std::chrono::duration<double, std::micro>(stop - start).count();
    }

    double sum = 0.0;
    for (const double duration : durations) {
      sum += duration;
    }
    std::sort(durations.begin(), durations.end());
    const auto p99 = durations[static_cast<std::size_t>(0.99 * (durations.size() - 1))];
    std::printf(
      "%-14s %12.1f %10.3f %10.3f\n", test_case.name.c_str(),
      static_cast<double>(allocations) / iterations, sum / iterations, p99);
  }
  return 0;
}
