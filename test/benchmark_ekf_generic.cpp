// Compare the original EKF, matched Eigen views, and model prediction in one executable.
// Usage: benchmark_ekf_generic [iterations_per_case] (default 100000).
// Configuration, input selection, state reset, and allocation counting
// are outside timed regions. See plans/interfaces-migration.md.
#include <algorithm>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <utility>
#include <vector>

#include "rpp_localization/filters/ekf_generic.hpp"
#include "rpp_localization/filters/extended_kalman_filter.hpp"
#include "rpp_localization/models/constant_acceleration_model.hpp"
#include "model_matrix_comparison.hpp"

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

using namespace rpp_localization;

volatile double g_checksum = 0.0;

class StaticModel final : public ModelBase
{
public:
  StaticModel()
  : ModelBase(STATE_SIZE)
  {
  }

  void predict(StateVector&, CovarianceMatrix&, TimestampNs, DurationNs) override {}
};

struct Workload
{
  const char* name;
  std::vector<std::size_t> measurements;
};

struct Statistics
{
  double mean;
  double median;
  double p99;
};

std::vector<Measurement> make_measurements()
{
  const std::vector<std::vector<int>> masks{
    {0, 1, 2},
    {0, 1, 2, 3, 4, 5},
    {3, 4, 5, 9, 10, 11, 12, 13, 14},
    {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14}};
  std::vector<Measurement> measurements;
  for (const auto& mask : masks) {
    Measurement measurement;
    measurement.update_vector_.assign(STATE_SIZE, false);
    for (const int index : mask) {
      measurement.update_vector_[static_cast<std::size_t>(index)] = true;
    }
    measurement.measurement_ = Eigen::VectorXd::LinSpaced(STATE_SIZE, -0.2, 0.2);
    measurement.measurement_(StateMemberYaw) = -3.1;
    measurement.covariance_ =
      0.05 * Eigen::MatrixXd::Identity(STATE_SIZE, STATE_SIZE) +
      0.002 * Eigen::MatrixXd::Ones(STATE_SIZE, STATE_SIZE);
    measurements.push_back(std::move(measurement));
  }
  return measurements;
}

void reset(ModelBase& model, const StateVector& state, const CovarianceMatrix& covariance)
{
  model.set_state(state);
  model.set_state_covariance(covariance);
}

void keep_result(const ModelBase& model)
{
#if defined(__GNUC__) || defined(__clang__)
  // Require the complete result to be materialized before reading the clock.
  __asm__ __volatile__("" : :
    "g"(model.get_state().data()), "g"(model.get_state_covariance().data()) : "memory");
#else
  std::atomic_signal_fence(std::memory_order_seq_cst);
#endif
}

template<typename Filter>
struct CorrectionUpdate
{
  Filter& filter;
  const Workload& workload;
  const std::vector<Measurement>& measurements;
  const Measurement* current{nullptr};

  void prepare(std::size_t index)
  {
    current = &measurements[workload.measurements[index % workload.measurements.size()]];
  }

  void operator()() const { filter.correct(*current); }
};

struct PredictionUpdate
{
  rpp_localization::test::ModelMatrixComparison::PredictionBinding binding;
  TimestampNs reference_time;
  DurationNs delta;

  void prepare(std::size_t) {}
  void operator()() const { binding.predict(reference_time, delta); }
};

template<typename Update>
double sample(
  Update& update, ModelBase& model, const StateVector& state,
  const CovarianceMatrix& covariance, std::size_t index)
{
  update.prepare(index);
  reset(model, state, covariance);
  const auto start = std::chrono::steady_clock::now();
  update();
  keep_result(model);
  const auto stop = std::chrono::steady_clock::now();
  g_checksum += model.get_state()(0) + model.get_state_covariance()(0, 0);
  return std::chrono::duration<double, std::micro>(stop - start).count();
}

template<typename Update>
double count_allocations(
  Update& update, ModelBase& model, const StateVector& state,
  const CovarianceMatrix& covariance, std::size_t iterations)
{
#if defined(__GLIBC__)
  std::size_t allocations = 0;
  for (std::size_t index = 0; index < iterations; ++index) {
    update.prepare(index);
    reset(model, state, covariance);
    g_allocations = 0;
    g_count_allocations = true;
    update();
    keep_result(model);
    g_count_allocations = false;
    allocations += g_allocations;
  }
  return static_cast<double>(allocations) / static_cast<double>(iterations);
#else
  (void)update;
  (void)model;
  (void)state;
  (void)covariance;
  (void)iterations;
  return std::numeric_limits<double>::quiet_NaN();
#endif
}

Statistics summarize(std::vector<double>& samples)
{
  double sum = 0.0;
  for (const double sample : samples) {
    sum += sample;
  }
  std::sort(samples.begin(), samples.end());
  const auto count = samples.size();
  const auto p99_index = static_cast<std::size_t>(std::ceil(0.99 * count)) - 1;
  return {sum / count, (samples[(count - 1) / 2] + samples[count / 2]) * 0.5, samples[p99_index]};
}

void print_statistics(const char* name, const Statistics& statistics, double allocations)
{
  char allocation_text[32];
  if (std::isfinite(allocations)) {
    std::snprintf(allocation_text, sizeof(allocation_text), "%.3f", allocations);
  } else {
    std::snprintf(allocation_text, sizeof(allocation_text), "n/a");
  }
  std::printf("  %-12s %12s %12.3f %12.3f %12.3f\n", name, allocation_text,
    statistics.mean, statistics.median, statistics.p99);
}

double percent_change(double baseline, double candidate)
{
  return baseline > 0.0 ? 100.0 * (candidate / baseline - 1.0) :
         std::numeric_limits<double>::quiet_NaN();
}

struct ComparisonInputs
{
  const StateVector& state;
  const CovarianceMatrix& covariance;
  std::size_t iterations;
};

template<typename BaselineUpdate, typename CandidateUpdate>
bool compare_updates(
  const char* name, const char* baseline_name, const char* candidate_name,
  BaselineUpdate& baseline, ModelBase& baseline_model,
  CandidateUpdate& candidate, ModelBase& candidate_model,
  const ComparisonInputs& inputs, std::size_t distinct_inputs)
{
  for (std::size_t index = 0; index < distinct_inputs; ++index) {
    baseline.prepare(index);
    reset(baseline_model, inputs.state, inputs.covariance);
    baseline();
    // Copies are necessary: controlled comparisons reuse the very same model.
    const StateVector expected_state = baseline_model.get_state();
    const CovarianceMatrix expected_covariance = baseline_model.get_state_covariance();
    candidate.prepare(index);
    reset(candidate_model, inputs.state, inputs.covariance);
    candidate();
    if (!candidate_model.get_state().isApprox(expected_state, 1e-10) ||
      !candidate_model.get_state_covariance().isApprox(expected_covariance, 1e-10))
    {
      std::fprintf(stderr, "Parity check failed for %s; stopping comparison.\n", name);
      return false;
    }
  }

  for (std::size_t index = 0; index < 1000; ++index) {
    baseline.prepare(index);
    reset(baseline_model, inputs.state, inputs.covariance);
    baseline();
    keep_result(baseline_model);
    candidate.prepare(index);
    reset(candidate_model, inputs.state, inputs.covariance);
    candidate();
    keep_result(candidate_model);
  }

  std::vector<double> baseline_samples(inputs.iterations);
  std::vector<double> candidate_samples(inputs.iterations);
  for (std::size_t index = 0; index < inputs.iterations; ++index) {
    // Same sampling function for both bindings. Alternate their call order.
    if (index % 2 == 0) {
      baseline_samples[index] = sample(baseline, baseline_model,
        inputs.state, inputs.covariance, index);
      candidate_samples[index] = sample(candidate, candidate_model,
        inputs.state, inputs.covariance, index);
    } else {
      candidate_samples[index] = sample(candidate, candidate_model,
        inputs.state, inputs.covariance, index);
      baseline_samples[index] = sample(baseline, baseline_model,
        inputs.state, inputs.covariance, index);
    }
  }
  const auto baseline_statistics = summarize(baseline_samples);
  const auto candidate_statistics = summarize(candidate_samples);
  const auto count_iterations = std::min<std::size_t>(inputs.iterations, 1000);
  const double baseline_allocations = count_allocations(
    baseline, baseline_model, inputs.state, inputs.covariance, count_iterations);
  const double candidate_allocations = count_allocations(
    candidate, candidate_model, inputs.state, inputs.covariance, count_iterations);

  std::printf("%s\n", name);
  print_statistics(baseline_name, baseline_statistics, baseline_allocations);
  print_statistics(candidate_name, candidate_statistics, candidate_allocations);
  std::printf("  Candidate time change: mean %+.1f%%, median %+.1f%%, p99 %+.1f%%\n",
    percent_change(baseline_statistics.mean, candidate_statistics.mean),
    percent_change(baseline_statistics.median, candidate_statistics.median),
    percent_change(baseline_statistics.p99, candidate_statistics.p99));
  return true;
}

struct ModelCase
{
  const char* name;
  bool jacobian;
  bool covariance;
  bool dynamic_noise;
  bool control;
};

void configure_model(NavigationModelBase& model, const ModelCase& mode)
{
  model.set_computes_jacobian(mode.jacobian);
  model.set_computes_covariance(mode.covariance);
  model.set_dynamic_process_noise_covariance(mode.dynamic_noise);
  if (mode.control) {
    model.set_control_params(std::vector<bool>(TWIST_SIZE, true), 1'000'000'000,
      std::vector<double>(TWIST_SIZE, 1.0), std::vector<double>(TWIST_SIZE, 0.5),
      std::vector<double>(TWIST_SIZE, 1.0), std::vector<double>(TWIST_SIZE, 0.5));
    ControlCommand command;
    command.stamp = 990'000'000;
    command.control.setConstant(0.4);
    model.set_control(command);
  }
}

}  // namespace

int main(int argc, char** argv)
{
#ifndef NDEBUG
  std::fprintf(stderr, "Build benchmark_ekf_generic in Release or RelWithDebInfo for timing.\n");
  return 2;
#endif
  std::size_t iterations = 100000;
  if (argc > 2) {
    std::fprintf(stderr, "Usage: benchmark_ekf_generic [iterations_per_case]\n");
    return 2;
  }
  if (argc == 2) {
    char* end = nullptr;
    errno = 0;
    const auto parsed = std::strtoull(argv[1], &end, 10);
    if (argv[1][0] == '-' || end == argv[1] || *end != '\0' || errno != 0 ||
      parsed == 0 || parsed > 10000000)
    {
      std::fprintf(stderr, "Iterations must be an integer from 1 to 10000000.\n");
      return 2;
    }
    iterations = static_cast<std::size_t>(parsed);
  }

  const std::vector<Workload> workloads{
    {"position (3)", {0}}, {"pose (6)", {1}}, {"imu (9)", {2}},
    {"full (15)", {3}}, {"mixed widths", {0, 1, 2, 3}}};
  const auto measurements = make_measurements();
  std::srand(1);
  const Eigen::MatrixXd a = Eigen::MatrixXd::Random(STATE_SIZE, STATE_SIZE);
  const CovarianceMatrix covariance =
    0.2 * a * a.transpose() + 0.1 * Eigen::MatrixXd::Identity(STATE_SIZE, STATE_SIZE);
  StateVector state = Eigen::VectorXd::LinSpaced(STATE_SIZE, 0.3, -0.3);
  state(StateMemberYaw) = 3.1;

  StaticModel legacy_model;
  StaticModel generic_model;
  Ekf legacy(legacy_model);
  EkfGeneric generic(generic_model, {StateMemberRoll, StateMemberPitch, StateMemberYaw});
  const ComparisonInputs correction_inputs{state, covariance, iterations};

  std::printf("15 states, %zu paired samples per case, 1000 warmup updates.\n", iterations);
  std::printf("Configuration, input selection and reset excluded; allocations counted separately.\n");
  std::printf("Time change: positive = candidate slower, negative = candidate faster.\n");
  std::printf("  %-12s %12s %12s %12s %12s\n",
    "variant", "allocs/call", "mean [us]", "median [us]", "p99 [us]");

  std::printf("\nOriginal EKF comparison (also differs in workspace and angle handling)\n");
  for (const auto& workload : workloads) {
    CorrectionUpdate<Ekf> baseline{legacy, workload, measurements};
    CorrectionUpdate<EkfGeneric> candidate{generic, workload, measurements};
    if (!compare_updates(workload.name, "Ekf", "EkfGeneric",
        baseline, legacy_model, candidate, generic_model,
        correction_inputs, workload.measurements.size()))
    {
      return 1;
    }
  }

  std::printf("\nControlled EKF comparison (same code, buffers, angles and call path)\n");
  std::printf("Only state dimensions differ in Eigen types; observation width and LDLT remain dynamic.\n");
  auto fixed_correction = generic.prepare_correction<STATE_SIZE>();
  auto dynamic_correction = generic.prepare_correction<Eigen::Dynamic>();
  for (const auto& workload : workloads) {
    CorrectionUpdate<EkfGeneric::CorrectionBinding> baseline{
      fixed_correction, workload, measurements};
    CorrectionUpdate<EkfGeneric::CorrectionBinding> candidate{
      dynamic_correction, workload, measurements};
    if (!compare_updates(workload.name, "StateFixed15", "StateDynamic",
        baseline, generic_model, candidate, generic_model,
        correction_inputs, workload.measurements.size()))
    {
      return 1;
    }
  }

  std::printf("\nControlled model comparison (same 15-state equations, buffers and call path)\n");
  const std::vector<ModelCase> model_cases{
    {"state only", false, false, false, false},
    {"state + Jacobian", true, false, false, false},
    {"state + covariance", true, true, false, false},
    {"dynamic process noise", true, true, true, false},
    {"control + covariance", true, true, false, true}};
  StateVector model_state = state;
  model_state(StateMemberVx) = 2.0;
  model_state(StateMemberVy) = 0.2;
  model_state(StateMemberAx) = 0.4;
  const ComparisonInputs prediction_inputs{model_state, covariance, iterations};
  constexpr TimestampNs reference_time = 1'000'000'000;
  constexpr DurationNs delta = 33'333'333;
  for (const auto& mode : model_cases) {
    rpp_localization::test::ModelMatrixComparison model(STATE_SIZE);
    ConstantAccelerationModel original(STATE_SIZE);
    original.initialize_runtime();
    configure_model(model, mode);
    configure_model(original, mode);
    PredictionUpdate baseline{model.prepare_prediction<STATE_SIZE>(), reference_time, delta};
    PredictionUpdate candidate{model.prepare_prediction<Eigen::Dynamic>(), reference_time, delta};

    reset(original, model_state, covariance);
    original.predict(original.get_state_unsafe(), original.get_state_covariance_unsafe(),
      reference_time, delta);
    reset(model, model_state, covariance);
    candidate();
    if (!model.get_state().isApprox(original.get_state(), 1e-10) ||
      !model.get_state_covariance().isApprox(original.get_state_covariance(), 1e-10))
    {
      std::fprintf(stderr, "Model comparison differs from original for %s.\n", mode.name);
      return 1;
    }
    if (!compare_updates(mode.name, "ModelFixed15", "ModelDynamic",
        baseline, model, candidate, model, prediction_inputs, 1))
    {
      return 1;
    }
  }
  std::printf("Result checksum: %.9g\n", static_cast<double>(g_checksum));
  return 0;
}
