#define EIGEN_RUNTIME_NO_MALLOC
#include <stdexcept>
#define eigen_assert(x) \
  do { if (!(x)) { throw std::runtime_error("Eigen assertion failed: " #x); } } while (false)

#include <cstdlib>
#include <limits>
#include <new>
#include <array>
#include <atomic>
#include <chrono>
#include <thread>

#include <gtest/gtest.h>

#include "rpp_localization/estimation/ekf.hpp"
#include "rpp_localization/estimation/ukf.hpp"
#include "rpp_localization/estimation/replay.hpp"
#include "rpp_localization/estimation/prepared_queue.hpp"
#include "rpp_localization/estimation/async_replay.hpp"
#include "rpp_localization/models/native_navigation15.hpp"

namespace
{
bool count_allocations = false;
std::size_t allocations = 0;

struct NoAllocations
{
  NoAllocations()
  {
    allocations = 0;
    count_allocations = true;
    Eigen::internal::set_is_malloc_allowed(false);
  }
  ~NoAllocations()
  {
    Eigen::internal::set_is_malloc_allowed(true);
    count_allocations = false;
  }
};
}

// Keep replacement allocation functions opaque to call-site diagnostics: GCC
// otherwise inlines delete's free() and reports it as a mismatched new/free.
#if defined(__GNUC__) || defined(__clang__)
#define RPP_TEST_NOINLINE __attribute__((noinline))
#else
#define RPP_TEST_NOINLINE
#endif

RPP_TEST_NOINLINE void* operator new(std::size_t size)
{
  if (count_allocations) {++allocations;}
  if (void* pointer = std::malloc(size == 0 ? 1 : size)) {
    return pointer;
  }
  throw std::bad_alloc();
}
RPP_TEST_NOINLINE void operator delete(void* pointer) noexcept { std::free(pointer); }
RPP_TEST_NOINLINE void operator delete(void* pointer, std::size_t) noexcept { std::free(pointer); }

#undef RPP_TEST_NOINLINE

namespace rpp_localization::test
{
namespace
{
namespace e = estimation;

TEST(ReplayStatistics, ComputesPopulationDeviationAndMergesWithoutAllocating)
{
  e::ReplayStatistics first, second;
  EXPECT_DOUBLE_EQ(first.states_stddev(), 0.0);
  {
    NoAllocations guard;
    first.observe(1); first.observe(3); second.observe(5); first.merge(second);
  }
  EXPECT_EQ(allocations, 0u);
  EXPECT_EQ(first.count, 3u); EXPECT_EQ(first.states_max, 5u);
  EXPECT_DOUBLE_EQ(first.states_mean, 3.0);
  EXPECT_NEAR(first.states_stddev(), std::sqrt(8.0 / 3.0), 1e-12);
  EXPECT_NE(first.json().find("\"states_stddev\":"), std::string::npos);
}

TEST(AsyncReplay, SpareIsIdleAndOnlyCatchesUpAfterAnExplicitFork)
{
  e::AsyncReplay replay;
  replay.prepare(4, 1, 1);
  const std::vector<double> values{1.0}, covariance{1.0};
  std::size_t applications = 0, switches = 0, forks = 0;
  bool first_switch = true, stale_switch = true, final_switch = false;
  {
    NoAllocations guard;
    replay.record(e::AsyncReplay::Kind::Predict, 0, values, covariance, 20, 20);
    replay.service(4, [&](const auto&) {++applications; return true;});
    replay.start([&] {++forks; return true;});
    replay.record(e::AsyncReplay::Kind::Correct, 0, values, covariance, 10);
    first_switch = replay.try_switch([&] {++switches;});
    replay.service(4, [&](const auto& event) {++applications; return event.sequence == 2;});
    // The live instance continues after the candidate is ready. An older
    // candidate cannot be adopted; the consumer must process this next event.
    replay.record(e::AsyncReplay::Kind::Predict, 0, values, covariance, 30, 10);
    stale_switch = replay.try_switch([&] {++switches;});
    replay.service(4, [&](const auto& event) {++applications; return event.sequence == 3;});
    final_switch = replay.try_switch([&] {++switches;});
    replay.record(e::AsyncReplay::Kind::Correct, 0, values, covariance, 30);
    replay.service(4, [&](const auto&) {++applications; return true;});
  }
  EXPECT_EQ(allocations, 0u);
  EXPECT_EQ(forks, 1u); EXPECT_EQ(applications, 2u); EXPECT_EQ(switches, 1u);
  EXPECT_FALSE(first_switch); EXPECT_FALSE(stale_switch); EXPECT_TRUE(final_switch);
  EXPECT_EQ(replay.phase(), e::AsyncReplay::Phase::Idle);
}

TEST(AsyncReplay, OverflowAndFailedWorkNeverAdoptACandidateAndResetInvalidatesIt)
{
  e::AsyncReplay replay;
  replay.prepare(1, 1, 1);
  const std::vector<double> values{1.0}, covariance{1.0};
  ASSERT_TRUE(replay.start([] {return true;}));
  ASSERT_TRUE(replay.record(e::AsyncReplay::Kind::Correct, 0, values, covariance, 10));
  EXPECT_FALSE(replay.record(e::AsyncReplay::Kind::Correct, 0, values, covariance, 11));
  EXPECT_FALSE(replay.try_switch([] {FAIL() << "Overflowed work must never become live";}));
  EXPECT_FALSE(replay.service(1, [](const auto&) {return true;}));
  replay.reset();
  ASSERT_TRUE(replay.start([] {return true;}));
  ASSERT_TRUE(replay.record(e::AsyncReplay::Kind::Correct, 0, values, covariance, 12));
  EXPECT_FALSE(replay.service(1, [](const auto&) {return false;}));
  EXPECT_EQ(replay.phase(), e::AsyncReplay::Phase::Cancelled);
  replay.reset();
  ASSERT_TRUE(replay.start([] {return true;}));
  replay.record(e::AsyncReplay::Kind::Correct, 0, values, covariance, 13);
  ASSERT_TRUE(replay.service(1, [](const auto&) {return true;}));
  replay.reset();
  EXPECT_FALSE(replay.try_switch([] {FAIL() << "A reset must invalidate ready work";}));
}

TEST(AsyncReplay, BatchesPinQueueSlotsAndHonorTheServiceLimitWithoutAllocating)
{
  e::AsyncReplay replay;
  replay.prepare(4, 1, 1);
  const std::array<double, 1> values{{1.0}}, covariance{{1.0}};
  ASSERT_TRUE(replay.start([] {return true;}));
  for (std::uint64_t time = 1; time <= 3; ++time) {
    ASSERT_TRUE(replay.record(e::AsyncReplay::Kind::Correct, 0, values, covariance, time));
  }
  bool first_ok, second_ok, early_switch, switched, borrowed_valid = true;
  std::size_t batches = 0, events = 0;
  {
    NoAllocations guard;
    first_ok = replay.service_batch(2, [&](const e::AsyncReplay::Batch& batch) {
      ++batches; events += batch.size();
      borrowed_valid = batch.size() == 2 && batch[0].sequence == 1 && batch[1].sequence == 2;
      borrowed_valid = replay.record(e::AsyncReplay::Kind::Correct, 0, values, covariance, 4) && borrowed_valid;
      return borrowed_valid && batch[0].time == 1 && batch[1].time == 2;
    });
    early_switch = replay.try_switch([] {});
    second_ok = replay.service_batch(2, [&](const e::AsyncReplay::Batch& batch) {
      ++batches; events += batch.size();
      return batch.size() == 2 && batch[0].sequence == 3 && batch[1].sequence == 4;
    });
    switched = replay.try_switch([] {});
  }
  EXPECT_EQ(allocations, 0u);
  EXPECT_TRUE(first_ok); EXPECT_TRUE(second_ok); EXPECT_TRUE(borrowed_valid);
  EXPECT_FALSE(early_switch); EXPECT_TRUE(switched);
  EXPECT_EQ(batches, 2u); EXPECT_EQ(events, 4u);
}

TEST(AsyncReplay, SeparateOwnersPublishCompleteEventsAndSwitchOnlyAfterTheLastEvent)
{
  constexpr std::uint64_t count = 256;
  e::AsyncReplay replay;
  replay.prepare(count, 3, 9);
  ASSERT_TRUE(replay.start([] {return true;}));
  std::atomic<bool> stop{false};
  std::uint64_t applied = 0;
  bool payloads_valid = true;
  std::thread consumer([&] {
    while (!stop.load(std::memory_order_acquire)) {
      replay.service(8, [&](const auto& event) {
        ++applied;
        payloads_valid = payloads_valid && event.sequence == applied && event.time == applied &&
          event.index == 2 && event.width == 3 && event.square == 9 &&
          event.values[0] == static_cast<double>(applied) && event.covariance[8] == 1.0;
        return payloads_valid;
      });
      std::this_thread::yield();
    }
  });
  std::array<double, 3> values{};
  const std::array<double, 9> covariance{{1, 0, 0, 0, 1, 0, 0, 0, 1}};
  bool recorded = true;
  for (std::uint64_t sequence = 1; sequence <= count; ++sequence) {
    values[0] = static_cast<double>(sequence);
    recorded = replay.record(e::AsyncReplay::Kind::Correct, 2, values, covariance, sequence) && recorded;
    std::this_thread::yield();
  }
  bool switched = false;
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  while (!switched && replay.phase() != e::AsyncReplay::Phase::Cancelled &&
    std::chrono::steady_clock::now() < deadline)
  {
    switched = replay.try_switch([] {});
    std::this_thread::yield();
  }
  stop.store(true, std::memory_order_release);
  consumer.join();
  EXPECT_TRUE(recorded); EXPECT_TRUE(payloads_valid); EXPECT_TRUE(switched);
  EXPECT_EQ(applied, count);
  EXPECT_EQ(replay.phase(), e::AsyncReplay::Phase::Idle);
}

class LinearProcess final : public e::ProcessModel
{
public:
  explicit LinearProcess(Eigen::Index n, std::vector<Eigen::Index> angles = {})
  : space_(n, std::move(angles)), a(e::Matrix::Identity(n, n)) {}

  const e::VectorSpace& state_space() const override { return space_; }
  Eigen::Index input_size() const override { return 0; }
  bool predict(e::VectorIn state, e::VectorIn, e::TimeNs time, e::TimeNs dt,
    e::TimeNs, e::VectorOut next) const override
  {
    last_time = time;
    last_dt = dt;
    next.noalias() = a * state;
    return !fail_prediction;
  }
  bool linearize(e::VectorIn, e::VectorIn, e::TimeNs, e::TimeNs,
    e::TimeNs, e::MatrixOut output) const override
  {
    output = a;
    return !fail_linearization;
  }

private:
  e::VectorSpace space_;
public:
  e::Matrix a;
  bool fail_prediction{false};
  bool fail_linearization{false};
  mutable e::TimeNs last_time{0}, last_dt{0};
};

struct ReplayBatchInput
{
  bool prediction;
  e::TimeNs time, dt;
  e::Vector values;
  e::Matrix covariance;
};

template<class Replay, std::size_t Size>
e::Status apply_replay_batch(Replay& replay, const std::array<ReplayBatchInput, Size>& events)
{
  using EventInfo = typename Replay::EventInfo;
  return replay.replay_batch(events.size(), [&](std::size_t index) {
    const auto& event = events[index];
    return EventInfo{event.prediction, event.time, event.dt};
  }, [&](std::size_t index, auto& output, bool) {
    const auto& event = events[index];
    output.prediction = event.prediction; output.time = event.time;
    output.dt = event.dt; output.input_dt = 0;
    if (event.prediction) {
      output.start = event.time - event.dt;
      output.input = event.values; output.q = event.covariance;
    } else {
      output.observation = 0; output.width = event.values.size();
      output.measurement.head(output.width) = event.values;
      output.r.topLeftCorner(output.width, output.width) = event.covariance;
    }
    return e::Status::ok;
  });
}

// Same observation through the general API, without the projection capability.
class GeneralSubsetObservation final : public e::ObservationModel
{
public:
  GeneralSubsetObservation(const e::VectorSpace& state, std::vector<Eigen::Index> indices)
  : subset_(state, std::move(indices)) {}
  Eigen::Index state_size() const override {return subset_.state_size();}
  Eigen::Index input_size() const override {return 0;}
  const e::VectorSpace& observation_space() const override {return subset_.observation_space();}
  bool observe(e::VectorIn state, e::VectorIn input, e::TimeNs time, e::TimeNs input_dt,
    e::VectorOut values) const override {return subset_.observe(state, input, time, input_dt, values);}
  bool jacobian(e::VectorIn state, e::VectorIn input, e::TimeNs time, e::TimeNs input_dt,
    e::MatrixOut h) const override {return subset_.jacobian(state, input, time, input_dt, h);}
private:
  e::StateSubsetObservation subset_;
};

TEST(Estimation, StateProjectionMatchesGeneralUpdatesWithCorrelationsAndAngles)
{
  for (const Eigen::Index n : {4, 21, 40}) {
    for (const bool full : {false, true}) {
      for (const int variant : {0, 1, 2}) {
        SCOPED_TRACE(n);
        SCOPED_TRACE(full);
        SCOPED_TRACE(variant);
        LinearProcess model(n, {n - 1});
        std::vector<Eigen::Index> indices{n - 1, 0, 2};
        if (full) {
          indices.resize(static_cast<std::size_t>(n));
          for (Eigen::Index row = 0; row < n; ++row) {indices[static_cast<std::size_t>(row)] = n - 1 - row;}
        }
        const auto m = static_cast<Eigen::Index>(indices.size());
        e::StateSubsetObservation projected(model.state_space(), indices);
        GeneralSubsetObservation general(model.state_space(), indices);
        const auto make = [&](const e::ObservationModel& observation) -> std::unique_ptr<e::GaussianFilter> {
          const std::vector<e::ObservationBinding> bindings{{&observation, std::numeric_limits<double>::infinity()}};
          if (variant == 0) {return std::make_unique<e::Ekf>(model, bindings);}
          e::UkfOptions options; options.alpha = 0.5;
          options.correction = variant == 1 ? e::UkfCorrection::legacy : e::UkfCorrection::innovation;
          return std::make_unique<e::Ukf>(model, bindings, options);
        };
        auto fast = make(projected), dense = make(general);
        e::Matrix basis = e::Matrix::Zero(n, n);
        for (Eigen::Index row = 0; row < n; ++row) {
          for (Eigen::Index column = 0; column <= row; ++column) {
            basis(row, column) = row == column ? 0.25 + 0.002 * row : 0.01 * ((row + column) % 5 - 2);
          }
        }
        const e::Matrix p = basis * basis.transpose(), q = 0.001 * e::Matrix::Identity(n, n);
        const e::Matrix r = 0.02 * e::Matrix::Identity(m, m) + 0.001 * e::Matrix::Ones(m, m);
        e::Vector initial = e::Vector::LinSpaced(n, -0.2, 0.2), z(m);
        initial(n - 1) = 3.1;
        for (Eigen::Index row = 0; row < m; ++row) {
          const auto index = indices[static_cast<std::size_t>(row)];
          z(row) = index == n - 1 ? -3.1 : initial(index) + 0.1;
        }
        const e::Vector empty;
        ASSERT_EQ(fast->initialize(initial, p, 0), e::Status::ok);
        ASSERT_EQ(dense->initialize(initial, p, 0), e::Status::ok);
        for (e::TimeNs time = 10; time <= 30; time += 10) {
          ASSERT_EQ(dense->predict(empty, q, time, 10), e::Status::ok);
          ASSERT_EQ(dense->correct(0, z, r, empty, time), e::Status::ok);
          e::Status predicted, corrected;
          {
            NoAllocations guard;
            predicted = fast->predict(empty, q, time, 10);
            corrected = fast->correct(0, z, r, empty, time);
          }
          EXPECT_EQ(allocations, 0u);
          ASSERT_EQ(predicted, e::Status::ok); ASSERT_EQ(corrected, e::Status::ok);
          EXPECT_TRUE(fast->state().isApprox(dense->state(), 1e-10));
          EXPECT_TRUE(fast->covariance().isApprox(dense->covariance(), 1e-10));
          EXPECT_TRUE(fast->covariance().isApprox(fast->covariance().transpose(), 1e-14));
        }
      }
    }
  }
}

#if defined(RPP_LOCALIZATION_ENABLE_BENCHMARK_METRICS) && RPP_LOCALIZATION_ENABLE_BENCHMARK_METRICS
TEST(EstimationBenchmark, InternalCountersIncludeFailedAndSuccessfulReplayWithoutAllocating)
{
  LinearProcess model(2);
  e::StateSubsetObservation observation(model.state_space(), {0});
  const std::vector<e::ObservationBinding> bindings{{&observation, std::numeric_limits<double>::infinity()}};
  e::ExecutionMetrics counts;
  e::ProfiledEkf filter(model, bindings, &counts);
  const e::Vector empty, z = e::Vector::Constant(1, 0.1);
  const e::Matrix q = 0.1 * e::Matrix::Identity(2, 2), r = e::Matrix::Constant(1, 1, 0.2);
  ASSERT_EQ(filter.initialize(e::Vector::Zero(2), e::Matrix::Identity(2, 2), 0), e::Status::ok);
  e::ProfiledReplay replay(filter, bindings, 0, 8, e::ReplayNoise::constant_rate, &counts);
  replay.reset();
  ASSERT_EQ(replay.predict(empty, q, 20, 20, 0), e::Status::ok);
  model.fail_prediction = true;
  e::Status status;
  {
    NoAllocations guard;
    status = replay.correct(0, z, r, empty, 10, 0);
  }
  EXPECT_EQ(allocations, 0u); EXPECT_EQ(status, e::Status::model_failure);
  EXPECT_EQ(filter.time(), 20u);
  model.fail_prediction = false;
  {
    NoAllocations guard;
    status = replay.correct(0, z, r, empty, 10, 0);
    if (status == e::Status::ok) {status = replay.correct(0, z, r, empty, 5, 0);}
  }
  EXPECT_EQ(allocations, 0u); ASSERT_EQ(status, e::Status::ok);
  EXPECT_EQ(counts.core_predictions, 7u); EXPECT_EQ(counts.core_corrections, 3u);
  EXPECT_EQ(counts.projected_corrections, 3u);
  EXPECT_EQ(counts.late_measurements, 3u); EXPECT_EQ(counts.replay_attempts, 3u);
  EXPECT_EQ(counts.replay_successes, 2u); EXPECT_EQ(counts.replay_failures, 1u);
  EXPECT_EQ(counts.replayed_predictions, 6u); EXPECT_EQ(counts.replayed_corrections, 1u);
  EXPECT_EQ(counts.tail_events_total, 5u); EXPECT_EQ(counts.tail_events_max, 3u);
  EXPECT_EQ(counts.replay_states.count, 3u);
  EXPECT_NEAR(counts.replay_states.states_mean, 5.0 / 3.0, 1e-12);
  EXPECT_NEAR(counts.replay_states.states_stddev(), std::sqrt(8.0 / 9.0), 1e-12);
  EXPECT_EQ(counts.replay_states.states_max, 3u);
  EXPECT_EQ(counts.rollback_span_ns_total, 60u); EXPECT_EQ(counts.rollback_span_ns_max, 20u);
  EXPECT_EQ(filter.time(), 20u);
}
#endif

class SquareObservation final : public e::ObservationModel
{
public:
  Eigen::Index state_size() const override { return 1; }
  Eigen::Index input_size() const override { return 0; }
  const e::VectorSpace& observation_space() const override { return space_; }
  bool observe(e::VectorIn state, e::VectorIn, e::TimeNs, e::TimeNs,
    e::VectorOut values) const override
  {
    values(0) = state(0) * state(0);
    return true;
  }
  bool jacobian(e::VectorIn state, e::VectorIn, e::TimeNs, e::TimeNs,
    e::MatrixOut h) const override
  {
    ++jacobian_calls;
    h(0, 0) = 2.0 * state(0);
    return true;
  }
  mutable int jacobian_calls{0};
private:
  e::VectorSpace space_{1};
};

TEST(Estimation, RuntimeDimensionsAndDifferentMeasurementWidth)
{
  const e::Vector empty;
  for (const Eigen::Index n : {4, 21}) {
    LinearProcess model(n);
    e::StateSubsetObservation observation(model.state_space(), {0, n - 1});
    const std::vector<e::ObservationBinding> bindings{{&observation, 10.0}};
    e::Ekf ekf(model, bindings);
    e::UkfOptions options;
    options.alpha = 1.0;
    options.correction = e::UkfCorrection::innovation;
    e::Ukf ukf(model, bindings, options);
    const e::Vector state = e::Vector::Constant(n, 0.2);
    const e::Matrix p = 0.5 * e::Matrix::Identity(n, n);
    const e::Matrix q = 0.01 * e::Matrix::Identity(n, n);
    const e::Matrix r = 0.5 * e::Matrix::Identity(2, 2);
    const e::Vector z = (e::Vector(2) << 1.0, 0.6).finished();
    ASSERT_EQ(ekf.initialize(state, p, 0), e::Status::ok);
    ASSERT_EQ(ukf.initialize(state, p, 0), e::Status::ok);
    ASSERT_EQ(ekf.predict(empty, q, 100, 100), e::Status::ok);
    ASSERT_EQ(ukf.predict(empty, q, 100, 100), e::Status::ok);
    EXPECT_EQ(model.last_time, 0u);
    EXPECT_EQ(model.last_dt, 100u);
    ASSERT_EQ(ekf.correct(0, z, r, empty, 100), e::Status::ok);
    ASSERT_EQ(ukf.correct(0, z, r, empty, 100), e::Status::ok);
    EXPECT_NEAR(ekf.state()(0), 0.2 + (0.51 / 1.01) * 0.8, 1e-12);
    EXPECT_NEAR(ekf.covariance()(0, 0), 0.51 * 0.5 / 1.01, 1e-12);
    EXPECT_NEAR(ekf.state()(1), 0.2, 1e-12);
    EXPECT_TRUE(ukf.state().isApprox(ekf.state(), 1e-10));
    EXPECT_TRUE(ukf.covariance().isApprox(ekf.covariance(), 1e-10));
  }
}

TEST(Estimation, NonlinearObservationIsEvaluatedWithoutUkfJacobianCalls)
{
  LinearProcess model(1);
  SquareObservation observation;
  e::UkfOptions options;
  options.alpha = 1.0;
  options.correction = e::UkfCorrection::innovation;
  e::Ukf filter(model, {{&observation, 10.0}}, options);
  const e::Vector empty;
  ASSERT_EQ(filter.initialize(e::Vector::Constant(1, 2.0), e::Matrix::Constant(1, 1, 0.1), 0),
    e::Status::ok);
  ASSERT_EQ(filter.correct(0, e::Vector::Constant(1, 4.4),
      e::Matrix::Constant(1, 1, 0.2), empty, 0), e::Status::ok);
  EXPECT_NEAR(filter.state()(0), 2.0 + 0.4 / 1.82 * 0.3, 1e-12);
  EXPECT_NEAR(filter.covariance()(0, 0), 0.1 - 0.16 / 1.82, 1e-12);
  EXPECT_EQ(observation.jacobian_calls, 0);
}

TEST(Estimation, EkfUsesTheNonlinearObservationJacobian)
{
  LinearProcess model(1);
  SquareObservation observation;
  e::Ekf filter(model, {{&observation, 10.0}});
  const e::Vector empty;
  ASSERT_EQ(filter.initialize(e::Vector::Constant(1, 2.0),
      e::Matrix::Constant(1, 1, 0.1), 0), e::Status::ok);
  ASSERT_EQ(filter.correct(0, e::Vector::Constant(1, 4.4),
      e::Matrix::Constant(1, 1, 0.2), empty, 0), e::Status::ok);
  EXPECT_NEAR(filter.state()(0), 2.0 + 0.4 / 1.8 * 0.4, 1e-12);
  EXPECT_NEAR(filter.covariance()(0, 0), 0.1 * 0.2 / 1.8, 1e-12);
  EXPECT_EQ(observation.jacobian_calls, 1);
}

TEST(Estimation, AngleResidualUsesConfiguredCoordinates)
{
  LinearProcess model(2, {1});
  e::StateSubsetObservation observation(model.state_space(), {1});
  e::Ekf filter(model, {{&observation, 10.0}});
  const e::Vector empty;
  const e::Vector state = (e::Vector(2) << 0.0, 3.1).finished();
  ASSERT_EQ(filter.initialize(state, 0.1 * e::Matrix::Identity(2, 2), 0), e::Status::ok);
  ASSERT_EQ(filter.correct(0, e::Vector::Constant(1, -3.1),
      e::Matrix::Constant(1, 1, 0.1), empty, 0), e::Status::ok);
  EXPECT_NEAR(std::abs(filter.state()(1)), PI, 1e-12);
  EXPECT_DOUBLE_EQ(filter.state()(0), 0.0);
}

TEST(Estimation, RejectionAndModelFailurePreserveTheWholeEstimate)
{
  LinearProcess model(4);
  e::StateSubsetObservation observation(model.state_space(), {0});
  e::Ekf filter(model, {{&observation, 3.0}});
  const e::Vector empty;
  const e::Vector state = e::Vector::Ones(4);
  const e::Matrix p = e::Matrix::Identity(4, 4);
  ASSERT_EQ(filter.initialize(state, p, 10), e::Status::ok);
  EXPECT_EQ(filter.correct(0, e::Vector::Constant(1, 1e6),
      e::Matrix::Identity(1, 1), empty, 10), e::Status::gated);
  EXPECT_EQ(filter.correct(0, e::Vector::Zero(1),
      e::Matrix::Identity(1, 1), empty, 9), e::Status::stale_input);
  EXPECT_EQ(filter.predict(empty, p, 20, 9), e::Status::invalid_input);
  model.a *= 2.0;
  model.fail_linearization = true;
  EXPECT_EQ(filter.predict(empty, p, 20, 10), e::Status::model_failure);
  EXPECT_TRUE(filter.state().isApprox(state));
  EXPECT_TRUE(filter.covariance().isApprox(p));
  EXPECT_EQ(filter.time(), 10u);
}

TEST(Estimation, SemidefiniteStartupAndResetDiscardRetainedSigmaPoints)
{
  LinearProcess model(1);
  e::StateSubsetObservation observation(model.state_space(), {0});
  e::Ukf filter(model, {{&observation, 10.0}});
  e::Ukf fresh(model, {{&observation, 10.0}});
  const e::Vector empty;
  ASSERT_EQ(filter.initialize(e::Vector::Zero(1), e::Matrix::Zero(1, 1), 0), e::Status::ok);
  ASSERT_EQ(filter.predict(empty, e::Matrix::Zero(1, 1), 1, 1), e::Status::ok);
  EXPECT_TRUE(filter.state().isZero());
  EXPECT_TRUE(filter.covariance().isZero());
  ASSERT_EQ(filter.reset(e::Vector::Ones(1), e::Matrix::Identity(1, 1), 0), e::Status::ok);
  ASSERT_EQ(filter.predict(empty, e::Matrix::Constant(1, 1, 0.1), 1, 1), e::Status::ok);
  const e::Vector state = filter.state();
  const e::Matrix p = filter.covariance();
  ASSERT_EQ(filter.reset(state, p, 1), e::Status::ok);
  ASSERT_EQ(fresh.initialize(state, p, 1), e::Status::ok);
  const e::Vector z = e::Vector::Constant(1, 1.1);
  const e::Matrix r = e::Matrix::Constant(1, 1, 0.2);
  ASSERT_EQ(filter.correct(0, z, r, empty, 1), e::Status::ok);
  ASSERT_EQ(fresh.correct(0, z, r, empty, 1), e::Status::ok);
  EXPECT_TRUE(filter.state().isApprox(fresh.state(), 1e-10));
  EXPECT_TRUE(filter.covariance().isApprox(fresh.covariance(), 1e-10));
}

template<typename Filter>
void check_allocations(Filter& filter, Eigen::Index n)
{
  const e::Vector empty;
  const e::Vector state = e::Vector::Constant(n, 0.2);
  const e::Matrix p = e::Matrix::Identity(n, n);
  const e::Matrix q = 0.01 * p;
  const e::Vector z = e::Vector::Constant(1, 0.3);
  const e::Matrix r = e::Matrix::Constant(1, 1, 0.1);
  for (int iteration = 0; iteration < 3; ++iteration) {
    {
      NoAllocations guard;
      EXPECT_EQ(filter.reset(state, p, 0), e::Status::ok);
      EXPECT_EQ(filter.predict(empty, q, 100, 100), e::Status::ok);
      EXPECT_EQ(filter.correct(0, z, r, empty, 100), e::Status::ok);
      EXPECT_EQ(filter.correct(0, z, r, empty, 100), e::Status::ok);
    }
    EXPECT_EQ(allocations, 0u);
  }
}

TEST(Estimation, EkfAndUkfPrepareStorageForEveryConfiguredDimension)
{
  for (const Eigen::Index n : {1, 4, 21}) {
    LinearProcess model(n, {n - 1});
    e::StateSubsetObservation observation(model.state_space(), {n - 1});
    e::Ekf ekf(model, {{&observation, 10.0}});
    e::Ukf ukf(model, {{&observation, 10.0}});
    check_allocations(ekf, n);
    check_allocations(ukf, n);
  }
}

TEST(Estimation, RejectsIncompatibleBindingsAndBadData)
{
  LinearProcess model(4);
  SquareObservation wrong_size;
  EXPECT_THROW((e::Ekf(model, {{&wrong_size, 3.0}})), std::invalid_argument);
  EXPECT_THROW((e::StateSubsetObservation(model.state_space(), {1, 1})), std::invalid_argument);
  e::Ekf filter(model, {});
  const e::Vector empty;
  EXPECT_EQ(filter.predict(empty, e::Matrix::Identity(4, 4), 0, 0), e::Status::not_initialized);
  EXPECT_EQ(filter.initialize(e::Vector::Zero(3), e::Matrix::Identity(4, 4), 0),
    e::Status::invalid_input);
  EXPECT_EQ(filter.initialize(e::Vector::Zero(4), -e::Matrix::Identity(4, 4), 0),
    e::Status::invalid_input);
}

TEST(Estimation, ConcreteNavigationUsesPreparedControlsWithoutOwningTheEstimate)
{
  NativeNavigationModel15 model(true);
  const e::Vector state = e::Vector::Zero(STATE_SIZE);
  e::Vector input = e::Vector::Zero(9);
  input(0) = 2.0;
  input(5) = 1.0;
  input.tail(3).setOnes();
  e::Vector next(STATE_SIZE);
  e::Matrix jacobian(STATE_SIZE, STATE_SIZE);
  bool predicted = false;
  {
    NoAllocations guard;
    predicted = model.predict_and_linearize(state, input, 1'000'000'000,
      100'000'000, 0, next, jacobian);
  }
  ASSERT_TRUE(predicted);
  EXPECT_EQ(allocations, 0u);
  EXPECT_TRUE(state.isZero());
  EXPECT_NEAR(next(StateMemberX), 0.01, 1e-12);
  EXPECT_NEAR(next(StateMemberVx), 0.2, 1e-12);
  EXPECT_NEAR(next(StateMemberAx), 2.0, 1e-12);
  EXPECT_NEAR(next(StateMemberYaw), 0.01, 1e-12);
  EXPECT_NEAR(next(StateMemberVyaw), 0.1, 1e-12);
  EXPECT_TRUE(jacobian.allFinite());
}

TEST(Estimation, UkfModelFailureAndUnsignedTimeBoundariesPreserveTheEstimate)
{
  LinearProcess model(1);
  e::Ukf filter(model, {});
  const e::Vector empty;
  const e::Vector state = e::Vector::Ones(1);
  const e::Matrix p = e::Matrix::Identity(1, 1);
  const auto initial_time = std::numeric_limits<e::TimeNs>::max() - 5;
  ASSERT_EQ(filter.initialize(state, p, initial_time), e::Status::ok);
  model.fail_prediction = true;
  EXPECT_EQ(filter.predict(empty, e::Matrix::Zero(1, 1), initial_time + 5, 5),
    e::Status::model_failure);
  EXPECT_TRUE(filter.state().isApprox(state));
  EXPECT_TRUE(filter.covariance().isApprox(p));
  EXPECT_EQ(filter.time(), initial_time);
  EXPECT_EQ(filter.predict(empty, p, 0, 6), e::Status::stale_input);
}

TEST(Estimation, PreparedReplayMatchesChronologicalUpdatesAndRollsBackWithoutAllocating)
{
  for (const bool unscented : {false, true}) {
    SCOPED_TRACE(unscented);
    LinearProcess model(4);
    e::StateSubsetObservation observation(model.state_space(), {0});
    const std::vector<e::ObservationBinding> observations{{&observation, std::numeric_limits<double>::infinity()}};
    const auto make = [&]() -> std::unique_ptr<e::GaussianFilter> {
      if (unscented) {return std::make_unique<e::Ukf>(model, observations);}
      return std::make_unique<e::Ekf>(model, observations);
    };
    auto ordered = make(), delayed = make();
    const e::Vector initial = e::Vector::Zero(4), empty, z = e::Vector::Ones(1);
    const e::Matrix p = 0.5 * e::Matrix::Identity(4, 4), q = 0.1 * e::Matrix::Identity(4, 4);
    const e::Matrix twice_q = 2.0 * q, r = e::Matrix::Constant(1, 1, 0.2);
    ASSERT_EQ(ordered->initialize(initial, p, 0), e::Status::ok);
    ASSERT_EQ(delayed->initialize(initial, p, 0), e::Status::ok);
    e::Replay replay(*delayed, observations, 0, 4, e::ReplayNoise::constant_rate);
    replay.reset();
    ASSERT_EQ(ordered->predict(empty, q, 10, 10), e::Status::ok);
    ASSERT_EQ(ordered->correct(0, z, r, empty, 10), e::Status::ok);
    ASSERT_EQ(ordered->predict(empty, q, 20, 10), e::Status::ok);
    ASSERT_EQ(replay.predict(empty, twice_q, 20, 20, 0), e::Status::ok);
    const e::Vector previous_state = delayed->state(); const e::Matrix previous_p = delayed->covariance();
    model.fail_prediction = true;
    e::Status failed, accepted;
    {
      NoAllocations guard;
      failed = replay.correct(0, z, r, empty, 10, 0);
    }
    EXPECT_EQ(allocations, 0u); EXPECT_EQ(failed, e::Status::model_failure);
    EXPECT_TRUE(delayed->state().isApprox(previous_state)); EXPECT_TRUE(delayed->covariance().isApprox(previous_p));
    EXPECT_EQ(delayed->time(), 20u);
    model.fail_prediction = false;
    {
      NoAllocations guard;
      accepted = replay.correct(0, z, r, empty, 10, 0);
    }
    EXPECT_EQ(allocations, 0u); ASSERT_EQ(accepted, e::Status::ok);
    EXPECT_TRUE(delayed->state().isApprox(ordered->state(), 1e-9));
    EXPECT_TRUE(delayed->covariance().isApprox(ordered->covariance(), 1e-9));
    EXPECT_EQ(delayed->time(), 20u);
    e::Status predicted;
    {
      NoAllocations guard;
      predicted = replay.predict(empty, q, 30, 10, 0);
      predicted = predicted == e::Status::ok ? replay.predict(empty, q, 40, 10, 0) : predicted;
      predicted = predicted == e::Status::ok ? replay.predict(empty, q, 50, 10, 0) : predicted;
    }
    EXPECT_EQ(allocations, 0u); ASSERT_EQ(predicted, e::Status::ok);
    EXPECT_EQ(replay.correct(0, z, r, empty, 0, 0), e::Status::stale_input);
  }
}

TEST(Estimation, BatchedReplayMatchesChronologicalUpdatesAndRetainsHistoryWithoutAllocating)
{
  for (const bool unscented : {false, true}) {
    for (const std::size_t capacity : {4u, 16u}) {
      SCOPED_TRACE(unscented);
      SCOPED_TRACE(capacity);
      LinearProcess model(4);
      e::StateSubsetObservation observation(model.state_space(), {0});
      const std::vector<e::ObservationBinding> bindings{{&observation, std::numeric_limits<double>::infinity()}};
      const auto make = [&]() -> std::unique_ptr<e::GaussianFilter> {
        if (unscented) {return std::make_unique<e::Ukf>(model, bindings);}
        return std::make_unique<e::Ekf>(model, bindings);
      };
      auto ordered = make(), delayed = make();
      const e::Vector initial = e::Vector::Zero(4), empty;
      const e::Vector z5 = e::Vector::Constant(1, 0.5), z10 = e::Vector::Ones(1);
      const e::Vector equal_time = e::Vector::Constant(1, 0.2), z15 = e::Vector::Constant(1, 0.75);
      const e::Vector z20 = e::Vector::Constant(1, 1.5), z30 = e::Vector::Constant(1, 0.3);
      const e::Matrix p = 0.5 * e::Matrix::Identity(4, 4), q = 0.1 * e::Matrix::Identity(4, 4);
      const e::Matrix half_q = 0.5 * q, twice_q = 2.0 * q, r = e::Matrix::Constant(1, 1, 0.2);
      ASSERT_EQ(ordered->initialize(initial, p, 0), e::Status::ok);
      ASSERT_EQ(delayed->initialize(initial, p, 0), e::Status::ok);
      e::Replay expected(*ordered, bindings, 0, 16, e::ReplayNoise::constant_rate);
      e::Replay replay(*delayed, bindings, 0, capacity, e::ReplayNoise::constant_rate);
      expected.reset(); replay.reset();
      ASSERT_EQ(expected.predict(empty, half_q, 5, 5, 0), e::Status::ok);
      ASSERT_EQ(expected.correct(0, z5, r, empty, 5, 0), e::Status::ok);
      ASSERT_EQ(expected.predict(empty, half_q, 10, 5, 0), e::Status::ok);
      ASSERT_EQ(expected.correct(0, z10, r, empty, 10, 0), e::Status::ok);
      ASSERT_EQ(expected.correct(0, equal_time, r, empty, 10, 0), e::Status::ok);
      ASSERT_EQ(expected.predict(empty, half_q, 15, 5, 0), e::Status::ok);
      ASSERT_EQ(expected.correct(0, z15, r, empty, 15, 0), e::Status::ok);
      ASSERT_EQ(expected.predict(empty, half_q, 20, 5, 0), e::Status::ok);
      ASSERT_EQ(expected.correct(0, z20, r, empty, 20, 0), e::Status::ok);
      ASSERT_EQ(expected.predict(empty, q, 30, 10, 0), e::Status::ok);
      ASSERT_EQ(expected.correct(0, z30, r, empty, 30, 0), e::Status::ok);
      ASSERT_EQ(replay.predict(empty, twice_q, 20, 20, 0), e::Status::ok);
      ASSERT_EQ(replay.correct(0, z20, r, empty, 20, 0), e::Status::ok);
      // Arrival order includes late and equal-time corrections around catch-up prediction.
      const std::array<ReplayBatchInput, 6> events{{
        {false, 10, 0, z10, r}, {false, 5, 0, z5, r}, {true, 30, 10, empty, q},
        {false, 30, 0, z30, r}, {false, 15, 0, z15, r}, {false, 10, 0, equal_time, r}}};
      e::Status status;
      {
        NoAllocations guard;
        status = apply_replay_batch(replay, events);
      }
      EXPECT_EQ(allocations, 0u); ASSERT_EQ(status, e::Status::ok);
      EXPECT_EQ(delayed->time(), 30u);
      EXPECT_TRUE(delayed->state().isApprox(ordered->state(), 1e-9));
      EXPECT_TRUE(delayed->covariance().isApprox(ordered->covariance(), 1e-9));
      // The committed ring must also support the next late insertion after eviction.
      ASSERT_EQ(expected.correct(0, z5, r, empty, 20, 0), e::Status::ok);
      {
        NoAllocations guard;
        status = replay.correct(0, z5, r, empty, 20, 0);
      }
      EXPECT_EQ(allocations, 0u); ASSERT_EQ(status, e::Status::ok);
      EXPECT_TRUE(delayed->state().isApprox(ordered->state(), 1e-9));
      EXPECT_TRUE(delayed->covariance().isApprox(ordered->covariance(), 1e-9));
    }
  }
}

TEST(Estimation, BatchedReplayRollsBackFailuresAndSkipsStaleAndGatedInsertions)
{
  for (const bool unscented : {false, true}) {
    SCOPED_TRACE(unscented);
    LinearProcess model(1);
    // An extra prediction split changes this transition, exposing rejected-insertion leaks.
    model.a(0, 0) = 1.1;
    e::StateSubsetObservation observation(model.state_space(), {0});
    const std::vector<e::ObservationBinding> bindings{{&observation, 1.0}};
    const auto make = [&]() -> std::unique_ptr<e::GaussianFilter> {
      if (unscented) {return std::make_unique<e::Ukf>(model, bindings);}
      return std::make_unique<e::Ekf>(model, bindings);
    };
    auto ordered = make(), delayed = make();
    const e::Vector initial = e::Vector::Zero(1), empty, z = e::Vector::Constant(1, 0.1);
    const e::Vector outlier = e::Vector::Constant(1, 100.0);
    const e::Matrix p = e::Matrix::Identity(1, 1), q = e::Matrix::Constant(1, 1, 0.1);
    const e::Matrix twice_q = 2.0 * q, r = e::Matrix::Constant(1, 1, 0.2);
    ASSERT_EQ(ordered->initialize(initial, p, 10), e::Status::ok);
    ASSERT_EQ(delayed->initialize(initial, p, 10), e::Status::ok);
    ASSERT_EQ(ordered->predict(empty, q, 20, 10), e::Status::ok);
    ASSERT_EQ(ordered->correct(0, z, r, empty, 20), e::Status::ok);
    ASSERT_EQ(ordered->predict(empty, q, 30, 10), e::Status::ok);
    e::Replay replay(*delayed, bindings, 0, 8, e::ReplayNoise::constant_rate);
    replay.reset();
    ASSERT_EQ(replay.predict(empty, twice_q, 30, 20, 0), e::Status::ok);
    const e::Vector previous = delayed->state(); const e::Matrix previous_p = delayed->covariance();
    const std::array<ReplayBatchInput, 3> events{{
      {false, 15, 0, outlier, r}, {false, 20, 0, z, r}, {false, 0, 0, z, r}}};
    model.fail_prediction = true;
    e::Status status;
    {
      NoAllocations guard;
      status = apply_replay_batch(replay, events);
    }
    EXPECT_EQ(allocations, 0u); EXPECT_EQ(status, e::Status::model_failure);
    EXPECT_EQ(delayed->time(), 30u);
    EXPECT_TRUE(delayed->state().isApprox(previous)); EXPECT_TRUE(delayed->covariance().isApprox(previous_p));
    model.fail_prediction = false;
    {
      NoAllocations guard;
      status = apply_replay_batch(replay, events);
    }
    EXPECT_EQ(allocations, 0u); ASSERT_EQ(status, e::Status::ok);
    EXPECT_TRUE(delayed->state().isApprox(ordered->state(), 1e-9));
    EXPECT_TRUE(delayed->covariance().isApprox(ordered->covariance(), 1e-9));
  }
}

TEST(Estimation, ForkedReplayKeepsLiveStateIndependentAndRetainsCorrectedHistoryAcrossRoleChanges)
{
  for (const bool unscented : {false, true}) {
    SCOPED_TRACE(unscented);
    LinearProcess model(4);
    e::StateSubsetObservation observation(model.state_space(), {0});
    const std::vector<e::ObservationBinding> observations{{&observation, std::numeric_limits<double>::infinity()}};
    const auto make = [&]() -> std::unique_ptr<e::GaussianFilter> {
      if (unscented) {return std::make_unique<e::Ukf>(model, observations);}
      return std::make_unique<e::Ekf>(model, observations);
    };
    auto a = make(), b = make(), reference = make();
    const e::Vector initial = e::Vector::Zero(4), empty, z = e::Vector::Ones(1);
    const e::Matrix p = e::Matrix::Identity(4, 4), q = 0.1 * p, r = e::Matrix::Constant(1, 1, 0.2);
    for (auto* filter : {a.get(), b.get(), reference.get()}) {
      ASSERT_EQ(filter->initialize(initial, p, 0), e::Status::ok);
    }
    e::Replay live(*a, observations, 0, 8, e::ReplayNoise::constant_rate);
    e::Replay spare(*b, observations, 0, 8, e::ReplayNoise::constant_rate);
    e::Replay expected(*reference, observations, 0, 8, e::ReplayNoise::constant_rate);
    live.reset(); spare.reset(); expected.reset();
    ASSERT_EQ(live.predict(empty, q, 20, 20, 0), e::Status::ok);
    ASSERT_EQ(expected.predict(empty, q, 20, 20, 0), e::Status::ok);
    bool accepted = false;
    {
      NoAllocations guard;
      accepted = live.fork_to(spare) && spare.correct(0, z, r, empty, 10, 0) == e::Status::ok &&
        expected.correct(0, z, r, empty, 10, 0) == e::Status::ok &&
        live.predict(empty, q, 30, 10, 0) == e::Status::ok &&
        spare.predict(empty, q, 30, 10, 0) == e::Status::ok &&
        expected.predict(empty, q, 30, 10, 0) == e::Status::ok;
    }
    EXPECT_EQ(allocations, 0u); ASSERT_TRUE(accepted);
    EXPECT_TRUE(a->state().isZero());
    EXPECT_TRUE(b->state().isApprox(reference->state(), 1e-8));
    EXPECT_TRUE(b->covariance().isApprox(reference->covariance(), 1e-8));
    // B is now live. A receives the checkpoint/history only when the next
    // replay starts; a late sample before the first handoff must still work.
    {
      NoAllocations guard;
      accepted = spare.fork_to(live) && live.correct(0, z, r, empty, 15, 0) == e::Status::ok &&
        expected.correct(0, z, r, empty, 15, 0) == e::Status::ok;
    }
    EXPECT_EQ(allocations, 0u); ASSERT_TRUE(accepted);
    EXPECT_TRUE(a->state().isApprox(reference->state(), 1e-8));
    EXPECT_TRUE(a->covariance().isApprox(reference->covariance(), 1e-8));
    EXPECT_FALSE(live.compatible(live));
  }
}

TEST(Estimation, ReplayRejectsUndefinedNoiseSplitsAndPreservesThePosterior)
{
  LinearProcess model(1);
  e::StateSubsetObservation observation(model.state_space(), {0});
  const std::vector<e::ObservationBinding> observations{{&observation, 10.0}};
  e::Ekf filter(model, observations);
  const e::Vector empty, initial = e::Vector::Zero(1), z = e::Vector::Ones(1);
  const e::Matrix p = e::Matrix::Identity(1, 1);
  ASSERT_EQ(filter.initialize(initial, p, 0), e::Status::ok);
  e::Replay replay(filter, observations, 0, 4); replay.reset();
  ASSERT_EQ(replay.predict(empty, p, 20, 20, 0), e::Status::ok);
  const e::Matrix expected = filter.covariance();
  EXPECT_EQ(replay.correct(0, z, p, empty, 10, 0), e::Status::stale_input);
  EXPECT_TRUE(filter.state().isApprox(initial)); EXPECT_TRUE(filter.covariance().isApprox(expected));
  EXPECT_EQ(filter.time(), 20u);
}

TEST(Estimation, BatchedReplaySkipsUndefinedSplitsWithoutLosingOtherCorrections)
{
  LinearProcess model(1);
  e::StateSubsetObservation observation(model.state_space(), {0});
  const std::vector<e::ObservationBinding> bindings{{&observation, 10.0}};
  e::Ekf ordered(model, bindings), delayed(model, bindings);
  const e::Vector initial = e::Vector::Zero(1), empty, z = e::Vector::Constant(1, 0.1);
  const e::Matrix p = e::Matrix::Identity(1, 1), q = e::Matrix::Constant(1, 1, 0.1), r = q;
  ASSERT_EQ(ordered.initialize(initial, p, 0), e::Status::ok);
  ASSERT_EQ(delayed.initialize(initial, p, 0), e::Status::ok);
  ASSERT_EQ(ordered.correct(0, z, r, empty, 0), e::Status::ok);
  ASSERT_EQ(ordered.predict(empty, q, 20, 20), e::Status::ok);
  e::Replay replay(delayed, bindings, 0, 4, e::ReplayNoise::reject_split);
  replay.reset();
  ASSERT_EQ(replay.predict(empty, q, 20, 20, 0), e::Status::ok);
  const std::array<ReplayBatchInput, 2> events{{{false, 5, 0, z, r}, {false, 0, 0, z, r}}};
  e::Status status;
  {
    NoAllocations guard;
    status = apply_replay_batch(replay, events);
  }
  EXPECT_EQ(allocations, 0u); ASSERT_EQ(status, e::Status::ok);
  EXPECT_TRUE(delayed.state().isApprox(ordered.state(), 1e-12));
  EXPECT_TRUE(delayed.covariance().isApprox(ordered.covariance(), 1e-12));
  EXPECT_EQ(delayed.time(), 20u);
}

TEST(Estimation, NavigationEvaluationCacheTracksStateInputAndTimeWithoutAllocating)
{
  NativeNavigationModel15 cached(true), reference(true);
  cached.enable_evaluation_cache();
  e::Vector state = e::Vector::Zero(15), input = e::Vector::Zero(9);
  state(3) = 0.2; state(4) = -0.1; state(5) = 0.3; state(6) = 1.0;
  input(0) = 0.4; input(6) = 1.0;
  e::Vector next(15), expected(15);
  e::Matrix a(15, 15), expected_a(15, 15);
  for (const e::TimeNs dt : {100'000'000u, 200'000'000u}) {
    ASSERT_TRUE(cached.predict(state, input, 0, dt, 0, next));
    bool linearized;
    {
      NoAllocations guard;
      linearized = cached.linearize(state, input, 0, dt, 0, a);
    }
    EXPECT_EQ(allocations, 0u); ASSERT_TRUE(linearized);
    ASSERT_TRUE(reference.predict_and_linearize(state, input, 0, dt, 0, expected, expected_a));
    EXPECT_TRUE(next.isApprox(expected, 1e-12)); EXPECT_TRUE(a.isApprox(expected_a, 1e-12));
    state(6) += 0.5; input(0) += 0.2;
    ASSERT_TRUE(cached.linearize(state, input, 0, dt, 0, a));
    ASSERT_TRUE(reference.predict_and_linearize(state, input, 0, dt, 0, expected, expected_a));
    EXPECT_TRUE(a.isApprox(expected_a, 1e-12));
  }
}

TEST(Estimation, PreparedQueueTransfersCoherentSamplesBetweenThreads)
{
  e::PreparedQueue<std::array<std::uint64_t, 2>> queue;
  queue.prepare(8, [] {return std::array<std::uint64_t, 2>{};});
  std::atomic<bool> valid{true};
  constexpr std::uint64_t samples = 10'000;
  std::thread consumer([&] {
    for (std::uint64_t expected = 0; expected < samples; ++expected) {
      const std::array<std::uint64_t, 2>* value = nullptr;
      while (!(value = queue.front())) {std::this_thread::yield();}
      if ((*value)[0] != expected || (*value)[1] != expected * 17) {valid.store(false);}
      queue.pop();
    }
  });
  for (std::uint64_t sample = 0; sample < samples; ++sample) {
    std::array<std::uint64_t, 2>* value = nullptr;
    while (!(value = queue.write())) {std::this_thread::yield();}
    *value = {sample, sample * 17}; queue.commit();
  }
  consumer.join(); EXPECT_TRUE(valid.load()); EXPECT_TRUE(queue.empty());
}

TEST(Estimation, BoundedProductsHandleRectangularTailsAndTransposeWithoutAllocating)
{
  const e::Matrix a = e::Matrix::Random(67, 41), b = e::Matrix::Random(53, 41);
  const e::Matrix expected = a * b.transpose();
  e::Matrix output(67, 53);
  {
    NoAllocations guard;
    e::product(output, a, b.transpose());
    e::product(output, a, b.transpose(), true, -0.25);
  }
  EXPECT_EQ(allocations, 0u);
  EXPECT_TRUE(output.isApprox(0.75 * expected, 1e-12));
}

TEST(Estimation, LargeRuntimeDimensionsAvoidHiddenEigenPackingAllocations)
{
  for (const bool unscented : {false, true}) {
    const Eigen::Index n = unscented ? 65 : 129;
    LinearProcess model(n);
    std::vector<Eigen::Index> selected;
    for (Eigen::Index index = 0; index < 37; ++index) {selected.push_back(index);}
    e::StateSubsetObservation observation(model.state_space(), selected);
    const std::vector<e::ObservationBinding> observations{{&observation, 100.0}};
    std::unique_ptr<e::GaussianFilter> filter;
    if (unscented) {
      e::UkfOptions options; options.alpha = 1.0; options.correction = e::UkfCorrection::innovation;
      filter = std::make_unique<e::Ukf>(model, observations, options);
    } else {filter = std::make_unique<e::Ekf>(model, observations);}
    const e::Vector initial = e::Vector::Zero(n), empty, z = e::Vector::Ones(37);
    const e::Matrix p = e::Matrix::Identity(n, n), q = 0.01 * p;
    const e::Matrix r = e::Matrix::Identity(37, 37);
    ASSERT_EQ(filter->initialize(initial, p, 0), e::Status::ok);
    e::Status prediction, correction;
    {
      NoAllocations guard;
      prediction = filter->predict(empty, q, 10, 10);
      correction = filter->correct(0, z, r, empty, 10);
    }
    EXPECT_EQ(allocations, 0u);
    ASSERT_EQ(prediction, e::Status::ok); ASSERT_EQ(correction, e::Status::ok);
    EXPECT_NEAR(filter->state()(0), 1.01 / 2.01, 1e-10);
    EXPECT_NEAR(filter->covariance()(0, 0), 1.01 / 2.01, 1e-10);
  }
}

TEST(Estimation, GatedLateSampleRestoresTheUnsplitPrediction)
{
  LinearProcess model(1);
  e::StateSubsetObservation observation(model.state_space(), {0});
  const std::vector<e::ObservationBinding> observations{{&observation, 0.1}};
  e::Ekf filter(model, observations);
  const e::Vector empty, initial = e::Vector::Zero(1), z = e::Vector::Constant(1, 100.0);
  const e::Matrix p = e::Matrix::Identity(1, 1);
  ASSERT_EQ(filter.initialize(initial, p, 0), e::Status::ok);
  e::Replay replay(filter, observations, 0, 4, e::ReplayNoise::constant_rate); replay.reset();
  ASSERT_EQ(replay.predict(empty, p, 20, 20, 0), e::Status::ok);
  const e::Matrix previous = filter.covariance();
  e::Status result;
  {
    NoAllocations guard;
    result = replay.correct(0, z, p, empty, 10, 0);
  }
  EXPECT_EQ(allocations, 0u); EXPECT_EQ(result, e::Status::gated);
  EXPECT_TRUE(filter.state().isApprox(initial)); EXPECT_TRUE(filter.covariance().isApprox(previous));
  EXPECT_EQ(filter.time(), 20u);
}

}  // namespace
}  // namespace rpp_localization::test
