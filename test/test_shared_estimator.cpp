#include <gtest/gtest.h>
#include <kj/async-io.h>

#include <rpp_cpp/component_runtime_builder.hpp>
#include <rpp_cpp/network_utils.hpp>
#include <rpp_cpp/plugin_loader.hpp>
#include <rpp_plugin_types/rpp_interfaces/Estimator.hpp>
#include <rpp_plugin_types/rpp_interfaces/Model.hpp>
#include <rpp_localization/estimation/localization_script.hpp>
#include <rpp_localization/estimation/rpp_support.hpp>
#include <rpp_localization/estimation/replay_driver.hpp>

#include <algorithm>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <limits>
#include <optional>
#include <stdexcept>

namespace
{
namespace c = rpp_interfaces::contract;
constexpr const char* ekf_id = "33333333-3333-4333-8333-333333333333";

TEST(SharedEstimator, CovarianceCacheRevalidatesChangedCorrelationsAndInvalidValues)
{
  namespace s = rpp_localization::estimation::shared;
  s::CovarianceValidator validator(2);
  rpp_localization::estimation::Matrix matrix = rpp_localization::estimation::Matrix::Identity(2, 2);
  EXPECT_TRUE(validator.valid(matrix));
  matrix(0, 1) = matrix(1, 0) = 0.2;
  EXPECT_TRUE(validator.valid(matrix));
  EXPECT_TRUE(validator.valid(matrix));
  // The same buffer now contains an indefinite covariance with positive diagonal.
  matrix(0, 1) = matrix(1, 0) = 1.5;
  EXPECT_FALSE(validator.valid(matrix));
  matrix(0, 1) = matrix(1, 0) = 0.2;
  EXPECT_TRUE(validator.valid(matrix));
  matrix(0, 0) = -1.0;
  EXPECT_FALSE(validator.valid(matrix));
  matrix(0, 0) = 1.0; matrix(0, 1) = 0.3;
  EXPECT_FALSE(validator.valid(matrix));
  matrix(0, 1) = 0.2; matrix(0, 0) = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(validator.valid(matrix));
  matrix.setZero();
  EXPECT_TRUE(validator.valid(matrix));
}

rpp::List<double> values(std::size_t size, double value = 0.0)
{
  rpp::List<double> result;
  result.resize(size);
  for (std::size_t index = 0; index < size; ++index) {result[index] = value;}
  return result;
}

rpp::List<double> covariance(std::size_t size, double diagonal)
{
  auto result = values(size * size);
  for (std::size_t index = 0; index < size; ++index) {result[index * size + index] = diagonal;}
  return result;
}

#if defined(RPP_LOCALIZATION_ENABLE_BENCHMARK_METRICS) && RPP_LOCALIZATION_ENABLE_BENCHMARK_METRICS
struct TemporaryBenchmarkInstance
{
  std::filesystem::path root;
  TemporaryBenchmarkInstance(const std::filesystem::path& source, const std::string& old_plugin,
    const std::string& benchmark_plugin)
  : root(std::filesystem::temp_directory_path() / ("rpp_localization_benchmark_" +
      std::to_string(std::chrono::steady_clock::now().time_since_epoch().count())))
  {
    try {
      std::filesystem::copy(source, root, std::filesystem::copy_options::recursive);
      const auto from = '"' + old_plugin + '"', to = '"' + benchmark_plugin + '"';
      bool replaced = false;
      for (const auto& entry : std::filesystem::recursive_directory_iterator(root)) {
        if (!entry.is_regular_file() || entry.path().extension() != ".json") {continue;}
        std::ifstream input(entry.path());
        std::string text((std::istreambuf_iterator<char>(input)), std::istreambuf_iterator<char>());
        input.close();
        std::size_t position = 0; bool changed = false;
        while ((position = text.find(from, position)) != std::string::npos) {
          text.replace(position, from.size(), to); position += to.size(); changed = replaced = true;
        }
        if (changed) {std::ofstream output(entry.path()); output << text;}
      }
      if (!replaced) {throw std::runtime_error("Benchmark profile has no estimator identity");}
    } catch (...) {
      std::error_code ignored; std::filesystem::remove_all(root, ignored); throw;
    }
  }
  ~TemporaryBenchmarkInstance() {std::error_code ignored; std::filesystem::remove_all(root, ignored);}
};

TEST(SharedEstimatorBenchmark, MetricsPolicyIsFixedInTheRegisteredEstimatorBase)
{
  for (const std::string configuration : {"ekf", "ukf"}) {
    SCOPED_TRACE(configuration);
    const bool ekf = configuration == "ekf";
    const std::string plugin = ekf ? "Ekf" : "Ukf";
    const std::string benchmark = "rpp_localization::Profiled" + plugin;
    const auto source = std::filesystem::path(RPP_LOCALIZATION_SOURCE_DIR) / ".rppws/parts" /
      (ekf ? "rpp_localization__ekf" : "rpp_localization__ukf") /
      (ekf ? ekf_id : "44444444-4444-4444-8444-444444444444");
    TemporaryBenchmarkInstance instance(source, "rpp_localization::" + plugin, benchmark);
    rpp::RppDataManager manager(rpp::RPP_HOME, RPP_LOCALIZATION_SOURCE_DIR);
    rpp::ComponentRuntimeBuilder builder(manager, rpp::ClockOptions(rpp::RPP_CLOCK_MOCK));
    auto runtime = builder.build_component({instance.root.string(), benchmark});
    ASSERT_TRUE(runtime.configure().success);
    auto estimator = runtime.get_instance<rpp_interfaces::Estimator>();
    const auto* shared = dynamic_cast<const rpp_localization::estimation::SharedEstimatorBase<true>*>(estimator.get());
    ASSERT_NE(shared, nullptr);
    auto initial = values(15), p = covariance(15, 0.1), q = covariance(15, 0.0);
    auto r = covariance(15, 0.1), empty = values(0);
    ASSERT_EQ(estimator->initialize(initial, p, 0), c::CODE_OK);
    ASSERT_TRUE(runtime.activate().success);
    ASSERT_EQ(estimator->predict(empty, q, 0, 0, 0), c::CODE_OK);
    ASSERT_EQ(estimator->correct(0, initial, r, 0), c::CODE_OK);
    const auto counts = shared->execution_metrics();
    ASSERT_TRUE(counts.has_value());
    EXPECT_EQ(counts->core_predictions, 1u); EXPECT_EQ(counts->core_corrections, 1u);
    EXPECT_EQ(counts->projected_corrections, 1u);
    EXPECT_EQ(counts->covariance_diagonal_checks, 3u);
    EXPECT_EQ(counts->covariance_factorizations, 0u);
    ASSERT_TRUE(runtime.deactivate().success);
    ASSERT_TRUE(runtime.cleanup().success);
  }
}

TEST(SharedEstimatorBenchmark, ReplayCountersSurviveRoleChangesAndCancellation)
{
  namespace e = rpp_localization::estimation;
  for (const std::string configuration : {"ekf", "ukf"}) {
    SCOPED_TRACE(configuration);
    const bool ekf = configuration == "ekf";
    const std::string plugin = "rpp_localization::" + std::string(ekf ? "Ekf" : "Ukf");
    const std::string benchmark = "rpp_localization::Profiled" + std::string(ekf ? "Ekf" : "Ukf");
    const auto source = std::filesystem::path(RPP_LOCALIZATION_SOURCE_DIR) / ".rppws/parts" /
      (ekf ? "rpp_localization__ekf" : "rpp_localization__ukf") /
      (ekf ? ekf_id : "44444444-4444-4444-8444-444444444444");
    TemporaryBenchmarkInstance instance(source, plugin, benchmark);
    rpp::RppDataManager manager(rpp::RPP_HOME, RPP_LOCALIZATION_SOURCE_DIR);
    rpp::ConcurrentComponentRuntimeBuilder builder(manager);
    const rpp::ComponentRequest request{instance.root.string(), benchmark};
    auto runtime = builder.build_component(request);
    ASSERT_TRUE(runtime.configure().success);
    auto estimator = runtime.get_instance<rpp_interfaces::Estimator>();
    auto* shared = dynamic_cast<e::SharedEstimatorBase<true>*>(estimator.get());
    ASSERT_NE(shared, nullptr);
    shared->prepare_replica(manager, request, 16);
    auto initial = values(15), p = covariance(15, 0.01), q = covariance(15, 0.0);
    auto measurement = values(15), r = covariance(15, 0.1), empty = values(0);
    measurement[0] = 1.0;
    ASSERT_EQ(estimator->initialize(initial, p, 10), c::CODE_OK);
    ASSERT_TRUE(runtime.activate().success);
    ASSERT_EQ(estimator->predict(empty, q, 20, 10, 0), c::CODE_OK);
    ASSERT_EQ(estimator->predict(empty, q, 30, 10, 0), c::CODE_OK);
    ASSERT_EQ(estimator->correct(0, measurement, r, 20), c::CODE_LIMITED);
    ASSERT_EQ(estimator->correct(0, measurement, r, 20), c::CODE_LIMITED);
    ASSERT_EQ(estimator->correct(0, measurement, r, 0), c::CODE_LIMITED);
    ASSERT_TRUE(shared->service_replay(16)); ASSERT_TRUE(shared->adopt_replay());
    const auto first_batch = shared->execution_metrics();
    ASSERT_TRUE(first_batch.has_value());
    EXPECT_EQ(first_batch->replay_jobs_started, 1u);
    EXPECT_EQ(first_batch->replay_attempts, 1u);
    EXPECT_EQ(first_batch->replay_states.count, 1u);
    const double corrected = std::get<0>(estimator->estimate())[0];
    EXPECT_GT(corrected, 0.01);
    ASSERT_EQ(estimator->correct(0, measurement, r, 20), c::CODE_LIMITED);
    ASSERT_TRUE(shared->service_replay(16));
    ASSERT_TRUE(runtime.deactivate().success);
    EXPECT_FALSE(shared->adopt_replay());
    const auto counts = shared->execution_metrics();
    ASSERT_TRUE(counts.has_value());
    EXPECT_EQ(counts->replay_jobs_started, 2u);
    EXPECT_EQ(counts->replay_role_switches, 1u);
    EXPECT_EQ(counts->replay_jobs_cancelled, 1u);
    EXPECT_EQ(counts->replay_stale_corrections, 1u);
    EXPECT_EQ(counts->replay_states.count, 2u);
    EXPECT_FALSE(counts->replay_pending);
    ASSERT_TRUE(runtime.activate().success);
    EXPECT_NEAR(std::get<0>(estimator->estimate())[0], corrected, 1e-12);
    ASSERT_TRUE(runtime.deactivate().success); ASSERT_TRUE(runtime.cleanup().success);
  }
}
#endif

TEST(SharedEstimator, ScriptRoutesReorderedMeasurementsAndTheirCovarianceForEitherPlugin)
{
  for (const std::string configuration : {"ekf", "ukf"}) {
    SCOPED_TRACE(configuration);
    rpp::RppDataManager manager(rpp::RPP_HOME, RPP_LOCALIZATION_SOURCE_DIR);
    rpp::ComponentRuntimeBuilder builder(manager, rpp::ClockOptions(rpp::RPP_CLOCK_MOCK));
    auto runtime = builder.build_script({rpp::LibraryScript{"rpp_localization", "localization_generic"},
      std::optional<std::string>{configuration}, std::nullopt});
    ASSERT_TRUE(runtime.configure().success);
    const auto& child = runtime.get_context().get_subcomponent_context("estimator");
    auto estimator = child.get_instance<rpp_interfaces::Estimator>();
    auto description = estimator->getDesc().as_struct();
    ASSERT_EQ(description.state.fields.size(), 15u);
    ASSERT_EQ(description.measurements.size(), 1u);
    auto fields = description.measurements[0].fields;
    std::reverse(fields.begin(), fields.end());
    rpp_localization::GenericLocalizationScript script(runtime.get_context());
    script.prepare({}, {rpp_interfaces::FieldLayout(fields)});
    auto initial = values(15);
    initial[6] = 1.0;
    auto p = covariance(15, 0.01);
    ASSERT_EQ(script.initialize(initial, p, 0), c::CODE_OK);
    EXPECT_EQ(estimator->initialize(initial, p, 0), c::CODE_REJECTED);
    auto empty = values(0);
    auto q = covariance(15, 0.0);
    ASSERT_EQ(script.predict(empty, q, 0, 0, 123), c::CODE_OK);
    auto [predicted, predicted_p, time, status] = script.estimate();
    ASSERT_EQ(status, c::CODE_OK);
    auto measurement = values(15);
    auto r = covariance(15, 0.0);
    for (std::size_t index = 0; index < 15; ++index) {
      measurement[index] = predicted[14 - index];
      r[index * 15 + index] = 0.1 + 0.01 * index;
    }
    measurement[14] = 1.0;  // x is last in the source layout.
    ASSERT_EQ(script.correct(0, measurement, r, 0), c::CODE_OK);
    auto [state, result_p, result_time, result_status] = script.estimate();
    ASSERT_EQ(result_status, c::CODE_OK);
    EXPECT_NEAR(state[0], 0.01 / (0.01 + 0.24), 1e-7);
    EXPECT_EQ(result_time, 0u);
    EXPECT_EQ(result_p.size(), 225u);
    EXPECT_EQ(script.correct(7, measurement, r, 0), c::CODE_INVALID_INPUT);
    ASSERT_TRUE(child.deactivate().success);
    ASSERT_TRUE(runtime.cleanup().success);
  }
}

TEST(SharedEstimator, OneScriptUsesAStableEstimatorWhileItsReplicaReplaysAndChangesRoles)
{
  for (const std::string configuration : {"ekf", "ukf"}) {
    SCOPED_TRACE(configuration);
    rpp::RppDataManager manager(rpp::RPP_HOME, RPP_LOCALIZATION_SOURCE_DIR);
    rpp::ConcurrentComponentRuntimeBuilder builder(manager);
    auto runtime = builder.build_script({rpp::LibraryScript{"rpp_localization", "localization_generic"},
      std::optional<std::string>{configuration}, std::nullopt});
    ASSERT_TRUE(runtime.configure().success);
    auto estimator = runtime.get_context().get_subcomponent_context("estimator").get_instance<rpp_interfaces::Estimator>();
    rpp_localization::estimation::ReplayDriver replay;
    replay.prepare(runtime, manager, true, 16);
    rpp_localization::GenericLocalizationScript script(runtime.get_context());
    script.prepare();
    auto initial = values(15); initial[6] = 1.0;
    auto p = covariance(15, 0.01), q = covariance(15, 0.0), r = covariance(15, 0.1);
    auto empty = values(0), measurement = values(15); measurement[0] = 1.0;
    ASSERT_EQ(script.initialize(initial, p, 0), c::CODE_OK);
    ASSERT_EQ(script.predict(empty, q, 10, 10, 0), c::CODE_OK);
    ASSERT_EQ(script.predict(empty, q, 20, 10, 0), c::CODE_OK);
    const double before = std::get<0>(script.estimate())[0];
    EXPECT_EQ(replay.phase(), rpp_localization::estimation::AsyncReplay::Phase::Idle);
    ASSERT_EQ(script.correct(0, measurement, r, 10), c::CODE_LIMITED);
    EXPECT_DOUBLE_EQ(std::get<0>(script.estimate())[0], before);
    ASSERT_TRUE(replay.service(16));
    // Live execution continues while the repaired candidate is ready. It must
    // consume the same next prediction before the role switch is accepted.
    ASSERT_EQ(script.predict(empty, q, 30, 10, 0), c::CODE_OK);
    EXPECT_FALSE(replay.adopt());
    ASSERT_TRUE(replay.service(16)); ASSERT_TRUE(replay.adopt());
    auto [state, result_p, time, code] = script.estimate();
    ASSERT_EQ(code, c::CODE_OK); EXPECT_EQ(time, 30u); EXPECT_GT(state[0], before + 0.01);
    EXPECT_EQ(runtime.get_context().get_subcomponent_context("estimator").get_instance<rpp_interfaces::Estimator>().get(), estimator.get());
    EXPECT_EQ(replay.phase(), rpp_localization::estimation::AsyncReplay::Phase::Idle);
    const double corrected = state[0];
    ASSERT_TRUE(script.deactivate()); ASSERT_TRUE(script.activate());
    EXPECT_NEAR(std::get<0>(script.estimate())[0], corrected, 1e-12);
    // The other physical instance is reused on the following late measurement.
    ASSERT_EQ(script.correct(0, measurement, r, 10), c::CODE_LIMITED);
    ASSERT_TRUE(replay.service(16)); ASSERT_TRUE(replay.adopt());
    ASSERT_TRUE(script.deactivate());
    EXPECT_EQ(replay.phase(), rpp_localization::estimation::AsyncReplay::Phase::Idle);
    EXPECT_EQ(script.predict(empty, q, 40, 10, 0), c::CODE_NOT_READY);
    ASSERT_TRUE(script.activate());
    ASSERT_EQ(script.predict(empty, q, 40, 10, 0), c::CODE_OK);
    ASSERT_EQ(script.correct(0, measurement, r, 30), c::CODE_LIMITED);
    ASSERT_TRUE(replay.service(16));
    ASSERT_EQ(script.reset(initial, p, 0, true), c::CODE_OK);
    EXPECT_EQ(replay.phase(), rpp_localization::estimation::AsyncReplay::Phase::Idle);
    EXPECT_FALSE(replay.adopt());
    EXPECT_EQ(std::get<2>(script.estimate()), 0u);
    ASSERT_TRUE(script.deactivate());
    ASSERT_TRUE(runtime.cleanup().success);
  }
}

TEST(SharedEstimator, RosReplayModeDoesNotConstructAReplicaWhenHistoryIsDisabled)
{
  namespace e = rpp_localization::estimation;
  for (const std::string configuration : {"ekf", "ukf"}) {
    SCOPED_TRACE(configuration);
    const std::string plugin = configuration == "ekf" ? "rpp_localization::Ekf" : "rpp_localization::Ukf";
    rpp::RppDataManager manager(rpp::RPP_HOME, RPP_LOCALIZATION_SOURCE_DIR);
    rpp::ConcurrentComponentRuntimeBuilder builder(manager);
    builder.set_parameter_overrides({{plugin, {{"history_capacity", std::int64_t{0}}}}});
    auto runtime = builder.build_script({rpp::LibraryScript{"rpp_localization", "localization_generic"},
      std::optional<std::string>{configuration}, std::nullopt});
    ASSERT_TRUE(runtime.configure().success);
    auto estimator = runtime.get_context().get_subcomponent_context("estimator").get_instance<rpp_interfaces::Estimator>();
    const auto* shared = dynamic_cast<const e::SharedEstimatorBase<false>*>(estimator.get());
    ASSERT_NE(shared, nullptr); EXPECT_FALSE(shared->replay_enabled());
    e::ReplayDriver replay;
    ASSERT_NO_THROW(replay.prepare(runtime, manager, true, 16));
    rpp_localization::GenericLocalizationScript script(runtime.get_context());
    script.prepare();
    auto initial = values(15), p = covariance(15, 0.01), q = covariance(15, 0.0);
    auto r = covariance(15, 0.1), empty = values(0);
    ASSERT_EQ(script.initialize(initial, p, 0), c::CODE_OK);
    ASSERT_EQ(script.predict(empty, q, 20, 20, 0), c::CODE_OK);
    EXPECT_EQ(script.correct(0, initial, r, 10), c::CODE_STALE_INPUT);
    EXPECT_EQ(replay.phase(), e::AsyncReplay::Phase::Idle);
    EXPECT_FALSE(replay.adopt()); EXPECT_FALSE(replay.metrics().has_value());
    ASSERT_TRUE(script.deactivate()); ASSERT_TRUE(runtime.cleanup().success);
  }
}

TEST(SharedEstimator, StaleSamplesDoNotDiscardARepairedCandidateAndDeactivationCancelsIt)
{
  namespace e = rpp_localization::estimation;
  for (const std::string configuration : {"ekf", "ukf"}) {
    SCOPED_TRACE(configuration);
    rpp::RppDataManager manager(rpp::RPP_HOME, RPP_LOCALIZATION_SOURCE_DIR);
    rpp::ConcurrentComponentRuntimeBuilder builder(manager);
    auto runtime = builder.build_script({rpp::LibraryScript{"rpp_localization", "localization_generic"},
      std::optional<std::string>{configuration}, std::nullopt});
    ASSERT_TRUE(runtime.configure().success);
    e::ReplayDriver replay; replay.prepare(runtime, manager, true, 16);
    rpp_localization::GenericLocalizationScript script(runtime.get_context());
    script.prepare();
    auto initial = values(15), p = covariance(15, 0.01), q = covariance(15, 0.0);
    auto measurement = values(15), r = covariance(15, 0.1), empty = values(0);
    measurement[0] = 1.0;
    ASSERT_EQ(script.initialize(initial, p, 10), c::CODE_OK);
    ASSERT_EQ(script.predict(empty, q, 20, 10, 0), c::CODE_OK);
    ASSERT_EQ(script.predict(empty, q, 30, 10, 0), c::CODE_OK);
    // A known-stale sample cannot fork away the valid retained history.
    EXPECT_EQ(script.correct(0, measurement, r, 0), c::CODE_STALE_INPUT);
    EXPECT_EQ(replay.phase(), e::AsyncReplay::Phase::Idle);
    ASSERT_EQ(script.correct(0, measurement, r, 20), c::CODE_LIMITED);
    // While replay is active, only its owner can inspect the older bank.
    ASSERT_EQ(script.correct(0, measurement, r, 0), c::CODE_LIMITED);
    ASSERT_TRUE(replay.service(16)); ASSERT_TRUE(replay.adopt());
    const double corrected = std::get<0>(script.estimate())[0];
    EXPECT_GT(corrected, 0.01);
    ASSERT_EQ(script.correct(0, measurement, r, 20), c::CODE_LIMITED);
    ASSERT_TRUE(script.deactivate());
    EXPECT_EQ(replay.phase(), e::AsyncReplay::Phase::Idle);
    EXPECT_FALSE(replay.adopt());
    ASSERT_TRUE(script.activate());
    EXPECT_NEAR(std::get<0>(script.estimate())[0], corrected, 1e-12);
    ASSERT_TRUE(script.deactivate()); ASSERT_TRUE(runtime.cleanup().success);
  }
}

TEST(SharedEstimator, RetainedSnapshotsRemainStableAndPoolExhaustionIsExplicit)
{
  rpp::RppDataManager manager(rpp::RPP_HOME, RPP_LOCALIZATION_SOURCE_DIR);
  rpp::ComponentRuntimeBuilder builder(manager);
  const std::string path = std::string(RPP_LOCALIZATION_SOURCE_DIR) +
    "/.rppws/parts/rpp_localization__ekf/" + ekf_id;
  auto runtime = builder.build_component({path, "rpp_localization::Ekf"});
  ASSERT_TRUE(runtime.configure().success);
  auto estimator = runtime.get_instance<rpp_interfaces::Estimator>();
  auto initial = values(15);
  initial[6] = 1.0;
  auto p = covariance(15, 0.01);
  ASSERT_EQ(estimator->initialize(initial, p, 0), c::CODE_OK);
  ASSERT_TRUE(runtime.activate().success);
  using Snapshot = decltype(estimator->estimate());
  std::optional<Snapshot> first(estimator->estimate());
  std::optional<Snapshot> second(estimator->estimate());
  std::optional<Snapshot> third(estimator->estimate());
  EXPECT_EQ(std::get<3>(*first), c::CODE_OK);
  EXPECT_EQ(std::get<3>(estimator->estimate()), c::CODE_BUDGET_EXCEEDED);
  auto empty = values(0);
  auto q = covariance(15, 0.0);
  ASSERT_EQ(estimator->predict(empty, q, 100'000'000, 100'000'000, 0), c::CODE_OK);
  EXPECT_DOUBLE_EQ(std::get<0>(*first)[0], 0.0);
  second.reset();
  auto current = estimator->estimate();
  ASSERT_EQ(std::get<3>(current), c::CODE_OK);
  EXPECT_NEAR(std::get<0>(current)[0], 0.1, 1e-12);
  ASSERT_TRUE(runtime.deactivate().success);
  ASSERT_TRUE(runtime.cleanup().success);
  EXPECT_DOUBLE_EQ(std::get<0>(*first)[0], 0.0);
  EXPECT_NEAR(std::get<0>(current)[0], 0.1, 1e-12);
}

TEST(SharedEstimator, RegisteredRpcAdaptersReachTheModelAndOwnedCore)
{
  auto io = kj::setupAsyncIo();
  rpp::RppDataManager manager(rpp::RPP_HOME, RPP_LOCALIZATION_SOURCE_DIR);
  rpp::ComponentRuntimeBuilder builder(manager);
  const std::string path = std::string(RPP_LOCALIZATION_SOURCE_DIR) +
    "/.rppws/parts/rpp_localization__ekf/" + ekf_id;
  auto runtime = builder.build_component({path, "rpp_localization::Ekf"});
  ASSERT_TRUE(runtime.configure().success);
  auto estimator = runtime.get_instance<rpp_interfaces::Estimator>();
  const auto info = manager.get_plugin_info_from_lib("rpp_localization::Ekf");
  auto server = rpp::load_plugin_adapter_server(info, estimator);
  const auto port = ::get_available_port();
  server->start_adapter_server__(io, "127.0.0.1", port);
  {
    rpp::RppRuntimeClientContext connection("127.0.0.1", port, std::chrono::seconds(5), &io);
    auto adapter = rpp::load_plugin_adapter_client(info);
    ASSERT_TRUE(adapter->connect_adapter_client__(connection));
    auto* client = dynamic_cast<rpp_interfaces::Estimator*>(adapter.get());
    ASSERT_NE(client, nullptr);
    EXPECT_EQ(client->getDesc().as_struct().state.fields.size(), 15u);
    auto state = values(15);
    state[6] = 1.0;
    auto p = covariance(15, 0.01);
    ASSERT_EQ(client->initialize(state, p, 0), c::CODE_OK);
    ASSERT_TRUE(runtime.activate().success);
    auto empty = values(0);
    auto q = covariance(15, 0.0);
    ASSERT_EQ(client->predict(empty, q, 100'000'000, 100'000'000, 1'000'000), c::CODE_OK);
    auto [estimate, result_p, time, status] = client->estimate();
    ASSERT_EQ(status, c::CODE_OK);
    EXPECT_NEAR(estimate[0], 0.1, 1e-12);
    EXPECT_EQ(result_p.size(), 225u);
    EXPECT_EQ(time, 100'000'000u);
  }
  server->close_adapter_server__();
  ASSERT_TRUE(runtime.deactivate().success);
  ASSERT_TRUE(runtime.cleanup().success);
}

TEST(SharedModel, RegisteredRpcAdaptersExposePredictionAndRowMajorDerivatives)
{
  auto io = kj::setupAsyncIo();
  rpp::RppDataManager manager(rpp::RPP_HOME, RPP_LOCALIZATION_SOURCE_DIR);
  rpp::ComponentRuntimeBuilder builder(manager);
  const std::string path = std::string(RPP_LOCALIZATION_SOURCE_DIR) +
    "/.rppws/parts/rpp_localization__ekf/" + ekf_id +
    "/subcomponents/33333333-3333-4333-8333-333333333334";
  auto runtime = builder.build_component({path, "rpp_localization::NavigationModel"});
  ASSERT_TRUE(runtime.configure().success);
  auto model = runtime.get_instance<rpp_interfaces::Model>();
  const auto info = manager.get_plugin_info_from_lib("rpp_localization::NavigationModel");
  auto server = rpp::load_plugin_adapter_server(info, model);
  const auto port = ::get_available_port();
  server->start_adapter_server__(io, "127.0.0.1", port);
  ASSERT_TRUE(runtime.activate().success);
  {
    rpp::RppRuntimeClientContext connection("127.0.0.1", port, std::chrono::seconds(5), &io);
    auto adapter = rpp::load_plugin_adapter_client(info);
    ASSERT_TRUE(adapter->connect_adapter_client__(connection));
    auto* client = dynamic_cast<rpp_interfaces::Model*>(adapter.get());
    ASSERT_NE(client, nullptr);
    const auto description = client->getDesc().as_struct();
    EXPECT_EQ(description.prediction.fields.size(), 15u);
    EXPECT_EQ(description.inputs.fields.size(), 0u);
    auto state = values(15);
    state[6] = 1.0;
    auto empty = values(0);
    auto [next, status] = client->predict(state, empty, 0, 100'000'000, 0);
    ASSERT_EQ(status, c::CODE_OK);
    EXPECT_NEAR(next[0], 0.1, 1e-12);
    auto [a, b, derivative_status] = client->linearize(state, empty, 0, 100'000'000, 0);
    ASSERT_EQ(derivative_status, c::CODE_OK);
    ASSERT_EQ(a.size(), 225u);
    EXPECT_NEAR(a[6], 0.1, 1e-12);  // dx/dvx in row-major order.
    EXPECT_EQ(b.size(), 0u);
  }
  server->close_adapter_server__();
  ASSERT_TRUE(runtime.deactivate().success);
  ASSERT_TRUE(runtime.cleanup().success);
}
}  // namespace
