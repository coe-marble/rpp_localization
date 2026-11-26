#pragma once

#include <cstdint>
#include <sstream>
#include <string>
#include <rpp_localization/estimation/replay_statistics.hpp>

namespace rpp_localization::estimation
{
// Optional, single-owner counters. Failed/gated calls and failed replay work
// count as execution too; restoring a posterior never restores these counters.
struct ExecutionMetrics
{
  std::uint64_t core_predictions{0}, core_corrections{0}, projected_corrections{0};
  std::uint64_t late_measurements{0}, replay_attempts{0}, replay_successes{0}, replay_failures{0};
  std::uint64_t replayed_predictions{0}, replayed_corrections{0};
  std::uint64_t tail_events_total{0}, tail_events_max{0};
  ReplayStatistics replay_states;
  std::uint64_t rollback_span_ns_total{0}, rollback_span_ns_max{0};
  std::uint64_t covariance_diagonal_checks{0}, covariance_cache_hits{0}, covariance_factorizations{0};
  std::uint64_t replay_jobs_started{0}, replay_role_switches{0}, replay_jobs_cancelled{0};
  std::uint64_t replay_stale_corrections{0};
  bool replay_pending{false};

  void merge(const ExecutionMetrics& other) noexcept
  {
    core_predictions += other.core_predictions; core_corrections += other.core_corrections;
    projected_corrections += other.projected_corrections; late_measurements += other.late_measurements;
    replay_attempts += other.replay_attempts; replay_successes += other.replay_successes;
    replay_failures += other.replay_failures; replayed_predictions += other.replayed_predictions;
    replayed_corrections += other.replayed_corrections; tail_events_total += other.tail_events_total;
    tail_events_max = std::max(tail_events_max, other.tail_events_max);
    rollback_span_ns_total += other.rollback_span_ns_total;
    rollback_span_ns_max = std::max(rollback_span_ns_max, other.rollback_span_ns_max);
    covariance_diagonal_checks += other.covariance_diagonal_checks;
    covariance_cache_hits += other.covariance_cache_hits; covariance_factorizations += other.covariance_factorizations;
    replay_states.merge(other.replay_states);
    replay_jobs_started += other.replay_jobs_started; replay_role_switches += other.replay_role_switches;
    replay_jobs_cancelled += other.replay_jobs_cancelled;
    replay_stale_corrections += other.replay_stale_corrections;
    replay_pending = replay_pending || other.replay_pending;
  }
};

inline std::string execution_metrics_json(const ExecutionMetrics& value)
{
  std::ostringstream out;
  out << "{\"core_predictions\":" << value.core_predictions
      << ",\"core_corrections\":" << value.core_corrections
      << ",\"projected_corrections\":" << value.projected_corrections
      << ",\"late_measurements\":" << value.late_measurements
      << ",\"replay_attempts\":" << value.replay_attempts
      << ",\"replay_successes\":" << value.replay_successes
      << ",\"replay_failures\":" << value.replay_failures
      << ",\"replayed_predictions\":" << value.replayed_predictions
      << ",\"replayed_corrections\":" << value.replayed_corrections
      << ",\"tail_events_total\":" << value.tail_events_total
      << ",\"tail_events_max\":" << value.tail_events_max
      << ",\"replay_states\":" << value.replay_states.json()
      << ",\"rollback_span_ns_total\":" << value.rollback_span_ns_total
      << ",\"rollback_span_ns_max\":" << value.rollback_span_ns_max
      << ",\"covariance_diagonal_checks\":" << value.covariance_diagonal_checks
      << ",\"covariance_cache_hits\":" << value.covariance_cache_hits
      << ",\"covariance_factorizations\":" << value.covariance_factorizations
      << ",\"replay_jobs_started\":" << value.replay_jobs_started
      << ",\"replay_role_switches\":" << value.replay_role_switches
      << ",\"replay_jobs_cancelled\":" << value.replay_jobs_cancelled
      << ",\"replay_stale_corrections\":" << value.replay_stale_corrections
      << ",\"replay_pending\":" << (value.replay_pending ? "true" : "false") << '}';
  return out.str();
}
}  // namespace rpp_localization::estimation
