#pragma once

#include <cstdint>
#include <sstream>
#include <string>

namespace rpp_localization
{
struct WorkerMetricsSnapshot
{
  std::uint64_t thread_id{0};
  std::uint64_t cpu_ns{0};
  std::uint64_t allocation_calls{0};
  std::uint64_t allocated_bytes{0};
  std::uint64_t live_heap_bytes{0};
  std::uint64_t peak_live_heap_bytes{0};
  bool memory_complete{true};
};

// Optional benchmark interposer. Production processes have no allocator hooks.
#if defined(__linux__) && (defined(__GNUC__) || defined(__clang__))
extern "C" bool rpp_worker_metrics_begin() noexcept __attribute__((weak));
extern "C" bool rpp_worker_metrics_snapshot(WorkerMetricsSnapshot*) noexcept __attribute__((weak));
#endif

class WorkerMetrics
{
public:
  bool available() const noexcept
  {
#if defined(__linux__) && (defined(__GNUC__) || defined(__clang__))
    return rpp_worker_metrics_begin && rpp_worker_metrics_snapshot;
#else
    return false;
#endif
  }

  void begin() const noexcept
  {
#if defined(__linux__) && (defined(__GNUC__) || defined(__clang__))
    if (available()) {(void)rpp_worker_metrics_begin();}
#endif
  }

  // Called with the worker paused. CPU and heap counters stop at this snapshot.
  std::string json() const
  {
    WorkerMetricsSnapshot snapshot;
#if defined(__linux__) && (defined(__GNUC__) || defined(__clang__))
    if (!available() || !rpp_worker_metrics_snapshot(&snapshot)) {
      return "{\"error\":\"Worker allocation tracker unavailable or worker CPU clock unreadable\"}";
    }
#else
    return "{\"error\":\"Worker allocation tracking requires the Linux benchmark interposer\"}";
#endif
    std::ostringstream out;
    out << "{\"thread_id\":" << snapshot.thread_id
        << ",\"cpu_seconds\":" << static_cast<double>(snapshot.cpu_ns) * 1e-9
        << ",\"allocation_calls\":" << snapshot.allocation_calls
        << ",\"allocated_bytes\":" << snapshot.allocated_bytes
        << ",\"live_heap_bytes\":" << snapshot.live_heap_bytes
        << ",\"peak_live_heap_bytes\":" << snapshot.peak_live_heap_bytes
        << ",\"memory_complete\":" << (snapshot.memory_complete ? "true" : "false")
        << ",\"window\":\"worker entry through paused stats snapshot\""
        << ",\"memory_scope\":\"live requested heap bytes allocated by this worker; excludes prepared/shared buffers and stack\"}";
    return out.str();
  }
};
}  // namespace rpp_localization
