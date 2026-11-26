#pragma once

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <ctime>
#include <ostream>
#include <sstream>
#include <string>
#include <rpp_localization/estimation/replay_statistics.hpp>

namespace rpp_localization
{
#ifndef RPP_LOCALIZATION_ENABLE_BENCHMARK_METRICS
#define RPP_LOCALIZATION_ENABLE_BENCHMARK_METRICS 0
#endif
inline constexpr bool benchmark_metrics_compiled = RPP_LOCALIZATION_ENABLE_BENCHMARK_METRICS != 0;
// Optional benchmark instrumentation. Each operation has one execution owner:
// the generic worker owns Estimation/Predict/Correct and the ROS executor owns
// Replay. Legacy operations all belong to its ROS executor. The stats service
// quiesces the worker and serializes between ROS callbacks. Recording neither
// allocates nor locks. Histograms give upper bounds within 6.25% above 16 ns.
class BenchmarkTiming
{
public:
  enum class Operation : std::size_t {Estimation, Predict, Correct, Replay};

  void configure(bool enabled, std::uint64_t budget_ns)
  {
    enabled_ = benchmark_metrics_compiled && enabled;
    budget_ns_ = budget_ns;
  }

  bool enabled() const noexcept {return benchmark_metrics_compiled && enabled_;}

  static std::uint64_t wall_now() noexcept
  {
    return static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count());
  }

  class Scope
  {
  public:
    Scope(BenchmarkTiming& owner, Operation operation, bool active = true) noexcept
    : owner_(nullptr), operation_(operation)
    {
      if constexpr (benchmark_metrics_compiled) {
        owner_ = owner.enabled_ && active ? &owner : nullptr;
        if (owner_) {wall_ = wall_now(); cpu_ = cpu_now();}
      }
    }
    Scope(const Scope&) = delete;
    Scope& operator=(const Scope&) = delete;
    ~Scope()
    {
      if constexpr (benchmark_metrics_compiled) {
        if (!owner_) {return;}
        const auto cpu = cpu_now();
        const auto wall = wall_now() - wall_;
        auto& samples = owner_->operations_[static_cast<std::size_t>(operation_)];
        samples.wall.add(wall);
        if (cpu_ && cpu >= cpu_) {samples.cpu.add(cpu - cpu_);}
        if (operation_ == Operation::Estimation && wall > owner_->budget_ns_) {++owner_->budget_overruns_;}
      }
    }
  private:
    BenchmarkTiming* owner_;
    Operation operation_;
    std::uint64_t wall_{0}, cpu_{0};
  };

  void queued(std::uint64_t enqueued_ns) noexcept
  {
    if constexpr (benchmark_metrics_compiled) {
      if (enabled_ && enqueued_ns) {queue_wait_.add(wall_now() - enqueued_ns);}
    }
  }

  void replayed_states(std::uint64_t states) noexcept
  {
    if constexpr (benchmark_metrics_compiled) {
      if (enabled_) {replay_.observe(states);}
    }
  }

  // Cold path only. A missing CPU clock is reported as zero samples, never as
  // process CPU or wall time pretending to be execution-thread CPU.
  std::string json(const estimation::ReplayStatistics* replay = nullptr) const
  {
    std::ostringstream out;
    out << "{\"enabled\":" << (enabled_ ? "true" : "false")
        << ",\"cpu_clock\":\"CLOCK_THREAD_CPUTIME_ID\",\"quantiles\":\"histogram upper bounds; <=6.25% above 16 ns\""
        << ",\"batch_budget_ns\":" << budget_ns_
        << ",\"batch_budget_overruns\":" << budget_overruns_ << ",\"operations\":{";
    const std::array<const char*, 4> names{{"estimation", "predict", "correct", "replay"}};
    for (std::size_t index = 0; index < names.size(); ++index) {
      if (index) {out << ',';}
      out << '"' << names[index] << "\":{\"cpu\":";
      operations_[index].cpu.write(out);
      out << ",\"wall\":";
      operations_[index].wall.write(out);
      out << '}';
    }
    out << "},\"queue_wait_wall\":";
    queue_wait_.write(out);
    out << ",\"replay_statistics\":" << (replay ? replay : &replay_)->json();
    out << '}';
    return out.str();
  }

private:
  static std::uint64_t cpu_now() noexcept
  {
#if defined(CLOCK_THREAD_CPUTIME_ID)
    timespec time{};
    if (clock_gettime(CLOCK_THREAD_CPUTIME_ID, &time) == 0) {
      return static_cast<std::uint64_t>(time.tv_sec) * 1'000'000'000ULL +
             static_cast<std::uint64_t>(time.tv_nsec);
    }
#endif
    return 0;
  }

  struct Samples
  {
    std::array<std::uint64_t, 1024> bins{};
    std::uint64_t count{0}, total{0}, maximum{0};

    static unsigned exponent(std::uint64_t value) noexcept
    {
#if defined(__GNUC__) || defined(__clang__)
      return 63U - static_cast<unsigned>(__builtin_clzll(value));
#else
      unsigned result = 0;
      while (value >>= 1U) {++result;}
      return result;
#endif
    }

    void add(std::uint64_t ns) noexcept
    {
      ++count; total += ns;
      if (ns > maximum) {maximum = ns;}
      const auto power = exponent(ns ? ns : 1);
      const auto shift = power > 4 ? power - 4 : 0;
      ++bins[power * 16U + static_cast<unsigned>((ns >> shift) & 15U)];
    }

    std::uint64_t quantile(unsigned percent) const noexcept
    {
      const auto target = count / 100 * percent + ((count % 100) * percent + 99) / 100;
      std::uint64_t cumulative = 0;
      for (unsigned index = 0; index < bins.size(); ++index) {
        cumulative += bins[index];
        if (count && cumulative >= target) {
          const auto power = index / 16U, part = index % 16U;
          const auto upper = power < 4 ? part : power == 63 && part == 15 ? maximum :
            ((static_cast<std::uint64_t>(17 + part) << (power - 4U)) - 1);
          return upper < maximum ? upper : maximum;
        }
      }
      return 0;
    }

    void write(std::ostream& out) const
    {
      out << "{\"count\":" << count << ",\"total_ns\":" << total
          << ",\"mean_ns\":" << (count ? static_cast<double>(total) / count : 0.0)
          << ",\"p50_upper_ns\":" << quantile(50) << ",\"p99_upper_ns\":" << quantile(99)
          << ",\"max_ns\":" << maximum << '}';
    }
  };
  struct OperationSamples {Samples cpu, wall;};
  std::array<OperationSamples, 4> operations_{};
  Samples queue_wait_;
  estimation::ReplayStatistics replay_;
  std::uint64_t budget_ns_{0}, budget_overruns_{0};
  bool enabled_{false};
};
}  // namespace rpp_localization
