#pragma once

#include <rpp_localization/estimation/filter.hpp>
#include <array>
#include <typeinfo>

namespace rpp_localization::estimation
{

// Splitting an arbitrary discrete Q is undefined. Enable constant_rate only
// when Q/dt is the declared, frozen noise density for that prediction interval.
enum class ReplayNoise {reject_split, constant_rate};

// One prepared ring for accepted events, another for a transactional replay.
// Checkpoints include algorithm history through GaussianFilter's virtual hooks.
// All model calls execute inside the adapter owner's guarded Model call.
template<bool CollectMetrics>
class ReplayImplementation final
{
  struct Event
  {
    Event(GaussianFilter& filter, Eigen::Index n, Eigen::Index inputs, Eigen::Index maximum)
    : input(inputs), q(n, n), measurement(maximum), r(maximum, maximum),
      checkpoint(filter.make_checkpoint()) {}
    bool prediction{true};
    TimeNs time{0}, start{0}, dt{0}, input_dt{0};
    std::size_t observation{0};
    Eigen::Index width{0};
    Vector input;
    Matrix q;
    Vector measurement;
    Matrix r;
    std::unique_ptr<GaussianFilter::Checkpoint> checkpoint;
  };
  struct Bank
  {
    Bank(GaussianFilter& filter, std::size_t capacity, Eigen::Index inputs, Eigen::Index maximum)
    : anchor(filter.make_checkpoint()), anchor_input(Vector::Zero(inputs))
    {
      events.reserve(capacity);
      for (std::size_t index = 0; index < capacity; ++index) {
        events.emplace_back(filter, filter.state().size(), inputs, maximum);
      }
    }
    Event& at(std::size_t offset) {return events[(head + offset) % events.size()];}
    const Event& at(std::size_t offset) const {return events[(head + offset) % events.size()];}
    Event& next()
    {
      if (count == events.size()) {
        if (!at(0).checkpoint->copy_to(*anchor)) {throw std::logic_error("Checkpoint type mismatch");}
        anchor_input = at(0).input; anchor_input_dt = at(0).input_dt;
        head = (head + 1) % events.size(); --count;
      }
      return at(count++);
    }
    std::vector<Event> events;
    std::unique_ptr<GaussianFilter::Checkpoint> anchor;
    Vector anchor_input;
    TimeNs anchor_input_dt{0};
    std::size_t head{0}, count{0};
  };

public:
  struct EventInfo {bool prediction; TimeNs time, dt;};
  static constexpr std::size_t batch_capacity = 4096;

  ReplayImplementation(GaussianFilter& filter, const std::vector<ObservationBinding>& observations,
    Eigen::Index input_size, std::size_t capacity = 512,
    ReplayNoise noise = ReplayNoise::reject_split, ExecutionMetrics* metrics = nullptr)
  : core_(filter), noise_(noise), main_(filter, checked_capacity(capacity), input_size, maximum(observations)),
    scratch_(filter, checked_capacity(capacity) + 2, input_size, maximum(observations)),
    incoming_(filter, filter.state().size(), input_size, maximum(observations)),
    covering_(filter, filter.state().size(), input_size, maximum(observations)),
    prediction_(filter, filter.state().size(), input_size, maximum(observations)),
    rollback_(filter.make_checkpoint()), before_insertion_(filter.make_checkpoint()), metrics_(metrics)
  {
    if constexpr (CollectMetrics) {
      if (!metrics_) {throw std::invalid_argument("Instrumented replay requires a metrics sink");}
    }
  }

  void reset()
  {
    main_.head = main_.count = scratch_.head = scratch_.count = 0;
    main_.anchor_input.setZero(); main_.anchor_input_dt = 0;
    if (!core_.save(*main_.anchor)) {throw std::logic_error("Cannot checkpoint this filter");}
  }

  bool compatible(const ReplayImplementation& target) const
  {
    return this != &target && typeid(core_) == typeid(target.core_) && main_.events.size() == target.main_.events.size() &&
      core_.state().size() == target.core_.state().size() && incoming_.input.size() == target.incoming_.input.size() &&
      incoming_.measurement.size() == target.incoming_.measurement.size() && noise_ == target.noise_;
  }

  TimeNs oldest_time() const noexcept {return main_.anchor->time;}
  const Vector& latest_input() const
  {return main_.count ? main_.at(main_.count - 1).input : main_.anchor_input;}
  TimeNs latest_input_dt() const
  {return main_.count ? main_.at(main_.count - 1).input_dt : main_.anchor_input_dt;}

  bool fork_to(ReplayImplementation& target)
  {
    if (!compatible(target) || !core_.save(*rollback_) || !target.core_.restore(*rollback_)) {return false;}
    // History changes owners; no checkpoint bank is copied on the live worker.
    // The running core keeps its posterior and starts a fresh history bank.
    using std::swap;
    swap(main_, target.main_);
    reset();
    const auto& source = target.main_;
    main_.anchor_input = source.count ? source.at(source.count - 1).input : source.anchor_input;
    main_.anchor_input_dt = source.count ? source.at(source.count - 1).input_dt : source.anchor_input_dt;
    return true;
  }

  Status predict(VectorIn input, MatrixIn q, TimeNs time, TimeNs dt, TimeNs input_dt)
  {
    const auto status = core_.predict(input, q, time, dt, input_dt);
    if (status != Status::ok) {return status;}
    incoming_.prediction = true; incoming_.time = time; incoming_.start = time - dt;
    incoming_.dt = dt; incoming_.input_dt = input_dt; incoming_.input = input; incoming_.q = q;
    append(main_, incoming_);
    return status;
  }

  Status correct(std::size_t observation, VectorIn measurement, MatrixIn r,
    VectorIn input, TimeNs time, TimeNs input_dt)
  {
    if (measurement.size() > incoming_.measurement.size() || r.rows() != measurement.size() ||
      r.cols() != measurement.size() || input.size() != incoming_.input.size()) {return Status::invalid_input;}
    incoming_.prediction = false; incoming_.time = time; incoming_.observation = observation;
    incoming_.width = measurement.size(); incoming_.input_dt = input_dt; incoming_.input = input;
    incoming_.measurement.head(incoming_.width) = measurement;
    incoming_.r.topLeftCorner(incoming_.width, incoming_.width) = r;
    if (time == core_.time()) {
      const auto status = apply(incoming_);
      if (accepted(status)) {append(main_, incoming_);}
      return status;
    }
    if (time > core_.time()) {return Status::invalid_input;}
    if constexpr (CollectMetrics) {++metrics_->late_measurements;}
    if (time < main_.anchor->time) {return Status::stale_input;}
    return replay();
  }

  // The source stays borrowed until return. Validate once, then merge by time;
  // equal-time retained events precede new events, whose arrival order is stable.
  template<class Describe, class Load>
  Status replay_batch(std::size_t count, Describe&& describe, Load&& load)
  {
    if (count > batch_capacity) {return Status::invalid_input;}
    const auto original_time = core_.time();
    auto arrival_time = original_time;
    auto earliest = original_time;
    std::size_t prepared = 0;
    for (std::size_t index = 0; index < count; ++index) {
      const auto info = describe(index);
      const auto status = load(index, incoming_, true);
      if (status != Status::ok) {return status;}
      batch_late_[index] = false;
      if (info.prediction) {
        if (info.time < arrival_time || info.dt != info.time - arrival_time) {return Status::invalid_input;}
        arrival_time = info.time;
      } else {
        if (info.time > arrival_time) {return Status::invalid_input;}
        batch_late_[index] = info.time < arrival_time;
        if constexpr (CollectMetrics) {if (batch_late_[index]) {++metrics_->late_measurements;}}
        if (info.time < oldest_time()) {count_stale_correction(); continue;}
        earliest = std::min(earliest, info.time);
      }
      batch_order_[prepared++] = index;
    }
    if (!prepared) {return Status::ok;}
    std::sort(batch_order_.begin(), batch_order_.begin() + prepared,
      [&](std::size_t a, std::size_t b) {
        const auto left = describe(a).time, right = describe(b).time;
        return left == right ? a < b : left < right;
      });
    const auto insertion = upper_bound(earliest);
    const bool rewound = earliest < original_time;
    if constexpr (CollectMetrics) {
      if (rewound) {
        ++metrics_->replay_attempts;
        metrics_->replay_states.observe(static_cast<std::uint64_t>(main_.count - insertion));
      }
    }
    if (!core_.save(*rollback_)) {
      if constexpr (CollectMetrics) {if (rewound) {++metrics_->replay_failures;}}
      return Status::model_failure;
    }
    const auto fail = [&](Status status) {
      if constexpr (CollectMetrics) {if (rewound) {++metrics_->replay_failures;}}
      core_.restore(*rollback_); return status;
    };
    try {
      scratch_.head = scratch_.count = 0;
      const auto& checkpoint = !rewound ? *rollback_ :
        (insertion ? *main_.at(insertion - 1).checkpoint : *main_.anchor);
      if constexpr (CollectMetrics) {
        if (rewound) {
          const auto tail = static_cast<std::uint64_t>(main_.count - insertion);
          const auto span = original_time - checkpoint.time;
          metrics_->tail_events_total += tail;
          metrics_->tail_events_max = std::max(metrics_->tail_events_max, tail);
          metrics_->rollback_span_ns_total += span;
          metrics_->rollback_span_ns_max = std::max(metrics_->rollback_span_ns_max, span);
        }
      }
      if (!checkpoint.copy_to(*scratch_.anchor) || !core_.restore(checkpoint)) {return fail(Status::model_failure);}
      scratch_.anchor_input = insertion ? main_.at(insertion - 1).input : main_.anchor_input;
      scratch_.anchor_input_dt = insertion ? main_.at(insertion - 1).input_dt : main_.anchor_input_dt;
      std::size_t historical = insertion, queued = 0;
      bool kept_new_event = false;
      while (historical < main_.count || queued < prepared) {
        if (historical < main_.count && (queued == prepared ||
            main_.at(historical).time <= describe(batch_order_[queued]).time)) {
          const auto& event = main_.at(historical++);
          const auto status = event.prediction ? predict_batch_part(event, event.time, true) : apply(event, true);
          const bool succeeded = event.prediction ? status == Status::ok : accepted(status);
          if (!succeeded) {return fail(status);}
          if (event.prediction) {append_checkpoint(scratch_, prediction_);}
          else {append(scratch_, event);}
          continue;
        }
        const auto index = batch_order_[queued++];
        auto status = load(index, incoming_, false);
        if (status != Status::ok) {return fail(status);}
        if (incoming_.prediction) {
          status = predict_batch_part(incoming_, incoming_.time, false);
          if (status != Status::ok) {return fail(status);}
          append_checkpoint(scratch_, prediction_);
          kept_new_event = true;
          continue;
        }
        if (!core_.save(*before_insertion_)) {return fail(Status::model_failure);}
        const bool advance = incoming_.time > core_.time();
        if (advance) {
          const Event* covering = nullptr;
          bool historical_prediction = true;
          for (std::size_t offset = historical; offset < main_.count; ++offset) {
            const auto& event = main_.at(offset);
            if (event.prediction && event.start <= core_.time() && event.time >= incoming_.time) {
              covering = &event; break;
            }
          }
          for (std::size_t offset = queued; offset < prepared; ++offset) {
            const auto info = describe(batch_order_[offset]);
            if (info.prediction && info.time - info.dt <= core_.time() && info.time >= incoming_.time) {
              if (!covering || info.time < covering->time) {
                status = load(batch_order_[offset], covering_, false);
                if (status != Status::ok) {return fail(status);}
                covering = &covering_; historical_prediction = false;
              }
              break;
            }
          }
          if (!covering) {count_stale_correction(); continue;}
          status = predict_batch_part(*covering, incoming_.time, historical_prediction);
          if (status == Status::stale_input) {count_stale_correction(); continue;}
          if (status != Status::ok) {return fail(status);}
          incoming_.input = covering->input; incoming_.input_dt = covering->input_dt;
        } else {
          const auto* previous = scratch_.count ? &scratch_.at(scratch_.count - 1) : nullptr;
          incoming_.input = previous ? previous->input : scratch_.anchor_input;
          incoming_.input_dt = previous ? previous->input_dt : scratch_.anchor_input_dt;
        }
        status = apply(incoming_);
        if ((status == Status::gated && batch_late_[index]) || status == Status::stale_input) {
          // Discard a rejected insertion's tentative split, not earlier accepted corrections.
          if (!core_.restore(*before_insertion_)) {return fail(Status::model_failure);}
          if (status == Status::stale_input) {count_stale_correction();}
          continue;
        }
        if (!accepted(status)) {return fail(status);}
        if (advance) {append_checkpoint(scratch_, prediction_);}
        append(scratch_, incoming_);
        kept_new_event = true;
      }
      if (!kept_new_event) {return fail(Status::gated);}
      commit_tail(insertion);
      if constexpr (CollectMetrics) {if (rewound) {++metrics_->replay_successes;}}
      return Status::ok;
    } catch (...) {return fail(Status::model_failure);}
  }

private:
  std::size_t upper_bound(TimeNs time) const
  {
    std::size_t first = 0, end = main_.count;
    while (first < end) {
      const auto middle = first + (end - first) / 2;
      if (main_.at(middle).time <= time) {first = middle + 1;}
      else {end = middle;}
    }
    return first;
  }
  void count_stale_correction()
  {if constexpr (CollectMetrics) {++metrics_->replay_stale_corrections;}}
  void append_checkpoint(Bank& bank, const Event& event)
  {
    auto& output = bank.next(); copy_event(event, output);
    if (!event.checkpoint->copy_to(*output.checkpoint)) {throw std::logic_error("Checkpoint type mismatch");}
  }
  void commit_tail(std::size_t insertion)
  {
    if (scratch_.count >= main_.events.size()) {
      // A full new tail replaces the prefix, including any evicted scratch history.
      main_.head = main_.count = 0;
      if (!scratch_.anchor->copy_to(*main_.anchor)) {throw std::logic_error("Checkpoint type mismatch");}
      main_.anchor_input = scratch_.anchor_input; main_.anchor_input_dt = scratch_.anchor_input_dt;
    } else {main_.count = insertion;}
    for (std::size_t index = 0; index < scratch_.count; ++index) {append_checkpoint(main_, scratch_.at(index));}
  }
  Status predict_batch_part(const Event& original, TimeNs end, bool replayed)
  {
    if (end < core_.time()) {return Status::invalid_input;}
    const auto dt = end - core_.time();
    if (dt != original.dt && noise_ != ReplayNoise::constant_rate) {return Status::stale_input;}
    copy_event(original, prediction_);
    prediction_.start = core_.time(); prediction_.time = end; prediction_.dt = dt;
    if (dt != original.dt) {
      if (original.dt == 0) {return Status::stale_input;}
      prediction_.q *= static_cast<double>(dt) / static_cast<double>(original.dt);
    }
    const auto status = apply(prediction_, replayed);
    if (status == Status::ok && !core_.save(*prediction_.checkpoint)) {return Status::model_failure;}
    return status;
  }
  static std::size_t checked_capacity(std::size_t capacity)
  {
    if (capacity == 0 || capacity > 4096) {throw std::invalid_argument("Replay capacity must be 1..4096");}
    return capacity;
  }
  static Eigen::Index maximum(const std::vector<ObservationBinding>& observations)
  {
    Eigen::Index width = 0;
    for (const auto& binding : observations) {width = std::max(width, binding.model->observation_space().size());}
    return width;
  }
  static bool accepted(Status status) {return status == Status::ok || status == Status::gated;}
  static void copy_event(const Event& source, Event& target)
  {
    target.prediction = source.prediction; target.time = source.time; target.start = source.start;
    target.dt = source.dt; target.input_dt = source.input_dt; target.observation = source.observation;
    target.width = source.width; target.input = source.input;
    if (source.prediction) {target.q = source.q;}
    else {
      target.measurement.head(source.width) = source.measurement.head(source.width);
      target.r.topLeftCorner(source.width, source.width) = source.r.topLeftCorner(source.width, source.width);
    }
  }
  void append(Bank& bank, const Event& event)
  {
    auto& output = bank.next(); copy_event(event, output);
    if (!core_.save(*output.checkpoint)) {throw std::logic_error("Cannot checkpoint this filter");}
  }
  Status apply(const Event& event, bool replayed = false)
  {
    if constexpr (CollectMetrics) {
      if (replayed) {
        if (event.prediction) {++metrics_->replayed_predictions;}
        else {++metrics_->replayed_corrections;}
      }
    }
    return event.prediction ? core_.predict(event.input, event.q, event.time, event.dt, event.input_dt) :
      core_.correct(event.observation, event.measurement.head(event.width),
        event.r.topLeftCorner(event.width, event.width), event.input, event.time, event.input_dt);
  }
  Status predict_part(const Event& original, TimeNs end)
  {
    const auto dt = end - core_.time();
    if (dt != original.dt && noise_ != ReplayNoise::constant_rate) {return Status::stale_input;}
    auto& event = scratch_.next();
    copy_event(original, event);
    event.start = core_.time(); event.time = end; event.dt = dt;
    if (dt != original.dt) {
      if (original.dt == 0) {return Status::stale_input;}
      event.q *= static_cast<double>(dt) / static_cast<double>(original.dt);
    }
    const auto status = apply(event, true);
    if (status == Status::ok && !core_.save(*event.checkpoint)) {return Status::model_failure;}
    return status;
  }
  Status replay()
  {
    // Locate the affected tail before checkpointing so every attempt, including
    // a failed checkpoint save, contributes exactly one history-length sample.
    std::size_t insertion = 0, end = main_.count;
    while (insertion < end) {
      const auto middle = insertion + (end - insertion) / 2;
      if (main_.at(middle).time <= incoming_.time) {insertion = middle + 1;}
      else {end = middle;}
    }
    if constexpr (CollectMetrics) {
      ++metrics_->replay_attempts;
      metrics_->replay_states.observe(static_cast<std::uint64_t>(main_.count - insertion));
    }
    if (!core_.save(*rollback_)) {
      if constexpr (CollectMetrics) {++metrics_->replay_failures;}
      return Status::model_failure;
    }
    const auto fail = [&](Status status) {
      if constexpr (CollectMetrics) {++metrics_->replay_failures;}
      core_.restore(*rollback_); return status;
    };
    try {
      scratch_.head = scratch_.count = 0;
      // Retained events are chronological, including equal-time corrections.
      const auto& checkpoint = insertion ? *main_.at(insertion - 1).checkpoint : *main_.anchor;
      if constexpr (CollectMetrics) {
        const auto tail = static_cast<std::uint64_t>(main_.count - insertion);
        const auto span = rollback_->time - checkpoint.time;
        metrics_->tail_events_total += tail;
        metrics_->tail_events_max = std::max(metrics_->tail_events_max, tail);
        metrics_->rollback_span_ns_total += span;
        metrics_->rollback_span_ns_max = std::max(metrics_->rollback_span_ns_max, span);
      }
      if (!checkpoint.copy_to(*scratch_.anchor)) {return fail(Status::model_failure);}
      scratch_.anchor_input = insertion ? main_.at(insertion - 1).input : main_.anchor_input;
      scratch_.anchor_input_dt = insertion ? main_.at(insertion - 1).input_dt : main_.anchor_input_dt;
      if (!core_.restore(checkpoint)) {return fail(Status::model_failure);}
      if (incoming_.time > core_.time()) {
        const Event* covering = nullptr;
        for (std::size_t index = insertion; index < main_.count; ++index) {
          const auto& event = main_.at(index);
          if (event.prediction && event.start <= core_.time() && event.time >= incoming_.time) {
            covering = &event; break;
          }
        }
        if (!covering) {return fail(Status::stale_input);}
        const auto status = predict_part(*covering, incoming_.time);
        if (status != Status::ok) {return fail(status);}
        incoming_.input = covering->input; incoming_.input_dt = covering->input_dt;
      } else {
        incoming_.input = insertion ? main_.at(insertion - 1).input : main_.anchor_input;
        incoming_.input_dt = insertion ? main_.at(insertion - 1).input_dt : main_.anchor_input_dt;
      }
      const auto result = apply(incoming_);
      // A rejected late sample must not change the estimate merely by splitting
      // and reintegrating a nonlinear prediction interval.
      if (result != Status::ok) {return fail(result);}
      append(scratch_, incoming_);
      for (std::size_t index = insertion; index < main_.count; ++index) {
        const auto& event = main_.at(index);
        if (event.prediction) {
          if (event.time < core_.time()) {return fail(Status::invalid_input);}
          const auto status = predict_part(event, event.time);
          if (status != Status::ok) {return fail(status);}
        } else {
          const auto status = apply(event, true);
          if (!accepted(status)) {return fail(status);}
          append(scratch_, event);
        }
      }
      // The unaffected prefix stays in place. Commit only the successfully
      // replayed tail; prepared checkpoint copies cannot allocate or change type.
      commit_tail(insertion);
      if constexpr (CollectMetrics) {++metrics_->replay_successes;}
      return result;
    } catch (...) {return fail(Status::model_failure);}
  }

  GaussianFilter& core_;
  ReplayNoise noise_;
  Bank main_, scratch_;
  Event incoming_, covering_, prediction_;
  std::unique_ptr<GaussianFilter::Checkpoint> rollback_, before_insertion_;
  std::array<std::size_t, batch_capacity> batch_order_{};
  std::array<bool, batch_capacity> batch_late_{};
  ExecutionMetrics* metrics_{nullptr};
};
using Replay = ReplayImplementation<false>;
using ProfiledReplay = ReplayImplementation<true>;
}  // namespace rpp_localization::estimation
