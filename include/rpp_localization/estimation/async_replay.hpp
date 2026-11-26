#pragma once

#include <rpp_localization/estimation/prepared_queue.hpp>
#include <algorithm>
#include <atomic>
#include <cstdint>
#include <utility>
#include <vector>

namespace rpp_localization::estimation
{
// One live producer and one ROS executor consumer. Only the live owner switches
// roles. A ready result is immutable until that owner adopts it or requests
// catch-up. Lifecycle/reset require both owners to be quiescent.
class AsyncReplay
{
public:
  enum class Phase : std::uint8_t {Idle, Running, Ready, Cancelled};
  enum class Kind : std::uint8_t {Predict, Correct};
  struct Event
  {
    Event(std::size_t width, std::size_t square) : values(width), covariance(square) {}
    Kind kind{Kind::Correct};
    std::uint64_t sequence{0}, time{0}, dt{0}, input_dt{0};
    std::size_t width{0}, square{0};
    std::uint32_t index{0};
    std::vector<double> values, covariance;
  };
  class Batch
  {
  public:
    Batch(const Event* const* events, std::size_t size) : events_(events), size_(size) {}
    std::size_t size() const noexcept {return size_;}
    const Event& operator[](std::size_t index) const {return *events_[index];}
  private:
    const Event* const* events_;
    std::size_t size_;
  };

  void prepare(std::size_t capacity, std::size_t width, std::size_t square)
  {
    queue_.prepare(capacity, [&] {return Event(width, square);});
    batch_.resize(capacity);
    reset();
  }
  void reset()
  {
    queue_.reset(); live_sequence_ = replay_sequence_ = 0;
    ready_sequence_.store(0, std::memory_order_relaxed);
    phase_.store(Phase::Idle, std::memory_order_release);
  }
  Phase phase() const noexcept {return phase_.load(std::memory_order_acquire);}
  bool active() const noexcept {const auto state = phase(); return state == Phase::Running || state == Phase::Ready;}

  template<class Fork> bool start(Fork&& fork)
  {
    if (phase() != Phase::Idle || !queue_.empty() || !fork()) {return false;}
    replay_sequence_ = live_sequence_;
    phase_.store(Phase::Running, std::memory_order_release);
    return true;
  }

  template<class Values, class Covariance>
  bool record(Kind kind, std::uint32_t index, const Values& values, const Covariance& covariance,
    std::uint64_t time, std::uint64_t dt = 0, std::uint64_t input_dt = 0)
  {
    ++live_sequence_;
    if (!active()) {return true;}
    auto* event = queue_.write();
    if (!event || values.size() > event->values.size() || covariance.size() > event->covariance.size()) {
      phase_.store(Phase::Cancelled, std::memory_order_release); return false;
    }
    event->kind = kind; event->index = index; event->sequence = live_sequence_;
    event->time = time; event->dt = dt; event->input_dt = input_dt;
    event->width = values.size(); event->square = covariance.size();
    for (std::size_t axis = 0; axis < event->width; ++axis) {event->values[axis] = values[axis];}
    for (std::size_t axis = 0; axis < event->square; ++axis) {event->covariance[axis] = covariance[axis];}
    queue_.commit(); return true;
  }

  // Called only at a live execution boundary. The consumer has published Ready
  // after releasing every borrowed event and will not touch either instance.
  template<class Switch> bool try_switch(Switch&& switch_roles)
  {
    if (phase() != Phase::Ready) {return false;}
    if (ready_sequence_.load(std::memory_order_acquire) != live_sequence_) {
      phase_.store(Phase::Running, std::memory_order_release); return false;
    }
    switch_roles();
    phase_.store(Phase::Idle, std::memory_order_release);
    return true;
  }

  template<class Apply> bool service(std::size_t limit, Apply&& apply)
  {return service_batches(limit, 1, [&](const Batch& batch) {return apply(batch[0]);});}

  // The callback borrows a bounded prefix. No slot is released until it returns.
  template<class ApplyBatch> bool service_batch(std::size_t limit, ApplyBatch&& apply)
  {return service_batches(limit, batch_.size(), std::forward<ApplyBatch>(apply));}

private:
  template<class ApplyBatch> bool service_batches(std::size_t limit, std::size_t batch_limit, ApplyBatch&& apply)
  {
    if (phase() == Phase::Cancelled) {discard(); return false;}
    if (phase() != Phase::Running) {return true;}
    try {
      std::size_t completed = 0;
      while (completed < limit && phase() == Phase::Running) {
        std::size_t count = 0;
        const auto maximum = std::min(batch_limit, limit - completed);
        for (; count < maximum; ++count) {
          const auto* event = queue_.front(count);
          if (!event) {break;}
          if (event->sequence != replay_sequence_ + count + 1) {
            phase_.store(Phase::Cancelled, std::memory_order_release); discard(); return false;
          }
          batch_[count] = event;
        }
        if (!count) {break;}
        const auto sequence = batch_[count - 1]->sequence;
        const bool accepted = apply(Batch(batch_.data(), count));
        for (std::size_t index = 0; index < count; ++index) {queue_.pop();}
        if (!accepted) {phase_.store(Phase::Cancelled, std::memory_order_release); discard(); return false;}
        replay_sequence_ = sequence;
        completed += count;
      }
    } catch (...) {
      phase_.store(Phase::Cancelled, std::memory_order_release); discard(); return false;
    }
    if (phase() == Phase::Cancelled) {discard(); return false;}
    if (phase() == Phase::Running && queue_.empty()) {
      ready_sequence_.store(replay_sequence_, std::memory_order_release);
      auto expected = Phase::Running;
      phase_.compare_exchange_strong(expected, Phase::Ready, std::memory_order_release, std::memory_order_relaxed);
    }
    return true;
  }

  void discard()
  {
    // A live producer may have been between write() and commit() when another
    // owner cancelled. Leave Cancelled until reset under a management pause;
    // never reset a ring while its producer could still be publishing a slot.
    while (!queue_.empty()) {queue_.pop();}
  }
  PreparedQueue<Event> queue_;
  std::vector<const Event*> batch_;
  std::atomic<Phase> phase_{Phase::Idle};
  std::atomic<std::uint64_t> ready_sequence_{0};
  std::uint64_t live_sequence_{0}, replay_sequence_{0};
};
}  // namespace rpp_localization::estimation
