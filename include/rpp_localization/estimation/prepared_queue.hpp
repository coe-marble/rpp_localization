#pragma once

#include <atomic>
#include <cstddef>
#include <stdexcept>
#include <vector>

namespace rpp_localization::estimation
{
// Exactly one producer and one consumer. Preparation/reset require both to be
// paused. A full queue rejects the new item; no live slot is overwritten.
template<class Value>
class PreparedQueue
{
public:
  template<class Factory>
  void prepare(std::size_t capacity, Factory&& factory)
  {
    if (capacity < 1 || capacity > 4096) {throw std::invalid_argument("Queue capacity must be 1..4096");}
    values_.clear(); values_.reserve(capacity + 1);
    for (std::size_t index = 0; index <= capacity; ++index) {values_.push_back(factory());}
    reset();
  }
  Value* write()
  {
    const auto current = write_.load(std::memory_order_relaxed);
    return values_.empty() || next(current) == read_.load(std::memory_order_acquire) ? nullptr : &values_[current];
  }
  void commit() {write_.store(next(write_.load(std::memory_order_relaxed)), std::memory_order_release);}
  // Consumer-only borrowing. Slots remain pinned until pop() releases them.
  const Value* front(std::size_t offset = 0) const
  {
    const auto current = read_.load(std::memory_order_relaxed);
    const auto written = write_.load(std::memory_order_acquire);
    const auto available = written >= current ? written - current : values_.size() - current + written;
    return offset < available ? &values_[(current + offset) % values_.size()] : nullptr;
  }
  void pop() {read_.store(next(read_.load(std::memory_order_relaxed)), std::memory_order_release);}
  // Observation must not borrow storage: an execution owner may check these
  // indices while management is replacing the prepared value buffers.
  bool empty() const
  {return read_.load(std::memory_order_relaxed) == write_.load(std::memory_order_acquire);}
  void reset() {read_.store(0, std::memory_order_relaxed); write_.store(0, std::memory_order_relaxed);}
private:
  std::size_t next(std::size_t index) const {return index + 1 == values_.size() ? 0 : index + 1;}
  std::vector<Value> values_;
  alignas(64) std::atomic<std::size_t> read_{0};
  alignas(64) std::atomic<std::size_t> write_{0};
};
}  // namespace rpp_localization::estimation
