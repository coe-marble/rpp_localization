// Benchmark-only GNU libc interposer. Pointers and allocator storage stay unchanged.
#include <rpp_localization/ros/worker_metrics.hpp>

#include <array>
#include <atomic>
#include <cerrno>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <limits>
#include <malloc.h>
#include <pthread.h>
#include <sys/syscall.h>
#include <time.h>
#include <unistd.h>

#if !defined(__GLIBC__)
#error "Worker allocation tracking requires GNU libc"
#endif

extern "C" {
void* __libc_malloc(std::size_t) noexcept;
void* __libc_calloc(std::size_t, std::size_t) noexcept;
void* __libc_realloc(void*, std::size_t) noexcept;
void __libc_free(void*) noexcept;
void* __libc_memalign(std::size_t, std::size_t) noexcept;
void* __libc_valloc(std::size_t) noexcept;
void* __libc_pvalloc(std::size_t) noexcept;
}

namespace
{
// Initial-exec TLS prevents the hooks themselves from allocating dynamic TLS.
[[gnu::tls_model("initial-exec")]] thread_local bool is_worker_thread = false;
constexpr std::size_t allocation_capacity = 65536;
constexpr std::size_t bucket_mask = allocation_capacity - 1;
struct Allocation
{
  std::uintptr_t pointer{0};
  std::size_t bytes{0};
};
std::array<Allocation, allocation_capacity> allocations{};
std::atomic_flag allocation_mutex = ATOMIC_FLAG_INIT;
std::atomic<std::size_t> live_allocations{0};
rpp_localization::WorkerMetricsSnapshot counters;
rpp_localization::WorkerMetricsSnapshot frozen_snapshot;
clockid_t worker_clock{};
std::uint64_t initial_cpu_ns{0};
bool registered{false};
bool recording{false};
bool snapshot_ready{false};

struct AllocationLock
{
  AllocationLock() noexcept
  {
    while (allocation_mutex.test_and_set(std::memory_order_acquire)) {}
  }
  ~AllocationLock() {allocation_mutex.clear(std::memory_order_release);}
};

std::size_t allocation_bucket(std::uintptr_t pointer) noexcept
{
  pointer >>= 3;
  pointer ^= pointer >> 17;
  return static_cast<std::size_t>(pointer * 0x9e3779b97f4a7c15ULL) & bucket_mask;
}

void remove_live_allocation(std::uintptr_t pointer) noexcept
{
  if (!pointer) {return;}
  auto slot = allocation_bucket(pointer);
  for (std::size_t scanned = 0; scanned < allocation_capacity;
    ++scanned, slot = (slot + 1) & bucket_mask) {
    if (!allocations[slot].pointer) {return;}
    if (allocations[slot].pointer != pointer) {continue;}
    counters.live_heap_bytes -= allocations[slot].bytes;
    live_allocations.fetch_sub(1, std::memory_order_relaxed);
    // Close the probe-chain gap, avoiding tombstones and any metadata allocation.
    auto hole = slot;
    auto next = (hole + 1) & bucket_mask;
    for (std::size_t moved = 0; moved + 1 < allocation_capacity && allocations[next].pointer;
      ++moved, next = (next + 1) & bucket_mask) {
      const auto home = allocation_bucket(allocations[next].pointer);
      if (((next - home) & bucket_mask) >= ((next - hole) & bucket_mask)) {
        allocations[hole] = allocations[next];
        hole = next;
      }
    }
    allocations[hole] = {};
    return;
  }
}

void record_live_allocation(void* pointer, std::size_t bytes) noexcept
{
  if (!pointer) {return;}
  const auto address = reinterpret_cast<std::uintptr_t>(pointer);
  auto slot = allocation_bucket(address);
  for (std::size_t scanned = 0; scanned < allocation_capacity;
    ++scanned, slot = (slot + 1) & bucket_mask) {
    if (allocations[slot].pointer) {continue;}
    allocations[slot] = {address, bytes};
    live_allocations.fetch_add(1, std::memory_order_relaxed);
    counters.live_heap_bytes += bytes;
    if (counters.live_heap_bytes > counters.peak_live_heap_bytes) {
      counters.peak_live_heap_bytes = counters.live_heap_bytes;
    }
    return;
  }
  // Never publish an incomplete live-byte estimate as an exact measurement.
  counters.memory_complete = false;
}

void record_allocation_call(void* pointer, std::size_t bytes) noexcept
{
  if (!is_worker_thread) {return;}
  AllocationLock lock;
  if (!recording) {return;}
  ++counters.allocation_calls;
  if (pointer) {
    counters.allocated_bytes += bytes;
    record_live_allocation(pointer, bytes);
  }
}

void record_free(std::uintptr_t pointer) noexcept
{
  if (!pointer || live_allocations.load(std::memory_order_relaxed) == 0) {return;}
  AllocationLock lock;
  // A buffer allocated by the worker may be freed by another thread.
  remove_live_allocation(pointer);
}

bool read_worker_cpu_ns(std::uint64_t& result) noexcept
{
  timespec value{};
  if (clock_gettime(worker_clock, &value) != 0) {return false;}
  result = static_cast<std::uint64_t>(value.tv_sec) * 1000000000ULL + value.tv_nsec;
  return true;
}
}  // namespace

namespace rpp_localization
{
extern "C" bool rpp_worker_metrics_begin() noexcept
{
  AllocationLock lock;
  if (registered || pthread_getcpuclockid(pthread_self(), &worker_clock) != 0 ||
    !read_worker_cpu_ns(initial_cpu_ns)) {return false;}
  counters.thread_id = static_cast<std::uint64_t>(syscall(SYS_gettid));
  registered = true;
  recording = true;
  is_worker_thread = true;
  return true;
}

extern "C" bool rpp_worker_metrics_snapshot(WorkerMetricsSnapshot* output) noexcept
{
  AllocationLock lock;
  if (!registered) {return false;}
  if (!snapshot_ready) {
    std::uint64_t current;
    if (!read_worker_cpu_ns(current) || current < initial_cpu_ns) {return false;}
    counters.cpu_ns = current - initial_cpu_ns;
    frozen_snapshot = counters;
    recording = false;
    snapshot_ready = true;
  }
  *output = frozen_snapshot;
  return true;
}
}  // namespace rpp_localization

extern "C" void* malloc(std::size_t bytes) noexcept
{
  void* pointer = __libc_malloc(bytes);
  record_allocation_call(pointer, bytes);
  return pointer;
}

extern "C" void free(void* pointer) noexcept
{
  const int saved_errno = errno;
  record_free(reinterpret_cast<std::uintptr_t>(pointer));
  __libc_free(pointer);
  errno = saved_errno;
}

extern "C" void* calloc(std::size_t count, std::size_t size) noexcept
{
  void* pointer = __libc_calloc(count, size);
  const auto bytes = size && count > std::numeric_limits<std::size_t>::max() / size ? 0 : count * size;
  record_allocation_call(pointer, bytes);
  return pointer;
}

extern "C" void* realloc(void* pointer, std::size_t bytes) noexcept
{
  const auto previous = reinterpret_cast<std::uintptr_t>(pointer);
  void* replacement = __libc_realloc(pointer, bytes);
  if (replacement || bytes == 0) {record_free(previous);}
  // Failed requests count, while the old block stays live.
  record_allocation_call(replacement, bytes);
  return replacement;
}

extern "C" void* aligned_alloc(std::size_t alignment, std::size_t bytes) noexcept
{
  if (!alignment || (alignment & (alignment - 1))) {
    record_allocation_call(nullptr, 0);
    errno = EINVAL;
    return nullptr;
  }
  void* pointer = __libc_memalign(alignment, bytes);
  record_allocation_call(pointer, bytes);
  return pointer;
}

extern "C" int posix_memalign(void** output, std::size_t alignment, std::size_t bytes) noexcept
{
  if (alignment < sizeof(void*) || (alignment & (alignment - 1))) {
    record_allocation_call(nullptr, 0);
    return EINVAL;
  }
  const int saved_errno = errno;
  void* pointer = __libc_memalign(alignment, bytes);
  record_allocation_call(pointer, bytes);
  errno = saved_errno;
  if (!pointer) {return ENOMEM;}
  *output = pointer;
  return 0;
}

extern "C" void* memalign(std::size_t alignment, std::size_t bytes) noexcept
{
  void* pointer = __libc_memalign(alignment, bytes);
  record_allocation_call(pointer, bytes);
  return pointer;
}

extern "C" void* valloc(std::size_t bytes) noexcept
{
  void* pointer = __libc_valloc(bytes);
  record_allocation_call(pointer, bytes);
  return pointer;
}

extern "C" void* pvalloc(std::size_t bytes) noexcept
{
  void* pointer = __libc_pvalloc(bytes);
  record_allocation_call(pointer, bytes);
  return pointer;
}

extern "C" void free_sized(void* pointer, std::size_t) noexcept {free(pointer);}
extern "C" void free_aligned_sized(void* pointer, std::size_t, std::size_t) noexcept {free(pointer);}
extern "C" void cfree(void* pointer) noexcept {free(pointer);}
