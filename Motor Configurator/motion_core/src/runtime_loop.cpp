#include "motion_core/runtime_loop.h"

#include <pthread.h>
#include <sched.h>
#include <sys/mman.h>

namespace {

void configure_realtime_best_effort() {
    (void)mlockall(MCL_CURRENT | MCL_FUTURE);

    sched_param params{};
    params.sched_priority = 80;
    (void)pthread_setschedparam(pthread_self(), SCHED_FIFO, &params);
}

} // namespace

namespace motion_core {

RuntimeLoop::~RuntimeLoop() {
    (void)stop();
}

Result<void> RuntimeLoop::start(std::chrono::milliseconds period, TickHandler on_tick) {
    if (period.count() <= 0) {
        return Result<void>::failure({ErrorCode::InvalidArgument, "period must be positive"});
    }
    if (!on_tick) {
        return Result<void>::failure({ErrorCode::InvalidArgument, "tick handler is empty"});
    }

    bool expected = false;
    if (!running_.compare_exchange_strong(expected, true, std::memory_order_release, std::memory_order_relaxed)) {
        return Result<void>::failure({ErrorCode::Busy, "runtime loop already running"});
    }

    overrun_count_.store(0, std::memory_order_relaxed);
    thread_ = std::thread(&RuntimeLoop::worker, this, period, std::move(on_tick));
    return Result<void>::success();
}

Result<void> RuntimeLoop::stop() {
    bool expected = true;
    if (!running_.compare_exchange_strong(expected, false, std::memory_order_release, std::memory_order_relaxed)) {
        return Result<void>::success();
    }

    if (thread_.joinable()) {
        thread_.join();
    }
    return Result<void>::success();
}

void RuntimeLoop::worker(std::chrono::milliseconds period, TickHandler on_tick) {
    configure_realtime_best_effort();

    auto next_tick = std::chrono::steady_clock::now() + period;

    while (running_.load(std::memory_order_acquire)) {
        on_tick();
        
        auto now = std::chrono::steady_clock::now();
        if (now > next_tick + period) {
            // Overrun by more than 1 frame -> reset scheduling to avoid burst catch-up
            next_tick = now + period;
            overrun_count_.fetch_add(1, std::memory_order_relaxed);
        } else {
            std::this_thread::sleep_until(next_tick);
            next_tick += period;
        }
    }
}

} // namespace motion_core
