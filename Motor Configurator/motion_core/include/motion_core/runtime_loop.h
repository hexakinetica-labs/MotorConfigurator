#pragma once

#include "motion_core/result.h"

#include <atomic>
#include <chrono>
#include <cstdint>
#include <functional>
#include <thread>

namespace motion_core {

class RuntimeLoop {
public:
    using TickHandler = std::function<void()>;

    RuntimeLoop() = default;
    ~RuntimeLoop();

    RuntimeLoop(const RuntimeLoop&) = delete;
    RuntimeLoop& operator=(const RuntimeLoop&) = delete;

    Result<void> start(std::chrono::microseconds period, TickHandler on_tick);
    Result<void> stop();

    [[nodiscard]] bool is_running() const noexcept { return running_.load(std::memory_order_acquire); }
    [[nodiscard]] std::uint64_t get_overrun_count() const noexcept { return overrun_count_.load(std::memory_order_relaxed); }

private:
    void worker(std::chrono::microseconds period, TickHandler on_tick);

    std::atomic<bool> running_{false};
    std::atomic<std::uint64_t> overrun_count_{0};
    std::thread thread_{};
};

} // namespace motion_core
