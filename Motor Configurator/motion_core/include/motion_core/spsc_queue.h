#pragma once

#include <array>
#include <atomic>
#include <cstddef>

namespace motion_core {

template <typename T, std::size_t Capacity>
class SpscQueue final {
    static_assert(Capacity >= 2U, "SpscQueue capacity must be >= 2");

public:
    [[nodiscard]] bool try_push(const T& value) noexcept {
        const std::size_t head = head_.load(std::memory_order_relaxed);
        const std::size_t next = advance(head);
        const std::size_t tail = tail_.load(std::memory_order_acquire);
        if (next == tail) {
            return false;
        }

        buffer_[head] = value;
        head_.store(next, std::memory_order_release);
        return true;
    }

    [[nodiscard]] bool try_pop(T& out_value) noexcept {
        const std::size_t tail = tail_.load(std::memory_order_relaxed);
        const std::size_t head = head_.load(std::memory_order_acquire);
        if (tail == head) {
            return false;
        }

        out_value = buffer_[tail];
        tail_.store(advance(tail), std::memory_order_release);
        return true;
    }

    [[nodiscard]] std::size_t size_approx() const noexcept {
        const std::size_t head = head_.load(std::memory_order_acquire);
        const std::size_t tail = tail_.load(std::memory_order_acquire);
        if (head >= tail) {
            return head - tail;
        }
        return Capacity - (tail - head);
    }

    [[nodiscard]] constexpr std::size_t capacity() const noexcept {
        return Capacity - 1U;
    }

private:
    [[nodiscard]] static constexpr std::size_t advance(const std::size_t index) noexcept {
        return (index + 1U) % Capacity;
    }

    std::array<T, Capacity> buffer_{};
    std::atomic<std::size_t> head_{0U}; // producer index
    std::atomic<std::size_t> tail_{0U}; // consumer index
};

} // namespace motion_core
