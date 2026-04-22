#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>

namespace mks {

struct AxisCycleChannelSnapshot {
    double rate_hz{0.0};
    double last_period_ms{0.0};
    double max_period_ms{0.0};
    std::uint64_t events{0U};
    std::uint64_t long_gaps{0U};
};

struct AxisCycleMetricsSnapshot {
    AxisCycleChannelSnapshot command_tx{};
    AxisCycleChannelSnapshot telemetry_publish{};
    AxisCycleChannelSnapshot position_rx{};
    AxisCycleChannelSnapshot speed_rx{};
    AxisCycleChannelSnapshot status_rx{};
    AxisCycleChannelSnapshot protection_rx{};
};

class AxisCycleMetrics final {
public:
    AxisCycleMetrics() = default;

    void mark_command_tx() noexcept { mark_channel(command_tx_); }
    void mark_telemetry_publish() noexcept { mark_channel(telemetry_publish_); }
    void mark_position_rx() noexcept { mark_channel(position_rx_); }
    void mark_speed_rx() noexcept { mark_channel(speed_rx_); }
    void mark_status_rx() noexcept { mark_channel(status_rx_); }
    void mark_protection_rx() noexcept { mark_channel(protection_rx_); }

    [[nodiscard]] AxisCycleMetricsSnapshot snapshot() const noexcept {
        AxisCycleMetricsSnapshot out{};
        out.command_tx = snapshot_channel(command_tx_);
        out.telemetry_publish = snapshot_channel(telemetry_publish_);
        out.position_rx = snapshot_channel(position_rx_);
        out.speed_rx = snapshot_channel(speed_rx_);
        out.status_rx = snapshot_channel(status_rx_);
        out.protection_rx = snapshot_channel(protection_rx_);
        return out;
    }

private:
    struct Channel final {
        std::atomic<std::int64_t> last_event_tenths_ms{0};
        std::atomic<std::int64_t> last_period_tenths_ms{0};
        std::atomic<std::int64_t> max_period_tenths_ms{0};
        std::atomic<std::int64_t> rate_tenths_hz{0};
        std::atomic<std::uint64_t> events{0U};
        std::atomic<std::uint64_t> long_gaps{0U};
    };

    static constexpr std::int64_t kLongGapThresholdTenthsMs = 500; // 50.0 ms

    [[nodiscard]] static std::int64_t now_tenths_ms() noexcept {
        const auto now = std::chrono::steady_clock::now().time_since_epoch();
        const auto microseconds = std::chrono::duration_cast<std::chrono::microseconds>(now).count();
        return static_cast<std::int64_t>(microseconds / 100);
    }

    static void update_max_period(std::atomic<std::int64_t>& current_max,
                                  const std::int64_t candidate) noexcept {
        auto observed = current_max.load(std::memory_order_acquire);
        while (candidate > observed) {
            if (current_max.compare_exchange_weak(
                    observed,
                    candidate,
                    std::memory_order_acq_rel,
                    std::memory_order_acquire)) {
                break;
            }
        }
    }

    static void mark_channel(Channel& channel) noexcept {
        const auto now = now_tenths_ms();
        const auto previous = channel.last_event_tenths_ms.exchange(now, std::memory_order_acq_rel);

        channel.events.fetch_add(1U, std::memory_order_acq_rel);

        if (previous <= 0 || now <= previous) {
            return;
        }

        const auto period_tenths_ms = now - previous;
        channel.last_period_tenths_ms.store(period_tenths_ms, std::memory_order_release);
        update_max_period(channel.max_period_tenths_ms, period_tenths_ms);

        if (period_tenths_ms >= kLongGapThresholdTenthsMs) {
            channel.long_gaps.fetch_add(1U, std::memory_order_acq_rel);
        }

        // Hz * 10 = 100000 / period_tenths_ms
        const auto rate_tenths_hz =
            static_cast<std::int64_t>((100000 + (period_tenths_ms / 2)) / period_tenths_ms);
        channel.rate_tenths_hz.store(rate_tenths_hz, std::memory_order_release);
    }

    [[nodiscard]] static AxisCycleChannelSnapshot snapshot_channel(const Channel& channel) noexcept {
        AxisCycleChannelSnapshot out{};
        out.rate_hz = static_cast<double>(channel.rate_tenths_hz.load(std::memory_order_acquire)) / 10.0;
        out.last_period_ms =
            static_cast<double>(channel.last_period_tenths_ms.load(std::memory_order_acquire)) / 10.0;
        out.max_period_ms =
            static_cast<double>(channel.max_period_tenths_ms.load(std::memory_order_acquire)) / 10.0;
        out.events = channel.events.load(std::memory_order_acquire);
        out.long_gaps = channel.long_gaps.load(std::memory_order_acquire);
        return out;
    }

    Channel command_tx_{};
    Channel telemetry_publish_{};
    Channel position_rx_{};
    Channel speed_rx_{};
    Channel status_rx_{};
    Channel protection_rx_{};
};

} // namespace mks
