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
    AxisCycleMetrics() : last_snapshot_time_(std::chrono::steady_clock::now()) {}

    void mark_command_tx() noexcept { command_tx_.fetch_add(1U, std::memory_order_relaxed); }
    void mark_telemetry_publish() noexcept { telemetry_publish_.fetch_add(1U, std::memory_order_relaxed); }
    void mark_position_rx() noexcept { position_rx_.fetch_add(1U, std::memory_order_relaxed); }
    void mark_speed_rx() noexcept { speed_rx_.fetch_add(1U, std::memory_order_relaxed); }
    void mark_status_rx() noexcept { status_rx_.fetch_add(1U, std::memory_order_relaxed); }
    void mark_protection_rx() noexcept { protection_rx_.fetch_add(1U, std::memory_order_relaxed); }

    [[nodiscard]] AxisCycleMetricsSnapshot snapshot() const noexcept {
        AxisCycleMetricsSnapshot out{};
        out.command_tx.events = command_tx_.load(std::memory_order_relaxed);
        out.telemetry_publish.events = telemetry_publish_.load(std::memory_order_relaxed);
        out.position_rx.events = position_rx_.load(std::memory_order_relaxed);
        out.speed_rx.events = speed_rx_.load(std::memory_order_relaxed);
        out.status_rx.events = status_rx_.load(std::memory_order_relaxed);
        out.protection_rx.events = protection_rx_.load(std::memory_order_relaxed);

        const auto now = std::chrono::steady_clock::now();
        const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_snapshot_time_).count();

        if (elapsed >= 1000) {
            auto update_cache = [&](const std::uint64_t current, std::uint64_t& last, double& cached) {
                const auto delta = current - last;
                last = current;
                cached = static_cast<double>(delta) * 1000.0 / static_cast<double>(elapsed);
            };

            update_cache(out.command_tx.events, last_command_tx_, cached_command_tx_hz_);
            update_cache(out.telemetry_publish.events, last_telemetry_publish_, cached_telemetry_publish_hz_);
            update_cache(out.position_rx.events, last_position_rx_, cached_position_rx_hz_);
            update_cache(out.speed_rx.events, last_speed_rx_, cached_speed_rx_hz_);
            update_cache(out.status_rx.events, last_status_rx_, cached_status_rx_hz_);
            update_cache(out.protection_rx.events, last_protection_rx_, cached_protection_rx_hz_);

            last_snapshot_time_ = now;
        }

        out.command_tx.rate_hz = cached_command_tx_hz_;
        out.telemetry_publish.rate_hz = cached_telemetry_publish_hz_;
        out.position_rx.rate_hz = cached_position_rx_hz_;
        out.speed_rx.rate_hz = cached_speed_rx_hz_;
        out.status_rx.rate_hz = cached_status_rx_hz_;
        out.protection_rx.rate_hz = cached_protection_rx_hz_;

        return out;
    }

private:
    std::atomic<std::uint64_t> command_tx_{0};
    std::atomic<std::uint64_t> telemetry_publish_{0};
    std::atomic<std::uint64_t> position_rx_{0};
    std::atomic<std::uint64_t> speed_rx_{0};
    std::atomic<std::uint64_t> status_rx_{0};
    std::atomic<std::uint64_t> protection_rx_{0};

    mutable std::chrono::steady_clock::time_point last_snapshot_time_;
    mutable std::uint64_t last_command_tx_{0};
    mutable std::uint64_t last_telemetry_publish_{0};
    mutable std::uint64_t last_position_rx_{0};
    mutable std::uint64_t last_speed_rx_{0};
    mutable std::uint64_t last_status_rx_{0};
    mutable std::uint64_t last_protection_rx_{0};

    mutable double cached_command_tx_hz_{0.0};
    mutable double cached_telemetry_publish_hz_{0.0};
    mutable double cached_position_rx_hz_{0.0};
    mutable double cached_speed_rx_hz_{0.0};
    mutable double cached_status_rx_hz_{0.0};
    mutable double cached_protection_rx_hz_{0.0};
};

} // namespace mks
