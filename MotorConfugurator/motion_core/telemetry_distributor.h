#pragma once

#include "motion_core/axis_interface.h"
#include "motion_core/hal_runtime.h"
#include <chrono>
#include <functional>
#include <vector>

namespace motion_core {

struct TelemetryFrame {
    uint16_t axis_id{0};
    TelemetrySnapshot snapshot{};
    MotionQueueStats queue_stats{};
    AxisTransportKind transport{AxisTransportKind::Unknown};
    double cycle_hz{0.0};
};

class TelemetryDistributor {
public:
    using Callback = std::function<void(const TelemetryFrame&)>;

    explicit TelemetryDistributor(HalRuntime& runtime);
    ~TelemetryDistributor();

    void set_watched_axes(const std::vector<uint16_t>& axis_ids);
    int register_callback(Callback cb);
    void unregister_callback(int token);

    // Call this from a fast timer or polling loop
    void tick_telemetry(double cycle_hz);

private:
    HalRuntime& runtime_;
    std::vector<std::pair<int, Callback>> callbacks_;
    int next_token_{1};
    std::mutex cb_mutex_;
    std::vector<uint16_t> watched_axes_;
    std::mutex axes_mutex_;
};

} // namespace motion_core
