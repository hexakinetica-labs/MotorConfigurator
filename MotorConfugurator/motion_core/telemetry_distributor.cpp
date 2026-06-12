#include "motion_core/telemetry_distributor.h"

namespace motion_core {

TelemetryDistributor::TelemetryDistributor(HalRuntime& runtime) : runtime_(runtime) {}

TelemetryDistributor::~TelemetryDistributor() {}

void TelemetryDistributor::set_watched_axes(const std::vector<uint16_t>& axis_ids) {
    std::lock_guard<std::mutex> lock(axes_mutex_);
    watched_axes_ = axis_ids;
}

int TelemetryDistributor::register_callback(Callback cb) {
    std::lock_guard<std::mutex> lock(cb_mutex_);
    if (cb) {
        int token = next_token_++;
        callbacks_.push_back({token, std::move(cb)});
        return token;
    }
    return 0;
}

void TelemetryDistributor::unregister_callback(int token) {
    std::lock_guard<std::mutex> lock(cb_mutex_);
    for (auto it = callbacks_.begin(); it != callbacks_.end(); ++it) {
        if (it->first == token) {
            callbacks_.erase(it);
            break;
        }
    }
}

void TelemetryDistributor::tick_telemetry(double cycle_hz) {
    std::vector<uint16_t> current_axes;
    {
        std::lock_guard<std::mutex> lock(axes_mutex_);
        current_axes = watched_axes_;
    }

    for (uint16_t axis_id : current_axes) {
        auto axis_res = runtime_.find_axis(axis_id);
        if (!axis_res.ok()) continue;
        const auto& axis = axis_res.value();

        TelemetryFrame frame{};
        frame.axis_id = axis_id;
        frame.snapshot = axis->telemetry();
        frame.queue_stats = axis->query_motion_queue_stats();
        frame.transport = axis->info().transport;
        frame.cycle_hz = cycle_hz;

        std::lock_guard<std::mutex> lock(cb_mutex_);
        for (const auto& cb_pair : callbacks_) {
            cb_pair.second(frame);
        }
    }
}

} // namespace motion_core
