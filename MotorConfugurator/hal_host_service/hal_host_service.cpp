#include "hal_host_service/hal_host_service.h"

#include <chrono>

namespace hal_host {

HalHostService::HalHostService()
    : runtime_(),
      orchestrator_(runtime_),
      telemetry_(runtime_),
      parameter_manager_(runtime_),
      config_manager_(runtime_),
      trajectory_generator_(orchestrator_),
      ipc_controller_(runtime_, orchestrator_) {
}

HalHostService::~HalHostService() {
    (void)stop();
}

motion_core::Result<void> HalHostService::start(const motion_core::HalRuntimeConfig& config) {
    auto res = runtime_.open_from_config(config);
    if (!res.ok()) {
        return res;
    }
    
    auto start_res = runtime_.start();
    if (!start_res.ok()) return start_res;

    if (!loop_running_) {
        loop_running_ = true;
        loop_thread_ = std::thread(&HalHostService::loop_thread_fn, this);
    }

    if (!ipc_controller_.is_running()) {
        // Start HexaMotion IPC
        auto ipc_res = ipc_controller_.start("127.0.0.1", 5555, [](){});
        if (!ipc_res.ok()) {
            // Depending on requirements, we could fail here, but usually IPC failure 
            // shouldn't block the UI runtime from operating.
        }
    }

    return motion_core::Result<void>::success();
}

motion_core::Result<void> HalHostService::stop() {
    if (ipc_controller_.is_running()) {
        (void)ipc_controller_.stop();
    }
    
    if (loop_running_) {
        loop_running_ = false;
        if (loop_thread_.joinable()) {
            loop_thread_.join();
        }
    }

    motion_core::Result<void> result = motion_core::Result<void>::success();
    if (runtime_.is_active()) {
        auto res = runtime_.stop();
        if (!res.ok()) {
            result = res;
        }
    }
    if (runtime_.is_open()) {
        auto close_res = runtime_.close();
        if (result.ok() && !close_res.ok()) {
            result = close_res;
        }
    }
    return result;
}

void HalHostService::loop_thread_fn() {
    auto next_wake = std::chrono::steady_clock::now();
    
    while (loop_running_) {
        const auto period = std::chrono::microseconds(
            static_cast<long long>(1000000.0 / (cached_cycle_hz_ > 0.0 ? cached_cycle_hz_ : 250.0)));
        next_wake += period;
        
        orchestrator_.tick_homing_sequence();
        trajectory_generator_.update_all_queues();
        telemetry_.tick_telemetry(cached_cycle_hz_);
        
        std::this_thread::sleep_until(next_wake);
    }
}

motion_core::HalRuntime& HalHostService::runtime() {
    return runtime_;
}

motion_core::MotionOrchestrator& HalHostService::orchestrator() {
    return orchestrator_;
}

motion_core::TrajectoryGenerator& HalHostService::trajectory_generator() {
    return trajectory_generator_;
}

motion_core::TelemetryDistributor& HalHostService::telemetry() {
    return telemetry_;
}

motion_core::ParameterManager& HalHostService::parameters() {
    return parameter_manager_;
}

motion_core::ConfigManager& HalHostService::config() {
    return config_manager_;
}

hal_ipc::HalIpcController& HalHostService::ipc() {
    return ipc_controller_;
}

} // namespace hal_host
