#pragma once

#include "motion_core/hal_runtime.h"
#include "motion_core/motion_orchestrator.h"
#include "motion_core/trajectory_generator.h"
#include "motion_core/telemetry_distributor.h"
#include "motion_core/parameter_manager.h"
#include "motion_core/config_manager.h"
#include "hal_ipc/ipc_controller/ipc_controller.h"

#include <thread>
#include <atomic>

namespace hal_host {

class HalHostService {
public:
    HalHostService();
    ~HalHostService();

    motion_core::Result<void> start(const motion_core::HalRuntimeConfig& config);
    motion_core::Result<void> stop();

    motion_core::HalRuntime& runtime();
    motion_core::MotionOrchestrator& orchestrator();
    motion_core::TrajectoryGenerator& trajectory_generator();
    motion_core::TelemetryDistributor& telemetry();
    motion_core::ParameterManager& parameters();
    motion_core::ConfigManager& config();
    hal_ipc::HalIpcController& ipc();

private:
    motion_core::HalRuntime runtime_;
    motion_core::MotionOrchestrator orchestrator_;
    motion_core::TelemetryDistributor telemetry_;
    motion_core::ParameterManager parameter_manager_;
    motion_core::ConfigManager config_manager_;
    motion_core::TrajectoryGenerator trajectory_generator_;
    hal_ipc::HalIpcController ipc_controller_;

    void loop_thread_fn();
    std::thread loop_thread_;
    std::atomic<bool> loop_running_{false};
    double cached_cycle_hz_{250.0};
};

} // namespace hal_host
