#pragma once

#include "hal_ipc/server/server.h"
#include "motion_core/hal_runtime.h"
#include "motion_core/motion_orchestrator.h"
#include <string>

namespace hal_ipc {

class HalIpcController {
public:
    HalIpcController(motion_core::HalRuntime& runtime, motion_core::MotionOrchestrator& orchestrator);
    ~HalIpcController();

    motion_core::Result<void> start(const std::string& bind_host, std::uint16_t port, 
                                    HalIpcServer::DisconnectHandler on_disconnect = nullptr);
    motion_core::Result<void> stop();
    bool is_running() const;
    int connected_client_count() const;

private:
    HalStateFrameDto handle_control_frame(const HalControlFrameDto& frame);
    motion_core::Result<std::string> execute_axis_operation(
        OwnerRole caller, ControlOp op, uint16_t axis_id, const AxisPointDto* point, const HalControlFrameDto& frame);

    motion_core::HalRuntime& runtime_;
    motion_core::MotionOrchestrator& orchestrator_;
    HalIpcServer server_;
};

} // namespace hal_ipc
