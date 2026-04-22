#pragma once

#include "hal_ipc/protocol.h"
#include "hal_ipc/server.h"
#include "motion_core/config/hal_runtime_config.h"
#include "motion_core/hal_runtime.h"
#include "motion_core/result.h"

#include <atomic>
#include <cstdint>
#include <functional>
#include <mutex>
#include <string>

namespace hal_host_service {

struct HalHostServiceConfig {
    std::string bind_host{"127.0.0.1"};
    std::uint16_t port{30110};
};

class HalHostService {
public:
    using AxisOperationHandler = std::function<motion_core::Result<std::string>(
        hal_ipc::OwnerRole caller,
        hal_ipc::ControlOp op,
        std::uint16_t axis_id,
        const hal_ipc::AxisPointDto* point,
        const hal_ipc::HalControlFrameDto& frame)>;

    struct HostStateSnapshot {
        hal_ipc::OwnerRole motion_owner{hal_ipc::OwnerRole::None};
        bool manual_override_active{false};
        bool estop_active{false};
        int connected_client_count{0};
        int connected_hexamotion_client_count{0};
        bool ipc_server_running{false};
    };

    explicit HalHostService(motion_core::HalRuntime& runtime) : runtime_(runtime) {}
    ~HalHostService();

    HalHostService(const HalHostService&) = delete;
    HalHostService& operator=(const HalHostService&) = delete;

    motion_core::Result<void> start(const HalHostServiceConfig& config);
    motion_core::Result<void> stop();

    [[nodiscard]] bool is_running() const noexcept;

    void set_axis_operation_handler(AxisOperationHandler handler);
    void set_state_provider(std::function<HostStateSnapshot()> provider);
    
    [[nodiscard]] HostStateSnapshot state_snapshot() const;

private:
    [[nodiscard]] hal_ipc::HalStateFrameDto handle_control_frame(const hal_ipc::HalControlFrameDto& frame);
    motion_core::Result<std::string> apply_control_frame(const hal_ipc::HalControlFrameDto& frame);
    motion_core::Result<std::string> apply_axis_op(hal_ipc::OwnerRole caller,
                                                   hal_ipc::ControlOp op,
                                                   std::uint16_t axis_id,
                                                   const hal_ipc::AxisPointDto* point,
                                                   const hal_ipc::HalControlFrameDto& frame);
    void refresh_state_from_runtime(hal_ipc::HalStateFrameDto& state) const;

    mutable std::mutex mutex_{};
    motion_core::HalRuntime& runtime_;
    hal_ipc::HalIpcServer server_{};
    std::atomic<bool> running_{false};

    std::shared_ptr<AxisOperationHandler> axis_operation_handler_{};
    std::shared_ptr<std::function<HostStateSnapshot()>> state_provider_{};

    std::atomic<std::uint64_t> state_seq_{0};
};

} // namespace hal_host_service
