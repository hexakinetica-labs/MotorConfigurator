#pragma once

#include "hal_ipc/protocol.h"
#include "motion_core/result.h"

#include <cstdint>
#include <mutex>

namespace hal_host_service {

struct OwnershipSnapshot {
    hal_ipc::OwnerRole motion_owner{hal_ipc::OwnerRole::None};
    hal_ipc::OwnerRole service_owner{hal_ipc::OwnerRole::None};
    std::uint32_t motion_epoch{0};
    std::uint32_t service_epoch{0};
    bool manual_override_active{false};
    bool service_mode_active{false};
};

class AxisOwnershipArbiter final {
public:
    AxisOwnershipArbiter() = default;

    motion_core::Result<void> handle_lease_op(hal_ipc::OwnerRole caller, hal_ipc::ControlOp op);
    motion_core::Result<void> authorize_operation(hal_ipc::OwnerRole caller, hal_ipc::ControlOp op, std::uint32_t expected_epoch);

    [[nodiscard]] OwnershipSnapshot snapshot() const;
    void reset();

private:
    [[nodiscard]] static bool is_motion_command(hal_ipc::ControlOp op);
    [[nodiscard]] static bool is_service_command(hal_ipc::ControlOp op);
    [[nodiscard]] static bool is_readonly_query(hal_ipc::ControlOp op);

    mutable std::mutex mutex_{};
    OwnershipSnapshot state_{};
};

} // namespace hal_host_service
