#pragma once

#include "motion_core/axis_data.h"
#include "motion_core/parameter_types.h"
#include "motion_core/result.h"
#include "motion_core/types.h"

#include <vector>

namespace motion_core {

enum class Capability {
    ReadTelemetry = 0,
    SetTargetPosition,
    SetTargetVelocity,
    EnableDisable,
    EmergencyStop,
    ReadParameters,
    WriteParameters,
    ReadSnapshot,
    WriteSnapshot,
    Homing
};

enum class PersistentCommand {
    MotorType = 0,
    CanId,
    CanBitrate,
};

struct PersistentWriteReport {
    bool command_supported{false};
    bool unlock_performed{false};
    bool write_completed{false};
    bool persistent_save_completed{false};
    bool readback_verified{false};
    bool reconnect_required{false};
    bool power_cycle_required{false};
    ParameterValue readback_value{};
};

struct AxisInfo {
    AxisId id{};
    AxisName name{};
    AxisTransportKind transport{AxisTransportKind::Unknown};
    std::vector<Capability> capabilities{};
};

class IAxis {
public:
    virtual ~IAxis() = default;

    [[nodiscard]] virtual AxisInfo info() const = 0;

    virtual Result<void> start() = 0;
    virtual Result<void> stop() = 0;

    virtual Result<void> set_enabled(bool enabled) = 0;
    virtual Result<void> set_mode(AxisMode mode) = 0;
    virtual Result<void> apply_command(const AxisCommand& command) = 0;

    [[nodiscard]] virtual Result<AxisTelemetry> read_telemetry() const = 0;

    [[nodiscard]] virtual Result<std::vector<ParameterDescriptor>> list_parameters() const = 0;
    [[nodiscard]] virtual Result<ParameterSet> read_parameters() const = 0;
    virtual Result<void> apply_parameter_patch(const ParameterPatch& patch) = 0;
    [[nodiscard]] virtual Result<PersistentWriteReport> set_persistent(PersistentCommand command,
                                                                       const ParameterValue& value) = 0;

    virtual Result<void> configure_motion_queue(std::size_t capacity, bool drop_oldest) = 0;
    [[nodiscard]] virtual Result<MotionQueueStats> enqueue_motion_batch(
        const std::vector<QueuedSetpoint>& points) = 0;
    virtual Result<void> clear_motion_queue() = 0;
    [[nodiscard]] virtual Result<MotionQueueStats> query_motion_queue_stats() const = 0;
};

} // namespace motion_core
