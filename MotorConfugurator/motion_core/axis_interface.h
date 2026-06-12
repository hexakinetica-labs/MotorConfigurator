#pragma once

#include "motion_core/axis_data.h"
#include "motion_core/parameter_types.h"
#include "motion_core/result.h"
#include "motion_core/types.h"

namespace motion_core {

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
};

class IAxis {
public:
    virtual ~IAxis() = default;

    [[nodiscard]] virtual AxisInfo info() const = 0;

    // Minimum Orchestration Contract
    virtual bool enqueueServicePoint(const ServiceCommandPoint& p) = 0;
    virtual bool enqueueCommandPoint(const MotionCommandPoint& point) = 0;

    virtual TelemetrySnapshot telemetry() const = 0;
    [[nodiscard]] virtual MotionQueueStats query_motion_queue_stats() const = 0;

    virtual void estop() = 0;

    virtual void setControlOwner(ControlOwner owner) = 0;

    // Parameter Tree (Kept per boss' instructions)
    [[nodiscard]] virtual Result<std::vector<ParameterDescriptor>> list_parameters() const = 0;
    [[nodiscard]] virtual Result<ParameterSet> read_parameters() const = 0;
    virtual Result<void> apply_parameter_patch(const ParameterPatch& patch) = 0;
    [[nodiscard]] virtual Result<PersistentWriteReport> set_persistent(PersistentCommand command,
                                                                       const ParameterValue& value) = 0;
};

} // namespace motion_core
