#pragma once

#include "motion_core/result.h"

#include <cstdint>
#include <string>
#include <vector>

namespace hal_ipc {

constexpr std::uint32_t kClientIdHexaMotion = 1;
constexpr std::uint32_t kClientIdMotorTesterUi = 2;

enum class ControlOp : std::uint32_t {
    None = 0,
    StreamPoint = 1,
    Hold = 2,
    Stop = 3,
    RequestManualLease = 4,
    ReleaseManualLease = 5,
    EnableAxis = 6,
    DisableAxis = 7,
    SetZero = 8,
    ClearFault = 9,
    Home = 10,
    EnqueueMotionBatch = 11,
    SetAxisMode = 12,
    ConfigureMotionQueue = 13,
    ClearMotionQueue = 14,
    QueryMotionQueueStats = 15,
    ReadParameters = 16,
    ListParameters = 17,
    ApplyParameterPatch = 18,
    SetPersistent = 19,
    ImportAxisConfig = 20,
    ImportAxisConfigPreview = 21,
    ExportAxisConfig = 22
};

enum class OwnerRole : std::uint32_t {
    None = 0,
    HexaMotion = 1,
    MotorTesterUi = 2,
    Service = 3
};

struct AxisPointDto {
    std::uint16_t axis_id{0};
    bool has_interpolated_position{false};
    double interpolated_position_deg{0.0};
    bool has_interpolated_velocity{false};
    double interpolated_velocity_deg_s{0.0};

    double segment_start_deg{0.0};
    double segment_target_deg{0.0};
    double segment_duration_sec{0.0};

    double actual_position_deg{0.0};
    double actual_velocity_deg_s{0.0};
};

struct HalControlFrameDto {
    std::uint64_t seq{0};
    std::uint32_t client_id{0};
    std::uint32_t lease_epoch{0};
    ControlOp op{ControlOp::None};
    std::vector<AxisPointDto> axes{};
    std::int32_t service_axis_id{-1};
    bool service_bool_value{false};
    std::int32_t service_int_value{0};
    std::string service_string_value{};
};

struct HalStateFrameDto {
    std::uint64_t seq{0};
    std::uint64_t ack_control_seq{0};
    OwnerRole motion_owner{OwnerRole::None};
    OwnerRole service_owner{OwnerRole::None};
    std::uint32_t motion_epoch{0};
    std::uint32_t service_epoch{0};
    bool manual_override_active{false};
    bool service_mode_active{false};
    bool estop_active{false};
    std::vector<AxisPointDto> axes{};
    std::int32_t service_status_code{0};
    std::int32_t service_axis_id{-1};
    std::string service_string_response{};
};

motion_core::Result<std::string> serialize_control_frame(const HalControlFrameDto& frame);
motion_core::Result<std::string> serialize_state_frame(const HalStateFrameDto& frame);

motion_core::Result<HalControlFrameDto> deserialize_control_frame(const std::string& payload);
motion_core::Result<HalStateFrameDto> deserialize_state_frame(const std::string& payload);

} // namespace hal_ipc
