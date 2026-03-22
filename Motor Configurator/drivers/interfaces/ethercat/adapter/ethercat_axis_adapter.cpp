#include "ethercat/adapter/ethercat_axis_adapter.h"
#include "ethercat/p100e_ethercat_dictionary.h"

#include <algorithm>
#include <cmath>
#include <chrono>
#include <thread>
#include <array>
#include <cstring>
#include <limits>

namespace ethercat_driver {

namespace {

constexpr auto kSdoRequestTimeout = std::chrono::milliseconds(250);
constexpr auto kEepromSaveSettleTime = std::chrono::milliseconds(200);
constexpr std::array<std::size_t, 4> kSupportedSdoWriteSizes{1U, 2U, 4U, 8U};

[[nodiscard]] ec_sdo_request_t* create_sdo_request_with_timeout(
    ec_slave_config_t* slave_config,
    const std::size_t size) {
    if (!slave_config) {
        return nullptr;
    }
    auto* request = ecrt_slave_config_create_sdo_request(slave_config, 0x0000, 0x00, size);
    if (!request) {
        return nullptr;
    }
    if (ecrt_sdo_request_timeout(request, static_cast<std::uint32_t>(kSdoRequestTimeout.count())) != 0) {
        return nullptr;
    }
    return request;
}

} // namespace

constexpr uint32_t DVS_VENDOR_ID   = 0x00445653;
constexpr uint32_t DVS_PRODUCT_CODE = 0x00009252;

constexpr uint16_t ObjControlword         = 0x6040;
constexpr uint16_t ObjTargetPosition      = 0x607A;
constexpr uint16_t ObjModesOfOperation    = 0x6060;
constexpr uint16_t ObjMaxProfileVelocity  = 0x607F;
constexpr uint16_t ObjStatusword          = 0x6041;
constexpr uint16_t ObjActualPosition      = 0x6064;
constexpr uint16_t ObjErrorCode           = 0x603F;
constexpr uint16_t ObjActualTorque        = 0x6077;
constexpr uint16_t ObjTargetVelocity      = 0x60FF;

constexpr uint16_t CmdShutdown      = 0x0006;
constexpr uint16_t CmdSwitchOn      = 0x0007;
constexpr uint16_t CmdEnable        = 0x000F;
constexpr uint16_t CmdQuickStop     = 0x000B;
constexpr uint16_t CmdFaultReset    = 0x0080;

constexpr uint16_t MaskFault            = 0x004F;
constexpr uint16_t ValFault             = 0x0008;
constexpr uint16_t MaskOperationEnabled = 0x006F;
constexpr uint16_t ValOperationEnabled  = 0x0027;
constexpr uint16_t MaskReadyToSwitchOn  = 0x006F;
constexpr uint16_t ValReadyToSwitchOn   = 0x0021;
constexpr uint16_t MaskSwitchedOn       = 0x006F;
constexpr uint16_t ValSwitchedOn        = 0x0023;
constexpr uint16_t MaskSwitchOnDisabled = 0x004F;
constexpr uint16_t ValSwitchOnDisabled  = 0x0040;
constexpr uint16_t MaskHomingAttained   = 0x1000;
constexpr uint16_t MaskHomingError      = 0x2000;
constexpr double kDefaultInterpolationVelocityDegPerSec = 30.0;

[[nodiscard]] bool parameter_id_equals(const motion_core::ParameterId lhs,
                                       const motion_core::ParameterId rhs) {
    return lhs.domain == rhs.domain && lhs.value == rhs.value;
}

[[nodiscard]] motion_core::ParameterValue decode_raw_parameter_value(
    const ParameterDefinition& definition,
    const std::uint8_t* buffer) {
    if (definition.type == motion_core::ParameterValueType::SignedInteger) {
        std::int64_t signed_value = 0;
        if (definition.data_size == 1U) {
            signed_value = static_cast<std::int8_t>(buffer[0]);
        } else if (definition.data_size == 2U) {
            std::int16_t tmp = 0;
            std::memcpy(&tmp, buffer, sizeof(tmp));
            signed_value = tmp;
        } else if (definition.data_size == 4U) {
            std::int32_t tmp = 0;
            std::memcpy(&tmp, buffer, sizeof(tmp));
            signed_value = tmp;
        } else if (definition.data_size == 8U) {
            std::int64_t tmp = 0;
            std::memcpy(&tmp, buffer, sizeof(tmp));
            signed_value = tmp;
        } else {
            return motion_core::ParameterValue::from_signed(0);
        }
        return motion_core::ParameterValue::from_signed(signed_value);
    }

    if (definition.type == motion_core::ParameterValueType::UnsignedInteger) {
        std::uint64_t unsigned_value = 0;
        if (definition.data_size == 1U) {
            unsigned_value = buffer[0];
        } else if (definition.data_size == 2U) {
            std::uint16_t tmp = 0;
            std::memcpy(&tmp, buffer, sizeof(tmp));
            unsigned_value = tmp;
        } else if (definition.data_size == 4U) {
            std::uint32_t tmp = 0;
            std::memcpy(&tmp, buffer, sizeof(tmp));
            unsigned_value = tmp;
        } else if (definition.data_size == 8U) {
            std::uint64_t tmp = 0;
            std::memcpy(&tmp, buffer, sizeof(tmp));
            unsigned_value = tmp;
        } else {
            return motion_core::ParameterValue::from_unsigned(0U);
        }
        return motion_core::ParameterValue::from_unsigned(unsigned_value);
    }

    if (definition.type == motion_core::ParameterValueType::FloatingPoint) {
        double floating_value = 0.0;
        if (definition.data_size == 4U) {
            float tmp = 0.0F;
            std::memcpy(&tmp, buffer, sizeof(tmp));
            floating_value = static_cast<double>(tmp);
        } else if (definition.data_size >= 8U) {
            std::memcpy(&floating_value, buffer, sizeof(floating_value));
        }
        return motion_core::ParameterValue::from_floating(floating_value);
    }

    return motion_core::ParameterValue::from_bool(buffer[0] != 0U);
}

[[nodiscard]] motion_core::Result<void> encode_raw_parameter_value(
    const ParameterDefinition& definition,
    const motion_core::ParameterValue& raw_value,
    std::uint8_t* out_buffer,
    const std::size_t out_size) {
    if (!out_buffer || out_size < definition.data_size) {
        return motion_core::Result<void>::failure(
            {motion_core::ErrorCode::InvalidArgument, "encode_raw_parameter_value: output buffer is too small"});
    }

    std::memset(out_buffer, 0, out_size);

    if (definition.type == motion_core::ParameterValueType::SignedInteger) {
        std::int64_t value = 0;
        if (raw_value.type == motion_core::ParameterValueType::SignedInteger) {
            value = raw_value.signed_value;
        } else if (raw_value.type == motion_core::ParameterValueType::UnsignedInteger) {
            value = static_cast<std::int64_t>(raw_value.unsigned_value);
        } else if (raw_value.type == motion_core::ParameterValueType::FloatingPoint) {
            value = static_cast<std::int64_t>(std::llround(raw_value.floating_value));
        } else {
            value = raw_value.bool_value ? 1 : 0;
        }

        if (definition.data_size == 1U) {
            const auto clipped = static_cast<std::int8_t>(std::clamp<std::int64_t>(
                value,
                static_cast<std::int64_t>(std::numeric_limits<std::int8_t>::min()),
                static_cast<std::int64_t>(std::numeric_limits<std::int8_t>::max())));
            std::memcpy(out_buffer, &clipped, sizeof(clipped));
        } else if (definition.data_size == 2U) {
            const auto clipped = static_cast<std::int16_t>(std::clamp<std::int64_t>(
                value,
                static_cast<std::int64_t>(std::numeric_limits<std::int16_t>::min()),
                static_cast<std::int64_t>(std::numeric_limits<std::int16_t>::max())));
            std::memcpy(out_buffer, &clipped, sizeof(clipped));
        } else if (definition.data_size == 4U) {
            const auto clipped = static_cast<std::int32_t>(std::clamp<std::int64_t>(
                value,
                static_cast<std::int64_t>(std::numeric_limits<std::int32_t>::min()),
                static_cast<std::int64_t>(std::numeric_limits<std::int32_t>::max())));
            std::memcpy(out_buffer, &clipped, sizeof(clipped));
        } else if (definition.data_size == 8U) {
            std::int64_t clipped = value;
            std::memcpy(out_buffer, &clipped, sizeof(clipped));
        } else {
            return motion_core::Result<void>::failure(
                {motion_core::ErrorCode::InvalidArgument, "encode_raw_parameter_value: unsupported signed data size"});
        }
        return motion_core::Result<void>::success();
    }

    if (definition.type == motion_core::ParameterValueType::UnsignedInteger) {
        std::uint64_t value = 0;
        if (raw_value.type == motion_core::ParameterValueType::UnsignedInteger) {
            value = raw_value.unsigned_value;
        } else if (raw_value.type == motion_core::ParameterValueType::SignedInteger) {
            value = raw_value.signed_value < 0 ? 0U : static_cast<std::uint64_t>(raw_value.signed_value);
        } else if (raw_value.type == motion_core::ParameterValueType::FloatingPoint) {
            value = raw_value.floating_value < 0.0
                        ? 0U
                        : static_cast<std::uint64_t>(std::llround(raw_value.floating_value));
        } else {
            value = raw_value.bool_value ? 1U : 0U;
        }

        if (definition.data_size == 1U) {
            const auto clipped = static_cast<std::uint8_t>(std::min<std::uint64_t>(
                value,
                static_cast<std::uint64_t>(std::numeric_limits<std::uint8_t>::max())));
            std::memcpy(out_buffer, &clipped, sizeof(clipped));
        } else if (definition.data_size == 2U) {
            const auto clipped = static_cast<std::uint16_t>(std::min<std::uint64_t>(
                value,
                static_cast<std::uint64_t>(std::numeric_limits<std::uint16_t>::max())));
            std::memcpy(out_buffer, &clipped, sizeof(clipped));
        } else if (definition.data_size == 4U) {
            const auto clipped = static_cast<std::uint32_t>(std::min<std::uint64_t>(
                value,
                static_cast<std::uint64_t>(std::numeric_limits<std::uint32_t>::max())));
            std::memcpy(out_buffer, &clipped, sizeof(clipped));
        } else if (definition.data_size == 8U) {
            const std::uint64_t clipped = value;
            std::memcpy(out_buffer, &clipped, sizeof(clipped));
        } else {
            return motion_core::Result<void>::failure(
                {motion_core::ErrorCode::InvalidArgument, "encode_raw_parameter_value: unsupported unsigned data size"});
        }
        return motion_core::Result<void>::success();
    }

    if (definition.type == motion_core::ParameterValueType::FloatingPoint) {
        if (definition.data_size == 4U) {
            float clipped = 0.0F;
            if (raw_value.type == motion_core::ParameterValueType::FloatingPoint) {
                clipped = static_cast<float>(raw_value.floating_value);
            } else if (raw_value.type == motion_core::ParameterValueType::SignedInteger) {
                clipped = static_cast<float>(raw_value.signed_value);
            } else if (raw_value.type == motion_core::ParameterValueType::UnsignedInteger) {
                clipped = static_cast<float>(raw_value.unsigned_value);
            } else {
                clipped = raw_value.bool_value ? 1.0F : 0.0F;
            }
            std::memcpy(out_buffer, &clipped, sizeof(clipped));
        } else {
            double clipped = 0.0;
            if (raw_value.type == motion_core::ParameterValueType::FloatingPoint) {
                clipped = raw_value.floating_value;
            } else if (raw_value.type == motion_core::ParameterValueType::SignedInteger) {
                clipped = static_cast<double>(raw_value.signed_value);
            } else if (raw_value.type == motion_core::ParameterValueType::UnsignedInteger) {
                clipped = static_cast<double>(raw_value.unsigned_value);
            } else {
                clipped = raw_value.bool_value ? 1.0 : 0.0;
            }
            std::memcpy(out_buffer, &clipped, sizeof(clipped));
        }
        return motion_core::Result<void>::success();
    }

    out_buffer[0] = (raw_value.type == motion_core::ParameterValueType::Boolean)
                        ? (raw_value.bool_value ? 1U : 0U)
                        : 0U;
    return motion_core::Result<void>::success();
}

static ec_pdo_entry_info_t dgn_pdo_entries[] = {
    {ObjControlword, 0x00, 16},
    {ObjTargetPosition, 0x00, 32},
    {ObjTargetVelocity, 0x00, 32},
    {ObjModesOfOperation, 0x00, 8},
    {ObjMaxProfileVelocity, 0x00, 32},
    {ObjStatusword, 0x00, 16},
    {ObjActualPosition, 0x00, 32},
    {ObjErrorCode, 0x00, 16},
    {ObjActualTorque, 0x00, 16},
};

static ec_pdo_info_t dgn_pdos[] = {
    {0x1600, 5, dgn_pdo_entries + 0}, 
    {0x1A00, 4, dgn_pdo_entries + 5}, 
};

static ec_sync_info_t dgn_syncs[] = {
    {0, EC_DIR_OUTPUT, 0, nullptr, EC_WD_DISABLE},
    {1, EC_DIR_INPUT,  0, nullptr, EC_WD_DISABLE},
    {2, EC_DIR_OUTPUT, 1, dgn_pdos + 0, EC_WD_ENABLE},
    {3, EC_DIR_INPUT,  1, dgn_pdos + 1, EC_WD_DISABLE},
    {0xFF, EC_DIR_OUTPUT, 0, nullptr, EC_WD_DISABLE}
};

EthercatAxisAdapter::EthercatAxisAdapter(EthercatAxisAdapterConfig config)
    : config_(std::move(config)) {
    recalculate_counts_per_radian();
}
    
EthercatAxisAdapter::~EthercatAxisAdapter() = default;

motion_core::AxisInfo EthercatAxisAdapter::info() const {
    motion_core::AxisInfo info{};
    info.id = config_.axis_id;
    info.name = config_.axis_name;
    info.transport = motion_core::AxisTransportKind::Ethercat;
    info.capabilities = {
        motion_core::Capability::ReadTelemetry,
        motion_core::Capability::SetTargetPosition,
        motion_core::Capability::SetTargetVelocity,
        motion_core::Capability::EnableDisable,
        motion_core::Capability::EmergencyStop,
        motion_core::Capability::ReadParameters,
        motion_core::Capability::WriteParameters,
    };
    return info;
}

motion_core::Result<void> EthercatAxisAdapter::configure_hardware() {
    auto* master = config_.bus_manager->master();
    auto* domain = config_.bus_manager->domain();
    if (!master || !domain) {
        return motion_core::Result<void>::failure(
            {motion_core::ErrorCode::NotConnected, "EtherCAT bus is not initialized"});
    }

    ec_slave_config_t* sc = ecrt_master_slave_config(master, 0, config_.ecat_bus_position, DVS_VENDOR_ID, DVS_PRODUCT_CODE);
    if (!sc) {
        return motion_core::Result<void>::failure(
            {motion_core::ErrorCode::TransportFailure, "Failed to get slave config"});
    }
    slave_config_ = sc;

    if (ecrt_slave_config_pdos(sc, EC_END, dgn_syncs)) {
        return motion_core::Result<void>::failure(
            {motion_core::ErrorCode::TransportFailure, "Failed to configure slave PDOs"});
    }

    // One bit-position slot per registered PDO entry below.
    // Keep the size in sync with the number of domain_regs entries that use bit_positions[i].
    std::array<unsigned int, 9> bit_positions{};

    ec_pdo_entry_reg_t domain_regs[] = {
        {0,
         static_cast<uint16_t>(config_.ecat_bus_position),
         DVS_VENDOR_ID,
         DVS_PRODUCT_CODE,
         ObjControlword,
         0,
         &off_ctrl_,
         &bit_positions[0]},
        {0,
         static_cast<uint16_t>(config_.ecat_bus_position),
         DVS_VENDOR_ID,
         DVS_PRODUCT_CODE,
         ObjTargetPosition,
         0,
         &off_target_pos_,
         &bit_positions[1]},
        {0,
         static_cast<uint16_t>(config_.ecat_bus_position),
         DVS_VENDOR_ID,
         DVS_PRODUCT_CODE,
         ObjModesOfOperation,
         0,
         &off_modes_op_,
         &bit_positions[2]},
        {0,
         static_cast<uint16_t>(config_.ecat_bus_position),
         DVS_VENDOR_ID,
         DVS_PRODUCT_CODE,
         ObjMaxProfileVelocity,
         0,
         &off_max_vel_,
         &bit_positions[3]},
        {0,
         static_cast<uint16_t>(config_.ecat_bus_position),
         DVS_VENDOR_ID,
         DVS_PRODUCT_CODE,
         ObjStatusword,
         0,
         &off_status_,
         &bit_positions[4]},
        {0,
         static_cast<uint16_t>(config_.ecat_bus_position),
         DVS_VENDOR_ID,
         DVS_PRODUCT_CODE,
         ObjActualPosition,
         0,
         &off_act_pos_,
         &bit_positions[5]},
        {0,
         static_cast<uint16_t>(config_.ecat_bus_position),
         DVS_VENDOR_ID,
         DVS_PRODUCT_CODE,
         ObjErrorCode,
         0,
         &off_error_,
         &bit_positions[6]},
        {0,
         static_cast<uint16_t>(config_.ecat_bus_position),
         DVS_VENDOR_ID,
         DVS_PRODUCT_CODE,
         ObjActualTorque,
         0,
         &off_act_torque_,
         &bit_positions[7]},
        {0,
         static_cast<uint16_t>(config_.ecat_bus_position),
         DVS_VENDOR_ID,
         DVS_PRODUCT_CODE,
         ObjTargetVelocity,
         0,
         &off_target_vel_,
         &bit_positions[8]},
        {0, 0, 0, 0, 0, 0, nullptr, nullptr}
    };

    if (ecrt_domain_reg_pdo_entry_list(domain, domain_regs)) {
        return motion_core::Result<void>::failure(
            {motion_core::ErrorCode::TransportFailure, "PDO entry registration failed"});
    }

    read_sdo_request_ = create_sdo_request_with_timeout(slave_config_, 8U);
    if (!read_sdo_request_) {
        return motion_core::Result<void>::failure(
            {motion_core::ErrorCode::TransportFailure, "Failed to create EtherCAT async SDO read request"});
    }
    for (std::size_t i = 0; i < kSupportedSdoWriteSizes.size(); ++i) {
        write_sdo_requests_[i] = create_sdo_request_with_timeout(slave_config_, kSupportedSdoWriteSizes[i]);
        if (!write_sdo_requests_[i]) {
            return motion_core::Result<void>::failure(
                {motion_core::ErrorCode::TransportFailure, "Failed to create EtherCAT async SDO write request"});
        }
    }

    config_.bus_manager->register_adapter(config_.ecat_axis_index, this);

    // Read axial resolution (0x6091:02) — instruction units per motor revolution.
    // This is the scale for all PDO position objects (0x6064, 0x607A).
    // Default is 10000. Do NOT use encoder_resolution_bits_ for this.
    if (const auto upload = config_.bus_manager->sdo_upload(config_.ecat_bus_position, 0x6091, 2, sizeof(std::uint32_t));
        upload.ok() && upload.value().size() == sizeof(std::uint32_t)) {
        std::uint32_t axial_res = 0;
        std::memcpy(&axial_res, upload.value().data(), sizeof(axial_res));
        if (axial_res > 0) {
            counts_per_revolution_.store(axial_res, std::memory_order_release);
            recalculate_counts_per_radian();
        }
    }

    if (const auto upload = config_.bus_manager->sdo_upload(
            config_.ecat_bus_position,
            ObjMaxProfileVelocity,
            0,
            sizeof(std::uint32_t));
        upload.ok() && upload.value().size() == sizeof(std::uint32_t)) {
        std::uint32_t max_profile_vel = 0;
        std::memcpy(&max_profile_vel, upload.value().data(), sizeof(max_profile_vel));
        if (max_profile_vel > 0) {
            max_profile_velocity_instr_s_.store(max_profile_vel, std::memory_order_release);
            has_max_profile_velocity_instr_s_.store(true, std::memory_order_release);
        }
    }

    return motion_core::Result<void>::success();
}

motion_core::Result<void> EthercatAxisAdapter::start() {
    has_last_position_sample_.store(false, std::memory_order_release);
    cmd_.homing_active.store(false, std::memory_order_release);
    clear_motion_queue_req_.store(true, std::memory_order_release);
    has_queued_target_.store(false, std::memory_order_release);
    interpolator_initialized_ = false;
    started_.store(true, std::memory_order_release);
    return motion_core::Result<void>::success();
}

motion_core::Result<void> EthercatAxisAdapter::stop() {
    cmd_.enable_req.store(false, std::memory_order_release);
    cmd_.homing_active.store(false, std::memory_order_release);
    has_last_position_sample_.store(false, std::memory_order_release);
    clear_motion_queue_req_.store(true, std::memory_order_release);
    has_queued_target_.store(false, std::memory_order_release);
    interpolator_initialized_ = false;
    started_.store(false, std::memory_order_release);
    return motion_core::Result<void>::success();
}

motion_core::Result<void> EthercatAxisAdapter::set_enabled(bool enabled) {
    auto status = ensure_started();
    if (!status.ok()) return status;
    
    if (enabled && !cmd_.enable_req.load(std::memory_order_relaxed)) {
        // Sync target before enabling (bumpless transfer), but only after the
        // first valid telemetry sample has arrived from the RT cycle.
        if (telem_.timestamp_ns.load(std::memory_order_acquire) != 0U) {
            cmd_.target_pos_deg.store(
                telem_.actual_position_deg.load(std::memory_order_acquire),
                std::memory_order_release);
        }
    }
    cmd_.enable_req.store(enabled, std::memory_order_release);
    if (!enabled) {
        cmd_.homing_active.store(false, std::memory_order_release);
    }
    return motion_core::Result<void>::success();
}

motion_core::Result<void> EthercatAxisAdapter::set_mode(motion_core::AxisMode mode) {
    auto status = ensure_started();
    if (!status.ok()) return status;
    int8_t work_mode = mode_to_work_mode(mode);
    if (work_mode < 0) {
        return motion_core::Result<void>::failure(
            {motion_core::ErrorCode::Unsupported, "Mode not supported"});
    }
    cmd_.mode_req.store(work_mode, std::memory_order_release);
    mode_.store(mode, std::memory_order_release);
    if (mode != motion_core::AxisMode::Homing) {
        cmd_.homing_active.store(false, std::memory_order_release);
    }
    return motion_core::Result<void>::success();
}

motion_core::Result<void> EthercatAxisAdapter::apply_command(const motion_core::AxisCommand& command) {
    auto status = ensure_started();
    if (!status.ok()) return status;

    if (command.emergency_stop) {
        cmd_.enable_req.store(false, std::memory_order_release);
        cmd_.homing_active.store(false, std::memory_order_release);
    }
    if (command.clear_errors) {
        cmd_.reset_req.store(true, std::memory_order_release);
    }
    if (command.go_home) {
        cmd_.mode_req.store(mode_to_work_mode(motion_core::AxisMode::Homing), std::memory_order_release);
        mode_.store(motion_core::AxisMode::Homing, std::memory_order_release);
        cmd_.homing_req.store(true, std::memory_order_release);
        cmd_.homing_active.store(true, std::memory_order_release);
    }
    if (command.set_zero) {
        cmd_.set_zero_req.store(true, std::memory_order_release);
    }
    if (command.has_target_velocity) {
        cmd_.has_target_vel.store(true, std::memory_order_release);
        cmd_.target_vel_deg_s.store(command.target_velocity_deg_per_sec, std::memory_order_release);
    } else {
        cmd_.has_target_vel.store(false, std::memory_order_release);
    }
    if (command.has_profile_speed_rpm) {
        cmd_.has_profile_vel.store(true, std::memory_order_release);
        cmd_.profile_vel_rpm.store(command.profile_speed_rpm, std::memory_order_release);
    } else {
        cmd_.has_profile_vel.store(false, std::memory_order_release);
    }
    if (command.has_target_position) {
        const auto mode = mode_.load(std::memory_order_acquire);
        if (mode != motion_core::AxisMode::ProfilePosition &&
            mode != motion_core::AxisMode::CyclicSyncPosition &&
            mode != motion_core::AxisMode::CyclicSyncVelocity) {
            return motion_core::Result<void>::failure(
                {motion_core::ErrorCode::Unsupported, "Current EtherCAT mode does not accept position targets"});
        }
        double target_deg = command.target_position_deg;
        if (command.is_relative) {
            target_deg += telem_.actual_position_deg.load(std::memory_order_acquire);
        }
        if (mode == motion_core::AxisMode::CyclicSyncPosition ||
            mode == motion_core::AxisMode::CyclicSyncVelocity) {
            motion_core::QueuedSetpoint point{};
            point.target_position_deg = target_deg;
            point.has_profile_speed_rpm = command.has_profile_speed_rpm;
            point.profile_speed_rpm = command.profile_speed_rpm;
            point.has_profile_accel_percent = command.has_profile_accel_percent;
            point.profile_accel_percent = command.profile_accel_percent;
            point.has_target_velocity = command.has_target_velocity;
            point.target_velocity_deg_per_sec = command.target_velocity_deg_per_sec;

            const auto capacity_limit = motion_queue_capacity_limit_.load(std::memory_order_acquire);
            const bool drop_oldest = motion_queue_drop_oldest_policy_.load(std::memory_order_acquire);
            const bool capacity_reached = motion_queue_.size_approx() >= capacity_limit;
            if (capacity_reached) {
                motion_points_dropped_.fetch_add(1U, std::memory_order_relaxed);
                if (drop_oldest) {
                    queued_target_position_deg_.store(point.target_position_deg, std::memory_order_release);
                    has_queued_target_.store(true, std::memory_order_release);
                }
            } else if (motion_queue_.try_push(point)) {
                motion_points_pushed_.fetch_add(1U, std::memory_order_relaxed);
            } else {
                motion_points_dropped_.fetch_add(1U, std::memory_order_relaxed);
                if (drop_oldest) {
                    queued_target_position_deg_.store(point.target_position_deg, std::memory_order_release);
                    has_queued_target_.store(true, std::memory_order_release);
                }
            }
        } else {
            cmd_.target_pos_deg.store(target_deg, std::memory_order_release);
        }
    }
    return motion_core::Result<void>::success();
}

motion_core::Result<motion_core::AxisTelemetry> EthercatAxisAdapter::read_telemetry() const {
    auto status = ensure_started();
    if (!status.ok()) return motion_core::Result<motion_core::AxisTelemetry>::failure(status.error());

    motion_core::AxisTelemetry out{};
    out.timestamp_ns = telem_.timestamp_ns.load(std::memory_order_acquire);
    out.actual_position_deg = telem_.actual_position_deg.load(std::memory_order_acquire);
    out.actual_velocity_deg_per_sec = telem_.actual_velocity_deg_s.load(std::memory_order_acquire);
    out.actual_torque_percent = telem_.actual_torque_pct.load(std::memory_order_acquire);
    out.target_position_deg = cmd_.target_pos_deg.load(std::memory_order_acquire);
    out.status_word = telem_.statusword.load(std::memory_order_acquire);
    out.protection_code = telem_.error_code.load(std::memory_order_acquire);
    out.motion_status_code = 0U;
    out.mode = mode_.load(std::memory_order_acquire);

    if (telem_.is_faulted.load(std::memory_order_acquire)) {
        out.state = motion_core::AxisState::Fault;
    } else if (telem_.is_enabled.load(std::memory_order_acquire)) {
        out.state = motion_core::AxisState::OperationEnabled;
    } else if (telem_.is_ready.load(std::memory_order_acquire)) {
        out.state = motion_core::AxisState::Ready;
    } else if ((out.status_word & MaskSwitchOnDisabled) == ValSwitchOnDisabled) {
        out.state = motion_core::AxisState::Disabled;
    } else {
        out.state = motion_core::AxisState::Unknown;
    }

    return motion_core::Result<motion_core::AxisTelemetry>::success(std::move(out));
}

std::vector<motion_core::ParameterDescriptor> EthercatAxisAdapter::make_parameter_descriptors() const {
    std::vector<motion_core::ParameterDescriptor> out;
    out.reserve(p100e_dictionary.size() + 2);

    EthercatConversionContext conversion_context{};
    conversion_context.gear_ratio = gear_ratio_.load(std::memory_order_acquire);
    conversion_context.axial_resolution_instr_per_motor_rev =
        counts_per_revolution_.load(std::memory_order_acquire);

    // Build descriptors from the authoritative p100e_dictionary.
    for (const auto& def : p100e_dictionary) {
        motion_core::ParameterDescriptor d{};
        d.id = def.id;
        d.name = def.name;
        d.group = def.group;
        const auto conversion_kind = parse_conversion_kind(def.conversion_id);
        const bool has_conversion = conversion_kind.ok()
            && conversion_kind.value() != EthercatConversionKind::NoConversion;

        d.unit = has_conversion && def.display_unit && def.display_unit[0] != '\0'
            ? def.display_unit
            : def.unit;
        d.read_only = def.is_read_only;
        d.persistable = def.persistable_runtime;
        d.has_min = true;
        d.has_max = true;
        d.raw_unit = def.raw_unit;
        d.display_unit = def.display_unit;
        d.conversion_id = def.conversion_id;
        d.semantic_scope = def.semantic_scope;
        d.conversion_formula = def.conversion_formula;
        d.conversion_depends_on_gear_ratio = def.conversion_depends_on_gear_ratio;
        d.conversion_depends_on_axial_resolution = def.conversion_depends_on_axial_resolution;

        if (has_conversion) {
            const auto converted_min = convert_from_raw_to_display(def, def.min_value, conversion_context);
            const auto converted_max = convert_from_raw_to_display(def, def.max_value, conversion_context);
            d.min_value = converted_min.ok() ? converted_min.value() : def.min_value;
            d.max_value = converted_max.ok() ? converted_max.value() : def.max_value;
        } else {
            d.min_value = def.min_value;
            d.max_value = def.max_value;
        }
        out.push_back(d);
    }

    // Gear ratio is a local config parameter, not an SDO in the dictionary.
    motion_core::ParameterDescriptor gear{};
    gear.id = motion_core::make_parameter_id(motion_core::CommonParameter::HardwareGearRatio);
    gear.name = "Gear Ratio";
    gear.group = "Common/Mechanics";
    gear.unit = "motor_turns_per_output_turn";
    gear.read_only = false;
    gear.persistable = true;
    gear.has_min = true;
    gear.has_max = true;
    gear.min_value = motion_core::ParameterValue::from_floating(0.001);
    gear.max_value = motion_core::ParameterValue::from_floating(1000.0);
    out.push_back(gear);

    return out;
}

motion_core::Result<std::vector<motion_core::ParameterDescriptor>> EthercatAxisAdapter::list_parameters() const {
    return motion_core::Result<std::vector<motion_core::ParameterDescriptor>>::success(make_parameter_descriptors());
}

motion_core::Result<motion_core::ParameterSet> EthercatAxisAdapter::read_parameters() const {
    auto status = ensure_started();
    if (!status.ok()) return motion_core::Result<motion_core::ParameterSet>::failure(status.error());

    motion_core::ParameterSet out{};
    EthercatConversionContext conversion_context{};
    conversion_context.gear_ratio = gear_ratio_.load(std::memory_order_acquire);
    conversion_context.axial_resolution_instr_per_motor_rev =
        counts_per_revolution_.load(std::memory_order_acquire);

    // Read all SDO entries from the dictionary.
    for (const auto& def : p100e_dictionary) {
        const auto uploaded = request_sdo_read(def.index, def.sub_index, def.data_size);
        if (!uploaded.ok()) {
            continue; // skip on SDO error
        }
        const auto raw_value = decode_raw_parameter_value(def, uploaded.value().data());
        const auto display_value = convert_from_raw_to_display(def, raw_value, conversion_context);
        if (display_value.ok()) {
            out.entries.push_back({def.id, display_value.value()});
        } else {
            out.entries.push_back({def.id, raw_value});
        }
    }

    // Gear ratio: local config, no SDO.
    out.entries.push_back({
        motion_core::make_parameter_id(motion_core::CommonParameter::HardwareGearRatio),
        motion_core::ParameterValue::from_floating(gear_ratio_.load(std::memory_order_acquire))
    });

    return motion_core::Result<motion_core::ParameterSet>::success(std::move(out));
}

motion_core::Result<void> EthercatAxisAdapter::apply_parameter_patch(const motion_core::ParameterPatch& patch) {
    auto status = ensure_started();
    if (!status.ok()) return status;

    const auto save_parameter_id =
        motion_core::make_parameter_id(EthercatParameter::SaveParametersToEeprom);
    bool has_save_trigger = false;
    motion_core::ParameterValue save_trigger_value = motion_core::ParameterValue::from_signed(1);

    for (const auto& entry : patch.entries) {
        if (entry.id.domain == motion_core::ParameterDomain::Common) {
            const auto common = static_cast<motion_core::CommonParameter>(entry.id.value);
            if (common == motion_core::CommonParameter::HardwareGearRatio) {
                double ratio = 0.0;
                if (entry.value.type == motion_core::ParameterValueType::FloatingPoint)
                    ratio = entry.value.floating_value;
                else if (entry.value.type == motion_core::ParameterValueType::UnsignedInteger)
                    ratio = static_cast<double>(entry.value.unsigned_value);
                else if (entry.value.type == motion_core::ParameterValueType::SignedInteger)
                    ratio = static_cast<double>(entry.value.signed_value);
                else {
                    return motion_core::Result<void>::failure(
                        {motion_core::ErrorCode::InvalidArgument,
                         "HardwareGearRatio requires numeric value"});
                }

                if (!(ratio > 0.0) || !std::isfinite(ratio)) {
                    return motion_core::Result<void>::failure(
                        {motion_core::ErrorCode::InvalidArgument,
                         "HardwareGearRatio must be finite and > 0"});
                }

                gear_ratio_.store(ratio, std::memory_order_release);
                recalculate_counts_per_radian();
            }
            continue;
        }

        if (entry.id.domain != motion_core::ParameterDomain::Ethercat) {
            continue;
        }

        if (parameter_id_equals(entry.id, save_parameter_id)) {
            has_save_trigger = true;
            save_trigger_value = entry.value;
            continue;
        }

        const ParameterDefinition* def = find_ethercat_parameter_definition(entry.id);
        if (!def || def->is_read_only) continue;

        EthercatConversionContext conversion_context{};
        conversion_context.gear_ratio = gear_ratio_.load(std::memory_order_acquire);
        conversion_context.axial_resolution_instr_per_motor_rev =
            counts_per_revolution_.load(std::memory_order_acquire);

        const auto raw_value = convert_from_display_to_raw(*def, entry.value, conversion_context);
        if (!raw_value.ok()) {
            return motion_core::Result<void>::failure(raw_value.error());
        }

        // Marshal value into little-endian bytes.
        std::array<std::uint8_t, 8> buf{};
        const auto encode_status =
            encode_raw_parameter_value(*def, raw_value.value(), buf.data(), buf.size());
        if (!encode_status.ok()) {
            return encode_status;
        }

        const auto payload = std::vector<std::uint8_t>(buf.data(), buf.data() + def->data_size);
        const auto download_status = request_sdo_write(def->index, def->sub_index, payload);
        if (!download_status.ok()) {
            return motion_core::Result<void>::failure(download_status.error());
        }

        // If this was AxialResolution, update local scale.
        if (entry.id.value == static_cast<uint32_t>(EthercatParameter::AxialResolution)
            && def->data_size == sizeof(std::uint32_t)) {
            uint32_t new_res = 0;
            std::memcpy(&new_res, buf.data(), sizeof(new_res));
            if (new_res > 0) {
                counts_per_revolution_.store(new_res, std::memory_order_release);
                recalculate_counts_per_radian();
            }
        }

        if (entry.id.value == static_cast<uint32_t>(EthercatParameter::MaxProfileVelocityCountsPerSec)
            && def->data_size == sizeof(std::uint32_t)) {
            uint32_t max_profile_vel = 0;
            std::memcpy(&max_profile_vel, buf.data(), sizeof(max_profile_vel));
            if (max_profile_vel > 0) {
                max_profile_velocity_instr_s_.store(max_profile_vel, std::memory_order_release);
                has_max_profile_velocity_instr_s_.store(true, std::memory_order_release);
            }
        }
    }

    if (has_save_trigger) {
        const ParameterDefinition* save_definition = find_ethercat_parameter_definition(save_parameter_id);
        if (!save_definition || save_definition->is_read_only) {
            return motion_core::Result<void>::failure(
                {motion_core::ErrorCode::Unsupported,
                 "EtherCAT save-to-EEPROM trigger is not available"});
        }

        // Unconditionally write '0' then '1' to 0x2005:03 (uint16) to ensure edge trigger for autosave
        const std::uint16_t save_reset_value = 0;
        std::vector<std::uint8_t> reset_payload(sizeof(save_reset_value), 0U);
        std::memcpy(reset_payload.data(), &save_reset_value, sizeof(save_reset_value));
        
        (void)config_.bus_manager->sdo_download(
            config_.ecat_bus_position,
            save_definition->index,
            save_definition->sub_index,
            reset_payload);

        std::this_thread::sleep_for(std::chrono::milliseconds(50));

        const std::uint16_t save_to_eeprom_value = 1;
        std::vector<std::uint8_t> save_payload(sizeof(save_to_eeprom_value), 0U);
        std::memcpy(save_payload.data(), &save_to_eeprom_value, sizeof(save_to_eeprom_value));

        const auto save_status = config_.bus_manager->sdo_download(
            config_.ecat_bus_position,
            save_definition->index,
            save_definition->sub_index,
            save_payload);

        if (!save_status.ok()) {
            return motion_core::Result<void>::failure(save_status.error());
        }

        std::this_thread::sleep_for(kEepromSaveSettleTime);
    }

    return motion_core::Result<void>::success();
}

motion_core::Result<motion_core::PersistentWriteReport> EthercatAxisAdapter::set_persistent(
    const motion_core::PersistentCommand command,
    const motion_core::ParameterValue& value) {
    const auto status = ensure_started();
    if (!status.ok()) {
        return motion_core::Result<motion_core::PersistentWriteReport>::failure(status.error());
    }

    if (command != motion_core::PersistentCommand::MotorType) {
        return motion_core::Result<motion_core::PersistentWriteReport>::failure(
            {motion_core::ErrorCode::Unsupported,
             "Persistent command is not supported by EtherCAT adapter"});
    }

    std::int64_t motor_type_code = 0;
    if (value.type == motion_core::ParameterValueType::SignedInteger) {
        motor_type_code = value.signed_value;
    } else if (value.type == motion_core::ParameterValueType::UnsignedInteger) {
        motor_type_code = static_cast<std::int64_t>(value.unsigned_value);
    } else {
        return motion_core::Result<motion_core::PersistentWriteReport>::failure(
            {motion_core::ErrorCode::InvalidArgument,
             "MotorType persistent command expects integer value"});
    }

    if (motor_type_code < 40 || motor_type_code > 180) {
        return motion_core::Result<motion_core::PersistentWriteReport>::failure(
            {motion_core::ErrorCode::InvalidArgument,
             "MotorType value is out of allowed range [40..180]"});
    }

    motion_core::PersistentWriteReport report{};
    report.command_supported = true;

    const auto motor_type_value = static_cast<std::int16_t>(motor_type_code);
    std::vector<std::uint8_t> motor_type_payload(sizeof(motor_type_value), 0U);
    std::memcpy(motor_type_payload.data(), &motor_type_value, sizeof(motor_type_value));

    const auto write_status = config_.bus_manager->sdo_download(
        config_.ecat_bus_position,
        0x2007,
        0x01,
        motor_type_payload);
    if (!write_status.ok()) {
        return motion_core::Result<motion_core::PersistentWriteReport>::failure(write_status.error());
    }
    report.write_completed = true;

    // Edge trigger for EEPROM save: Write 0, short delay, write 1
    const auto reset_value = static_cast<std::int16_t>(0);
    std::vector<std::uint8_t> reset_payload(sizeof(reset_value), 0U);
    std::memcpy(reset_payload.data(), &reset_value, sizeof(reset_value));
    
    (void)config_.bus_manager->sdo_download(
        config_.ecat_bus_position,
        0x2005,
        0x03,
        reset_payload);
        
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    const auto save_to_eeprom_value = static_cast<std::int16_t>(1);
    std::vector<std::uint8_t> save_to_eeprom_payload(sizeof(save_to_eeprom_value), 0U);
    std::memcpy(save_to_eeprom_payload.data(), &save_to_eeprom_value, sizeof(save_to_eeprom_value));

    const auto save_status = config_.bus_manager->sdo_download(
        config_.ecat_bus_position,
        0x2005,
        0x03,
        save_to_eeprom_payload);
        
    if (!save_status.ok()) {
        return motion_core::Result<motion_core::PersistentWriteReport>::failure(save_status.error());
    }
    report.persistent_save_completed = true;

    std::this_thread::sleep_for(kEepromSaveSettleTime);

    const auto readback = config_.bus_manager->sdo_upload(
        config_.ecat_bus_position,
        0x2007,
        0x01,
        sizeof(std::int16_t));
    if (!readback.ok()) {
        return motion_core::Result<motion_core::PersistentWriteReport>::failure(readback.error());
    }
    if (readback.value().size() < sizeof(std::int16_t)) {
        return motion_core::Result<motion_core::PersistentWriteReport>::failure(
            {motion_core::ErrorCode::ProtocolFailure,
             "EtherCAT MotorType readback payload is too small"});
    }

    std::int16_t readback_motor_type = 0;
    std::memcpy(&readback_motor_type, readback.value().data(), sizeof(readback_motor_type));
    report.readback_value = motion_core::ParameterValue::from_signed(readback_motor_type);
    report.readback_verified = (readback_motor_type == motor_type_value);
    report.power_cycle_required = true;

    if (!report.readback_verified) {
        return motion_core::Result<motion_core::PersistentWriteReport>::failure(
            {motion_core::ErrorCode::ProtocolFailure,
             "EtherCAT MotorType readback does not match requested value"});
    }

    return motion_core::Result<motion_core::PersistentWriteReport>::success(report);
}

motion_core::Result<void> EthercatAxisAdapter::configure_motion_queue(
    const std::size_t capacity,
    const bool drop_oldest) {
    if (capacity == 0U) {
        return motion_core::Result<void>::failure(
            {motion_core::ErrorCode::InvalidArgument, "Motion queue capacity must be > 0"});
    }

    motion_queue_capacity_limit_.store(
        std::min<std::size_t>(capacity, kMotionQueuePhysicalCapacity),
        std::memory_order_release);
    motion_queue_drop_oldest_policy_.store(drop_oldest, std::memory_order_release);
    return motion_core::Result<void>::success();
}

motion_core::Result<motion_core::MotionQueueStats> EthercatAxisAdapter::enqueue_motion_batch(
    const std::vector<motion_core::QueuedSetpoint>& points) {
    const auto status = ensure_started();
    if (!status.ok()) {
        return motion_core::Result<motion_core::MotionQueueStats>::failure(status.error());
    }

    std::uint64_t pushed = 0U;
    std::uint64_t dropped = 0U;
    const auto capacity_limit = motion_queue_capacity_limit_.load(std::memory_order_acquire);
    const bool drop_oldest = motion_queue_drop_oldest_policy_.load(std::memory_order_acquire);

    for (const auto& point : points) {
        const auto current_size = motion_queue_.size_approx();
        if (current_size >= capacity_limit) {
            ++dropped;
            if (drop_oldest) {
                queued_target_position_deg_.store(point.target_position_deg, std::memory_order_release);
                has_queued_target_.store(true, std::memory_order_release);
            }
            continue;
        }

        if (motion_queue_.try_push(point)) {
            ++pushed;
        } else {
            ++dropped;
            if (drop_oldest) {
                queued_target_position_deg_.store(point.target_position_deg, std::memory_order_release);
                has_queued_target_.store(true, std::memory_order_release);
            }
        }
    }

    motion_points_pushed_.fetch_add(pushed, std::memory_order_relaxed);
    motion_points_dropped_.fetch_add(dropped, std::memory_order_relaxed);

    return query_motion_queue_stats();
}

motion_core::Result<void> EthercatAxisAdapter::clear_motion_queue() {
    clear_motion_queue_req_.store(true, std::memory_order_release);
    has_queued_target_.store(false, std::memory_order_release);
    return motion_core::Result<void>::success();
}

motion_core::Result<motion_core::MotionQueueStats> EthercatAxisAdapter::query_motion_queue_stats() const {
    motion_core::MotionQueueStats stats{};
    stats.size = motion_queue_.size_approx();
    if (has_queued_target_.load(std::memory_order_acquire)) {
        ++stats.size;
    }
    stats.capacity = motion_queue_capacity_limit_.load(std::memory_order_acquire);
    stats.pushed = motion_points_pushed_.load(std::memory_order_acquire);
    stats.dropped = motion_points_dropped_.load(std::memory_order_acquire);
    return motion_core::Result<motion_core::MotionQueueStats>::success(stats);
}

void EthercatAxisAdapter::process_cycle(uint8_t* domain_pd, double dt_s) {
    if (!domain_pd) return;

    // --- Read Process Data ---
    uint16_t statusword = EC_READ_U16(domain_pd + off_status_);
    int32_t act_pos_counts = EC_READ_S32(domain_pd + off_act_pos_);
    int16_t act_torque = EC_READ_S16(domain_pd + off_act_torque_);
    uint16_t err_code = 0;
    
    bool is_faulted = false;
    bool is_ready = false;
    bool is_enabled = false;

    if ((statusword & MaskFault) == ValFault) {
        is_faulted = true;
        err_code = EC_READ_U16(domain_pd + off_error_);
    } else if ((statusword & MaskOperationEnabled) == ValOperationEnabled) {
        is_enabled = true;
    } else if ((statusword & MaskSwitchedOn) == ValSwitchedOn || 
               (statusword & MaskReadyToSwitchOn) == ValReadyToSwitchOn) {
        is_ready = true;
    }

    bool velocity_resync_required = false;
    const bool set_zero_executed = cmd_.set_zero_req.exchange(false, std::memory_order_relaxed);
    if (set_zero_executed) {
        zero_offset_counts_.store(act_pos_counts, std::memory_order_release);
        cmd_.target_pos_deg.store(0.0, std::memory_order_release);
        cmd_.target_pos_counts.store(act_pos_counts, std::memory_order_release);
        cmd_.has_target_vel.store(false, std::memory_order_release);
        cmd_.target_vel_deg_s.store(0.0, std::memory_order_release);
        cmd_.has_profile_vel.store(false, std::memory_order_release);
        cmd_.profile_vel_rpm.store(0, std::memory_order_release);
        clear_motion_queue_req_.store(true, std::memory_order_release);
        has_queued_target_.store(false, std::memory_order_release);
        queued_target_position_deg_.store(0.0, std::memory_order_release);
        interpolator_initialized_ = false;
        interpolated_target_deg_ = 0.0;
        velocity_resync_required = true;
    }

    const auto zero_offset_counts = zero_offset_counts_.load(std::memory_order_acquire);
    const auto counts_per_radian = counts_per_radian_.load(std::memory_order_acquire);

    double pos_rad = static_cast<double>(act_pos_counts - zero_offset_counts) / counts_per_radian;
    double pos_deg = pos_rad * (180.0 / M_PI);

    // Calc velocity using cycle DT
    double velocity_deg_s = 0.0;
    if (!has_last_position_sample_.load(std::memory_order_acquire)
        || velocity_resync_required
        || !(dt_s > 0.0)) {
        velocity_deg_s = 0.0;
        has_last_position_sample_.store(true, std::memory_order_release);
    } else {
        velocity_deg_s = (pos_deg - last_position_deg_) / dt_s;
    }
    last_position_deg_ = pos_deg;
    const auto now_ns = static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now().time_since_epoch())
            .count());
    last_timestamp_ns_ = now_ns;

    // Update Telemetry Atomically
    telem_.statusword.store(statusword, std::memory_order_release);
    telem_.error_code.store(err_code, std::memory_order_release);
    telem_.is_faulted.store(is_faulted, std::memory_order_release);
    telem_.is_ready.store(is_ready, std::memory_order_release);
    telem_.is_enabled.store(is_enabled, std::memory_order_release);
    telem_.actual_position_counts.store(act_pos_counts, std::memory_order_release);
    telem_.actual_position_deg.store(pos_deg, std::memory_order_release);
    telem_.actual_torque_pct.store(act_torque / 10.0, std::memory_order_release);
    telem_.actual_velocity_deg_s.store(velocity_deg_s, std::memory_order_release);
    telem_.timestamp_ns.store(now_ns, std::memory_order_release);

    // --- Write Process Data ---
    bool enable_req = cmd_.enable_req.load(std::memory_order_acquire);
    bool reset_req = cmd_.reset_req.exchange(false, std::memory_order_relaxed); // one-shot
    bool homing_req = cmd_.homing_req.exchange(false, std::memory_order_relaxed);
    bool homing_active = cmd_.homing_active.load(std::memory_order_acquire);
    int8_t mode = cmd_.mode_req.load(std::memory_order_acquire);
    
    uint16_t controlword = 0;
    
    if (reset_req) {
         controlword = CmdFaultReset;
    } else if (enable_req) {
         if (is_faulted) {
             // wait for manual reset
         } else if ((statusword & MaskSwitchOnDisabled) == ValSwitchOnDisabled) {
             controlword = CmdShutdown;
         } else if ((statusword & MaskReadyToSwitchOn) == ValReadyToSwitchOn) {
             controlword = CmdSwitchOn;
         } else if ((statusword & MaskSwitchedOn) == ValSwitchedOn) {
             controlword = CmdEnable;
             EC_WRITE_S8(domain_pd + off_modes_op_, mode);
         } else if (is_enabled) {
             controlword = CmdEnable;
             EC_WRITE_S8(domain_pd + off_modes_op_, mode);
         } else {
             controlword = CmdShutdown;
         }
    } else {
         if (is_enabled || (statusword & MaskSwitchedOn) == ValSwitchedOn || (statusword & MaskReadyToSwitchOn) == ValReadyToSwitchOn) {
             controlword = CmdQuickStop;
         }
    }

    if (homing_req) {
        homing_active = true;
    }
    if (mode != 6 || !enable_req || is_faulted) {
        homing_active = false;
    }
    if ((statusword & MaskHomingError) != 0U) {
        homing_active = false;
    }
    if ((statusword & MaskHomingAttained) != 0U) {
        homing_active = false;
    }

    if (mode == 6 && homing_active && is_enabled) {
        controlword |= 0x0010; // Homing Operation Start (Bit 4)
    }
    cmd_.homing_active.store(homing_active, std::memory_order_release);
    
    EC_WRITE_U16(domain_pd + off_ctrl_, controlword);
    cmd_.controlword_last_sent.store(controlword, std::memory_order_release);

    if (clear_motion_queue_req_.exchange(false, std::memory_order_acquire)) {
        motion_core::QueuedSetpoint ignored{};
        while (motion_queue_.try_pop(ignored)) {
        }
        has_queued_target_.store(false, std::memory_order_release);
    }

    // Target Position (with RT interpolation for CSP/CSV)
    double target_deg = cmd_.target_pos_deg.load(std::memory_order_acquire);
    if (mode == 8 || mode == 9) {
        motion_core::QueuedSetpoint queued{};
        bool has_new_target = false;
        if (motion_queue_.try_pop(queued)) {
            has_new_target = true;
        } else if (has_queued_target_.exchange(false, std::memory_order_acquire)) {
            queued.target_position_deg = queued_target_position_deg_.load(std::memory_order_acquire);
            has_new_target = true;
        }

        if (has_new_target) {
            cmd_.target_pos_deg.store(queued.target_position_deg, std::memory_order_release);
            if (queued.has_target_velocity) {
                cmd_.has_target_vel.store(true, std::memory_order_release);
                cmd_.target_vel_deg_s.store(queued.target_velocity_deg_per_sec, std::memory_order_release);
            }
            if (queued.has_profile_speed_rpm) {
                cmd_.has_profile_vel.store(true, std::memory_order_release);
                cmd_.profile_vel_rpm.store(queued.profile_speed_rpm, std::memory_order_release);
            }
        }

        const double endpoint_deg = cmd_.target_pos_deg.load(std::memory_order_acquire);
        if (!is_enabled || is_faulted || !interpolator_initialized_) {
            interpolated_target_deg_ = pos_deg;
            interpolator_initialized_ = true;
            cmd_.target_pos_deg.store(pos_deg, std::memory_order_relaxed);
            target_deg = pos_deg;
        } else {
            double interpolation_velocity_deg_s = kDefaultInterpolationVelocityDegPerSec;
            if (cmd_.has_target_vel.load(std::memory_order_acquire)) {
                interpolation_velocity_deg_s = std::max(
                    0.1,
                    std::abs(cmd_.target_vel_deg_s.load(std::memory_order_acquire)));
            } else if (cmd_.has_profile_vel.load(std::memory_order_acquire)) {
                double gear_ratio = gear_ratio_.load(std::memory_order_acquire);
                if (!(gear_ratio > 0.0) || !std::isfinite(gear_ratio)) {
                    gear_ratio = 1.0;
                }
                interpolation_velocity_deg_s = std::max(
                    0.1,
                    (static_cast<double>(cmd_.profile_vel_rpm.load(std::memory_order_acquire)) * 6.0) / gear_ratio);
            }

            const double effective_dt = (dt_s > 0.0 && std::isfinite(dt_s)) ? dt_s : 0.004;
            const double max_step_deg = interpolation_velocity_deg_s * effective_dt;
            const double delta_deg = endpoint_deg - interpolated_target_deg_;
            const double step_deg = std::clamp(delta_deg, -max_step_deg, max_step_deg);
            interpolated_target_deg_ += step_deg;
            target_deg = interpolated_target_deg_;
        }
    } else {
        interpolator_initialized_ = false;
    }

    double target_rad = target_deg * (M_PI / 180.0);
    int32_t target_counts = static_cast<int32_t>(target_rad * counts_per_radian) + zero_offset_counts;
    
    cmd_.target_pos_counts.store(target_counts, std::memory_order_release);
    EC_WRITE_S32(domain_pd + off_target_pos_, target_counts);
    
    // Target Velocity
    if (cmd_.has_target_vel.load(std::memory_order_acquire)) {
        double vel_deg_s = cmd_.target_vel_deg_s.load(std::memory_order_acquire);
        double vel_rad_s = vel_deg_s * (M_PI / 180.0);
        int32_t target_vel_counts = static_cast<int32_t>(vel_rad_s * counts_per_radian);
        EC_WRITE_S32(domain_pd + off_target_vel_, target_vel_counts);
    } else {
        EC_WRITE_S32(domain_pd + off_target_vel_, 0);
    }

    // Profile Velocity (Max Vel)
    if (cmd_.has_profile_vel.load(std::memory_order_acquire)) {
        double rps = cmd_.profile_vel_rpm.load(std::memory_order_acquire) / 60.0;
        double counts_per_rev_output = counts_per_revolution_.load(std::memory_order_acquire) * gear_ratio_.load(std::memory_order_acquire);
        uint32_t profile_vel_instr_s = static_cast<uint32_t>(rps * counts_per_rev_output);
        EC_WRITE_U32(domain_pd + off_max_vel_, profile_vel_instr_s);
    } else {
        const bool has_max_profile_velocity = has_max_profile_velocity_instr_s_.load(std::memory_order_acquire);
        const uint32_t max_profile_velocity = max_profile_velocity_instr_s_.load(std::memory_order_acquire);
        if (has_max_profile_velocity && max_profile_velocity > 0U) {
            EC_WRITE_U32(domain_pd + off_max_vel_, max_profile_velocity);
        }
    }
}

motion_core::Result<void> EthercatAxisAdapter::ensure_started() const {
    if (!started_.load(std::memory_order_acquire)) {
        return motion_core::Result<void>::failure({motion_core::ErrorCode::NotConnected, "Axis not started"});
    }
    return motion_core::Result<void>::success();
}

motion_core::Result<std::vector<std::uint8_t>> EthercatAxisAdapter::request_sdo_read(
    const std::uint16_t index,
    const std::uint8_t sub_index,
    const std::size_t expected_size) const {
    if (!read_sdo_request_) {
        return motion_core::Result<std::vector<std::uint8_t>>::failure(
            {motion_core::ErrorCode::InternalError, "EtherCAT async SDO read request is not initialized"});
    }

    std::lock_guard<std::mutex> lock(sdo_request_mutex_);
    if (ecrt_sdo_request_index(read_sdo_request_, index, sub_index) != 0) {
        return motion_core::Result<std::vector<std::uint8_t>>::failure(
            {motion_core::ErrorCode::InternalError, "EtherCAT async SDO read request index update failed"});
    }
    if (ecrt_sdo_request_read(read_sdo_request_) != 0) {
        return motion_core::Result<std::vector<std::uint8_t>>::failure(
            {motion_core::ErrorCode::TransportFailure, "EtherCAT async SDO read request start failed"});
    }

    const auto deadline = std::chrono::steady_clock::now() + kSdoRequestTimeout;
    while (true) {
        const auto state = ecrt_sdo_request_state(read_sdo_request_);
        if (state == EC_REQUEST_SUCCESS) {
            break;
        }
        if (state == EC_REQUEST_ERROR) {
            return motion_core::Result<std::vector<std::uint8_t>>::failure(
                {motion_core::ErrorCode::ProtocolFailure, "EtherCAT async SDO read request failed"});
        }
        if (std::chrono::steady_clock::now() >= deadline) {
            return motion_core::Result<std::vector<std::uint8_t>>::failure(
                {motion_core::ErrorCode::Timeout, "EtherCAT async SDO read request timed out"});
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    const auto data_size = ecrt_sdo_request_data_size(read_sdo_request_);
    auto* data = ecrt_sdo_request_data(read_sdo_request_);
    if (!data || data_size == 0U) {
        return motion_core::Result<std::vector<std::uint8_t>>::failure(
            {motion_core::ErrorCode::ProtocolFailure, "EtherCAT async SDO read returned empty payload"});
    }

    std::vector<std::uint8_t> out(expected_size, 0U);
    std::memcpy(out.data(), data, std::min(expected_size, data_size));
    return motion_core::Result<std::vector<std::uint8_t>>::success(std::move(out));
}

ec_sdo_request_t* EthercatAxisAdapter::write_request_for_size(const std::size_t payload_size) const {
    for (std::size_t i = 0; i < kSupportedSdoWriteSizes.size(); ++i) {
        if (kSupportedSdoWriteSizes[i] == payload_size) {
            return write_sdo_requests_[i];
        }
    }
    return nullptr;
}

motion_core::Result<void> EthercatAxisAdapter::request_sdo_write(
    const std::uint16_t index,
    const std::uint8_t sub_index,
    const std::vector<std::uint8_t>& payload) {
    auto* request = write_request_for_size(payload.size());
    if (!request) {
        return motion_core::Result<void>::failure(
            {motion_core::ErrorCode::InvalidArgument, "EtherCAT async SDO write payload size is unsupported"});
    }

    std::lock_guard<std::mutex> lock(sdo_request_mutex_);
    if (ecrt_sdo_request_index(request, index, sub_index) != 0) {
        return motion_core::Result<void>::failure(
            {motion_core::ErrorCode::InternalError, "EtherCAT async SDO write request index update failed"});
    }

    auto* data = ecrt_sdo_request_data(request);
    if (!data) {
        return motion_core::Result<void>::failure(
            {motion_core::ErrorCode::InternalError, "EtherCAT async SDO write request buffer is null"});
    }
    std::memset(data, 0, payload.size());
    std::memcpy(data, payload.data(), payload.size());

    if (ecrt_sdo_request_write(request) != 0) {
        return motion_core::Result<void>::failure(
            {motion_core::ErrorCode::TransportFailure, "EtherCAT async SDO write request start failed"});
    }

    const auto deadline = std::chrono::steady_clock::now() + kSdoRequestTimeout;
    while (true) {
        const auto state = ecrt_sdo_request_state(request);
        if (state == EC_REQUEST_SUCCESS) {
            return motion_core::Result<void>::success();
        }
        if (state == EC_REQUEST_ERROR) {
            return motion_core::Result<void>::failure(
                {motion_core::ErrorCode::ProtocolFailure, "EtherCAT async SDO write request failed"});
        }
        if (std::chrono::steady_clock::now() >= deadline) {
            return motion_core::Result<void>::failure(
                {motion_core::ErrorCode::Timeout, "EtherCAT async SDO write request timed out"});
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}

void EthercatAxisAdapter::recalculate_counts_per_radian() {
    const double counts_per_rev_output =
        static_cast<double>(counts_per_revolution_.load(std::memory_order_acquire))
        * gear_ratio_.load(std::memory_order_acquire);
    double counts_per_radian = counts_per_rev_output / (2.0 * M_PI);
    if (!(counts_per_radian > 0.0) || !std::isfinite(counts_per_radian)) {
        counts_per_radian = 1.0;
    }
    counts_per_radian_.store(counts_per_radian, std::memory_order_release);
}

int8_t EthercatAxisAdapter::mode_to_work_mode(motion_core::AxisMode mode) const {
    switch (mode) {
        case motion_core::AxisMode::ProfilePosition: return 1;    // PP
        case motion_core::AxisMode::ProfileVelocity: return 3;    // PV
        case motion_core::AxisMode::Homing: return 6;             // HM
        case motion_core::AxisMode::CyclicSyncPosition: return 8; // CSP
        case motion_core::AxisMode::CyclicSyncVelocity: return 9; // CSV
        case motion_core::AxisMode::CyclicSyncTorque: return 10;  // CST
        default: return -1;
    }
}

std::shared_ptr<motion_core::IAxis> make_ethercat_axis_adapter(EthercatAxisAdapterConfig config) {
    return std::make_shared<EthercatAxisAdapter>(std::move(config));
}

motion_core::Result<std::shared_ptr<motion_core::IAxis>>
make_configured_ethercat_axis_adapter(EthercatAxisAdapterConfig config) {
    auto adapter = std::make_shared<EthercatAxisAdapter>(std::move(config));
    const auto configured = adapter->configure_hardware();
    if (!configured.ok()) {
        return motion_core::Result<std::shared_ptr<motion_core::IAxis>>::failure(configured.error());
    }
    std::shared_ptr<motion_core::IAxis> axis = adapter;
    return motion_core::Result<std::shared_ptr<motion_core::IAxis>>::success(std::move(axis));
}

} // namespace ethercat_driver
