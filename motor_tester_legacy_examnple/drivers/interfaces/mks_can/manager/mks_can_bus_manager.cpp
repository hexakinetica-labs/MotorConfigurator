#include "mks_can/manager/mks_can_bus_manager.h"

#include "mks_can/adapter/mks_axis_adapter.h"
#include "mks_can/internal/port/can_port_interface.h"
#include "mks_can/internal/port/gs_usb_can_port.h"
#include "mks_can/internal/port/sim_can_port.h"
#include "mks_can/internal/protocol/mks_protocol.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>

namespace {

using namespace std::chrono_literals;

constexpr std::size_t kMaxTxCommandsPerCycle = 24U;
constexpr std::size_t kMaxTelemetryRequestsPerCycle = 8U;
constexpr std::size_t kMaxRxFramesPerCycle = 64U;

constexpr auto kResponsePolicyRetryPeriod = 500ms;
constexpr auto kStatusPollPeriod = 50ms;
constexpr auto kProtectionPollPeriod = 200ms;
constexpr auto kSpeedPollPeriod = 25ms;
constexpr auto kPositionPollPeriod = 10ms;
constexpr auto kSyncFlagReassertPeriod = 1500ms;

[[nodiscard]] std::uint64_t to_timestamp_ns(const std::chrono::steady_clock::time_point now) {
    return static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count());
}

[[nodiscard]] bool send_async_command(mks::MksProtocol& protocol,
                                      const std::uint16_t can_id,
                                      const mks::MksCommand command,
                                      const std::uint8_t* payload_data = nullptr,
                                      std::size_t payload_size = 0) {
    std::vector<std::uint8_t> ignored_response{};
    return protocol.sendCommand(can_id, command, payload_data, payload_size, ignored_response, 0xFF, 0U, false);
}

[[nodiscard]] bool send_async_command(mks::MksProtocol& protocol,
                                      const std::uint16_t can_id,
                                      const mks::MksCommand command,
                                      std::initializer_list<std::uint8_t> payload) {
    std::vector<std::uint8_t> ignored_response{};
    return protocol.sendCommand(can_id, command, payload.begin(), payload.size(), ignored_response, 0xFF, 0U, false);
}

[[nodiscard]] motion_core::AxisState decode_axis_state_from_status(const std::uint8_t motor_status,
                                                                    const std::uint8_t protection_code) {
    if (protection_code != 0U) {
        return motion_core::AxisState::Fault;
    }

    switch (motor_status) {
        case 0U:
            return motion_core::AxisState::Fault;
        case 1U:
            return motion_core::AxisState::Ready;
        case 2U:
        case 3U:
        case 4U:
        case 5U:
        case 6U:
            return motion_core::AxisState::OperationEnabled;
        default:
            return motion_core::AxisState::Unknown;
    }
}

} // namespace

namespace mks {

MksCanBusManager::MksCanBusManager(MksCanBusConfig config)
    : config_(std::move(config)) {}

MksCanBusManager::~MksCanBusManager() {
    (void)stop();
}

motion_core::Result<void> MksCanBusManager::start() {
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        if (started_) {
            return motion_core::Result<void>::success();
        }
    }

    last_stats_time_ = std::chrono::steady_clock::now();
    cycles_since_last_stats_ = 0U;
    tx_bits_last_ = 0U;
    rx_bits_last_ = 0U;
    current_stats_ = motion_core::BusStatistics{};

    if (config_.device_path.empty()) {
        return motion_core::Result<void>::failure(
            {motion_core::ErrorCode::InvalidArgument, "device_path is empty"});
    }

    std::unique_ptr<ICanPort> can_port{};
    if (config_.simulated || config_.device_path.rfind("sim", 0) == 0) {
        can_port = std::make_unique<SimCanPort>();
    } else {
        can_port = std::make_unique<GsUsbCanPort>();
    }

    if (!can_port->open(config_.device_path.c_str(), config_.baud_rate)) {
        return motion_core::Result<void>::failure(
            {motion_core::ErrorCode::TransportFailure, "failed to open CAN port"});
    }

    auto protocol = std::make_unique<MksProtocol>(*can_port);

    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        can_port_ = std::move(can_port);
        protocol_ = std::move(protocol);

        const auto now = std::chrono::steady_clock::now();
        telemetry_state_.clear();
        for (const auto& [can_id, adapter] : adapter_registry_) {
            if (!adapter) {
                continue;
            }

            auto schedule = TelemetryScheduleState{};
            schedule.telemetry.mode = adapter->current_mode();
            schedule.telemetry.state = motion_core::AxisState::Unknown;
            schedule.telemetry.timestamp_ns = to_timestamp_ns(now);
            schedule.next_response_policy_due = now;
            schedule.next_status_due = now;
            schedule.next_protection_due = now;
            schedule.next_speed_due = now;
            schedule.next_position_due = now;
            schedule.next_sync_flag_reassert_due = now;
            telemetry_state_[can_id] = schedule;
        }

        telemetry_rr_index_ = 0U;
        started_ = true;
    }

    const auto loop_result = runtime_loop_.start(config_.cycle_time, [this]() { poll_cycle(); });
    if (!loop_result.ok()) {
        std::lock_guard<std::mutex> lock(state_mutex_);
        started_ = false;
        telemetry_state_.clear();
        protocol_.reset();
        if (can_port_) {
            can_port_->close();
            can_port_.reset();
        }
        return loop_result;
    }

    return motion_core::Result<void>::success();
}

motion_core::Result<void> MksCanBusManager::stop() {
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        if (!started_) {
            return motion_core::Result<void>::success();
        }
        started_ = false;
    }

    (void)runtime_loop_.stop();

    std::lock_guard<std::mutex> io_lock(io_mutex_);
    std::lock_guard<std::mutex> lock(state_mutex_);
    protocol_.reset();
    if (can_port_) {
        can_port_->close();
        can_port_.reset();
    }

    rr_index_ = 0U;
    telemetry_rr_index_ = 0U;
    telemetry_state_.clear();

    return motion_core::Result<void>::success();
}

motion_core::Result<void> MksCanBusManager::register_adapter(const std::uint16_t can_id, MksAxisAdapter* adapter) {
    if (can_id == 0U || adapter == nullptr) {
        return motion_core::Result<void>::failure(
            {motion_core::ErrorCode::InvalidArgument, "invalid adapter or can_id"});
    }

    std::lock_guard<std::mutex> lock(state_mutex_);
    if (!is_started_locked()) {
        return motion_core::Result<void>::failure(
            {motion_core::ErrorCode::NotConnected, "not started"});
    }

    const auto existing_it = adapter_registry_.find(can_id);
    if (existing_it != adapter_registry_.end()) {
        if (existing_it->second == adapter) {
            return motion_core::Result<void>::success();
        }
        return motion_core::Result<void>::failure(
            {motion_core::ErrorCode::AlreadyExists, "exists"});
    }

    adapter_registry_.emplace(can_id, adapter);
    axis_order_.push_back(can_id);

    const auto now = std::chrono::steady_clock::now();
    auto schedule = TelemetryScheduleState{};
    schedule.telemetry.mode = adapter->current_mode();
    schedule.telemetry.state = motion_core::AxisState::Unknown;
    schedule.telemetry.timestamp_ns = to_timestamp_ns(now);
    schedule.next_response_policy_due = now;
    schedule.next_status_due = now;
    schedule.next_protection_due = now;
    schedule.next_speed_due = now;
    schedule.next_position_due = now;
    schedule.next_sync_flag_reassert_due = now;
    telemetry_state_[can_id] = schedule;

    return motion_core::Result<void>::success();
}

motion_core::Result<void> MksCanBusManager::unregister_adapter(const std::uint16_t can_id, MksAxisAdapter* adapter) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    const auto it = adapter_registry_.find(can_id);
    if (it == adapter_registry_.end() || it->second != adapter) {
        return motion_core::Result<void>::failure(
            {motion_core::ErrorCode::NotFound, "not registered"});
    }

    adapter_registry_.erase(it);
    axis_order_.erase(std::remove(axis_order_.begin(), axis_order_.end(), can_id), axis_order_.end());
    telemetry_state_.erase(can_id);

    if (rr_index_ >= axis_order_.size()) {
        rr_index_ = 0U;
    }
    if (telemetry_rr_index_ >= axis_order_.size()) {
        telemetry_rr_index_ = 0U;
    }

    return motion_core::Result<void>::success();
}

motion_core::Result<void> MksCanBusManager::remap_adapter_can_id(const std::uint16_t old_can_id,
                                                                 const std::uint16_t new_can_id,
                                                                 MksAxisAdapter* adapter) {
    if (adapter == nullptr) {
        return motion_core::Result<void>::failure(
            {motion_core::ErrorCode::InvalidArgument, "adapter is null"});
    }
    if (new_can_id == 0U || new_can_id > 0x07FFU) {
        return motion_core::Result<void>::failure(
            {motion_core::ErrorCode::InvalidArgument, "new CAN ID must be in range 1..0x7FF"});
    }

    std::lock_guard<std::mutex> lock(state_mutex_);
    const auto old_it = adapter_registry_.find(old_can_id);
    if (old_it == adapter_registry_.end() || old_it->second != adapter) {
        return motion_core::Result<void>::failure(
            {motion_core::ErrorCode::NotFound, "old CAN ID mapping not found for adapter"});
    }
    if (old_can_id == new_can_id) {
        return motion_core::Result<void>::success();
    }
    if (adapter_registry_.find(new_can_id) != adapter_registry_.end()) {
        return motion_core::Result<void>::failure(
            {motion_core::ErrorCode::AlreadyExists, "new CAN ID is already in use"});
    }

    adapter_registry_.erase(old_it);
    adapter_registry_.emplace(new_can_id, adapter);

    for (auto& axis_can_id : axis_order_) {
        if (axis_can_id == old_can_id) {
            axis_can_id = new_can_id;
            break;
        }
    }

    const auto old_schedule_it = telemetry_state_.find(old_can_id);
    if (old_schedule_it != telemetry_state_.end()) {
        auto moved_schedule = old_schedule_it->second;
        moved_schedule.response_policy_retries_remaining = 3U;
        moved_schedule.next_response_policy_due = std::chrono::steady_clock::now();
        moved_schedule.next_sync_flag_reassert_due = std::chrono::steady_clock::now();
        telemetry_state_.erase(old_schedule_it);
        telemetry_state_[new_can_id] = moved_schedule;
    } else {
        const auto now = std::chrono::steady_clock::now();
        auto schedule = TelemetryScheduleState{};
        schedule.telemetry.mode = adapter->current_mode();
        schedule.telemetry.state = motion_core::AxisState::Unknown;
        schedule.telemetry.timestamp_ns = to_timestamp_ns(now);
        schedule.next_response_policy_due = now;
        schedule.next_status_due = now;
        schedule.next_protection_due = now;
        schedule.next_speed_due = now;
        schedule.next_position_due = now;
        schedule.next_sync_flag_reassert_due = now;
        telemetry_state_[new_can_id] = schedule;
    }

    return motion_core::Result<void>::success();
}

bool MksCanBusManager::is_can_id_registered(const std::uint16_t can_id) const {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return adapter_registry_.find(can_id) != adapter_registry_.end();
}

motion_core::Result<void> MksCanBusManager::send_raw_command(const std::uint16_t can_id,
                                                             const std::uint8_t cmd_byte,
                                                             const std::vector<std::uint8_t>& data) {
    std::lock_guard<std::mutex> io_lock(io_mutex_);
    {
        std::lock_guard<std::mutex> state_lock(state_mutex_);
        if (!is_started_locked()) {
            return motion_core::Result<void>::failure(
                {motion_core::ErrorCode::NotConnected, "not started"});
        }
    }

    std::vector<std::uint8_t> response{};
    if (!protocol_->sendCommand(can_id, static_cast<MksCommand>(cmd_byte), data, response, 0xFF, 0U, false)) {
        return motion_core::Result<void>::failure(
            {motion_core::ErrorCode::TransportFailure, "async write failed"});
    }

    return motion_core::Result<void>::success();
}

motion_core::Result<std::vector<std::uint8_t>> MksCanBusManager::execute_raw_command_sync(
    const std::uint16_t can_id,
    const std::uint8_t cmd_byte,
    const std::vector<std::uint8_t>& data) {
    std::lock_guard<std::mutex> dispatch_lock(sync_dispatch_mutex_);
    const auto validation = validate_sync_request(can_id);
    if (!validation.ok()) {
        return motion_core::Result<std::vector<std::uint8_t>>::failure(validation.error());
    }

    {
        std::lock_guard<std::mutex> lock(sync_request_.mutex);
        sync_request_.active = true;
        sync_request_.completed = false;
        sync_request_.can_id = can_id;
        sync_request_.expected_cmd = cmd_byte;
        sync_request_.payload.clear();
    }

    {
        std::lock_guard<std::mutex> io_lock(io_mutex_);
        std::vector<std::uint8_t> ignored_response{};
        if (!protocol_->sendCommand(can_id,
                                    static_cast<MksCommand>(cmd_byte),
                                    data.data(), data.size(),
                                    ignored_response,
                                    cmd_byte,
                                    0U,
                                    false)) {
            std::lock_guard<std::mutex> lock(sync_request_.mutex);
            sync_request_.active = false;
            return motion_core::Result<std::vector<std::uint8_t>>::failure(
                {motion_core::ErrorCode::TransportFailure, "sync write dispatch failed"});
        }
    }

    const auto response = await_sync_response(can_id, cmd_byte, std::chrono::milliseconds(100));
    if (response.ok()) {
        return response;
    }

    std::lock_guard<std::mutex> io_lock(io_mutex_);
    const std::string trace = protocol_->lastExchangeTraceSummary();
    std::fprintf(stderr,
                 "[MKS][SYNC-WRITE-FAIL] cmd=0x%02X can_id=0x%03X error=%s trace=%s\n",
                 static_cast<unsigned int>(cmd_byte),
                 static_cast<unsigned int>(can_id),
                 response.error().message,
                 trace.c_str());
    return response;
}

motion_core::Result<std::vector<std::uint8_t>> MksCanBusManager::read_system_parameter(
    std::uint16_t can_id,
    std::uint8_t parameter_cmd) {
    std::lock_guard<std::mutex> dispatch_lock(sync_dispatch_mutex_);
    const auto validation = validate_sync_request(can_id);
    if (!validation.ok()) {
        return motion_core::Result<std::vector<std::uint8_t>>::failure(validation.error());
    }

    {
        std::lock_guard<std::mutex> lock(sync_request_.mutex);
        sync_request_.active = true;
        sync_request_.completed = false;
        sync_request_.can_id = can_id;
        sync_request_.expected_cmd = parameter_cmd;
        sync_request_.payload.clear();
    }

    {
        std::lock_guard<std::mutex> io_lock(io_mutex_);
        std::vector<std::uint8_t> ignored_response{};
        std::uint8_t req_data = parameter_cmd;
        if (!protocol_->sendCommand(can_id,
                                    static_cast<MksCommand>(0x00),
                                    &req_data, 1,
                                    ignored_response,
                                    parameter_cmd,
                                    0U,
                                    false)) {
            std::lock_guard<std::mutex> lock(sync_request_.mutex);
            sync_request_.active = false;
            return motion_core::Result<std::vector<std::uint8_t>>::failure(
                {motion_core::ErrorCode::TransportFailure, "sync read dispatch failed"});
        }
    }

    const auto response = await_sync_response(can_id, parameter_cmd, std::chrono::milliseconds(100));
    if (response.ok()) {
        return response;
    }

    std::lock_guard<std::mutex> io_lock(io_mutex_);
    const std::string trace = protocol_->lastExchangeTraceSummary();
    std::fprintf(stderr,
                 "[MKS][SYNC-READ-FAIL] param_cmd=0x%02X can_id=0x%03X error=%s trace=%s\n",
                 static_cast<unsigned int>(parameter_cmd),
                 static_cast<unsigned int>(can_id),
                 response.error().message,
                 trace.c_str());
    return response;
}

bool MksCanBusManager::is_started_locked() const noexcept {
    return started_ && can_port_ && protocol_;
}

motion_core::Result<void> MksCanBusManager::validate_sync_request(const std::uint16_t can_id) const {
    std::lock_guard<std::mutex> state_lock(state_mutex_);
    if (!is_started_locked()) {
        return motion_core::Result<void>::failure(
            {motion_core::ErrorCode::NotConnected, "not started"});
    }
    if (adapter_registry_.find(can_id) == adapter_registry_.end()) {
        return motion_core::Result<void>::failure(
            {motion_core::ErrorCode::NotFound, "not registered"});
    }
    return motion_core::Result<void>::success();
}

motion_core::Result<std::vector<std::uint8_t>> MksCanBusManager::await_sync_response(
    const std::uint16_t /*can_id*/,
    const std::uint8_t /*expected_response_cmd*/,
    const std::chrono::milliseconds timeout) {
    
    std::unique_lock<std::mutex> lock(sync_request_.mutex);
    if (!sync_request_.cv.wait_for(lock, timeout, [this] { return sync_request_.completed; })) {
        sync_request_.active = false;
        return motion_core::Result<std::vector<std::uint8_t>>::failure(
            {motion_core::ErrorCode::Timeout, "timeout waiting sync response"});
    }
    
    sync_request_.active = false;
    return motion_core::Result<std::vector<std::uint8_t>>::success(std::move(sync_request_.payload));
}

motion_core::Result<motion_core::BusStatistics> MksCanBusManager::get_statistics() const {
    std::lock_guard<std::mutex> lock(stats_mutex_);
    return motion_core::Result<motion_core::BusStatistics>::success(current_stats_);
}

void MksCanBusManager::update_stats(const bool can_sample_io_stats) {
    const auto now = std::chrono::steady_clock::now();
    const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_stats_time_).count();
    if (elapsed >= 1000) {
        std::lock_guard<std::mutex> stat_lock(stats_mutex_);
        current_stats_.cycle_rate_hz =
            static_cast<double>(cycles_since_last_stats_) * 1000.0 / static_cast<double>(elapsed);

        if (can_sample_io_stats && protocol_) {
            const auto& io = protocol_->ioStats();
            const std::uint64_t tx_diff = io.tx_bits - tx_bits_last_;
            const std::uint64_t rx_diff = io.rx_bits - rx_bits_last_;
            const double total_bits_per_sec =
                static_cast<double>(tx_diff + rx_diff) * 1000.0 / static_cast<double>(elapsed);
            if (config_.baud_rate > 0U) {
                current_stats_.bus_load_percent =
                    (total_bits_per_sec / static_cast<double>(config_.baud_rate)) * 100.0;
            } else {
                current_stats_.bus_load_percent = 0.0;
            }

            tx_bits_last_ = io.tx_bits;
            rx_bits_last_ = io.rx_bits;
        }

        cycles_since_last_stats_ = 0U;
        last_stats_time_ = now;
    }

    ++cycles_since_last_stats_;
}

std::size_t MksCanBusManager::dispatch_worker_commands(
    const std::vector<std::pair<std::uint16_t, MksAxisAdapter*>>& adapters,
    MksProtocol& protocol,
    bool& has_pending_sync_motion,
    const std::chrono::steady_clock::time_point now) {
    has_pending_sync_motion = false;
    std::size_t tx_sent = 0U;
    for (const auto& [can_id, adapter] : adapters) {
        static_cast<void>(can_id);
        if (!adapter) {
            continue;
        }

        bool sent_sync_for_axis = false;
        MksBusCommand command{};
        while (tx_sent < kMaxTxCommandsPerCycle && adapter->consume_tx_command(command)) {
            if (command.requires_sync_execute && !sent_sync_for_axis) {
                sent_sync_for_axis = true;

                bool enable_multi_sync = false;
                {
                    std::lock_guard<std::mutex> lock(state_mutex_);
                    const auto schedule_it = telemetry_state_.find(command.can_id);
                    if (schedule_it != telemetry_state_.end()
                        && schedule_it->second.next_sync_flag_reassert_due <= now) {
                        enable_multi_sync = true;
                        schedule_it->second.next_sync_flag_reassert_due = now + kSyncFlagReassertPeriod;
                    }
                }

                if (enable_multi_sync) {
                    (void)send_async_command(protocol,
                                             command.can_id,
                                             MksCommand::SetMultiMotorSyncFlag,
                                             {0x01U});
                    ++tx_sent;
                }
            }

            (void)send_async_command(protocol,
                                     command.can_id,
                                     static_cast<MksCommand>(command.command),
                                     command.payload.data(),
                                     command.payload_size);
            adapter->mark_command_tx();
            has_pending_sync_motion = has_pending_sync_motion || command.requires_sync_execute;
            ++tx_sent;
        }
        
        if (tx_sent >= kMaxTxCommandsPerCycle) {
            break;
        }
    }

    return tx_sent;
}

void MksCanBusManager::schedule_async_telemetry_requests(
    const std::vector<std::pair<std::uint16_t, MksAxisAdapter*>>& adapters,
    MksProtocol& protocol,
    const std::size_t tx_commands_sent,
    const std::chrono::steady_clock::time_point now) {
    if (adapters.empty()) {
        return;
    }

    const std::size_t free_slots =
        (tx_commands_sent >= kMaxTxCommandsPerCycle) ? 0U : (kMaxTxCommandsPerCycle - tx_commands_sent);
    const std::size_t telemetry_budget = std::min(free_slots, kMaxTelemetryRequestsPerCycle);
    if (telemetry_budget == 0U) {
        return;
    }

    if (telemetry_rr_index_ >= adapters.size()) {
        telemetry_rr_index_ = 0U;
    }

    std::size_t sent = 0U;
    std::size_t attempts = 0U;
    const std::size_t max_attempts = adapters.size() * 4U;

    while (sent < telemetry_budget && attempts < max_attempts) {
        const std::size_t rr_index = telemetry_rr_index_ % adapters.size();
        telemetry_rr_index_ = (telemetry_rr_index_ + 1U) % adapters.size();
        ++attempts;

        const auto can_id = adapters[rr_index].first;

        std::lock_guard<std::mutex> lock(state_mutex_);
        const auto schedule_it = telemetry_state_.find(can_id);
        if (schedule_it == telemetry_state_.end()) {
            continue;
        }

        auto& schedule = schedule_it->second;

        if (schedule.response_policy_retries_remaining > 0U
            && schedule.next_response_policy_due <= now) {
            (void)send_async_command(protocol,
                                     can_id,
                                     MksCommand::SetSlaveRespondActive,
                                     {1U, 0U});
            --schedule.response_policy_retries_remaining;
            schedule.next_response_policy_due = now + kResponsePolicyRetryPeriod;
            ++sent;
            continue;
        }

        if (schedule.next_position_due <= now) {
            (void)send_async_command(protocol, can_id, MksCommand::ReadEncoderAddition);
            schedule.next_position_due = now + kPositionPollPeriod;
            ++sent;
            continue;
        }

        if (schedule.next_speed_due <= now) {
            (void)send_async_command(protocol, can_id, MksCommand::ReadMotorSpeed);
            schedule.next_speed_due = now + kSpeedPollPeriod;
            ++sent;
            continue;
        }

        if (schedule.next_status_due <= now) {
            (void)send_async_command(protocol, can_id, MksCommand::QueryMotorStatus);
            schedule.next_status_due = now + kStatusPollPeriod;
            ++sent;
            continue;
        }

        if (schedule.next_protection_due <= now) {
            (void)send_async_command(protocol, can_id, MksCommand::ReadProtectionState);
            schedule.next_protection_due = now + kProtectionPollPeriod;
            ++sent;
            continue;
        }
    }
}

void MksCanBusManager::drain_async_rx_frames(
    const std::unordered_map<std::uint16_t, MksAxisAdapter*>& adapters_by_can_id,
    const std::chrono::steady_clock::time_point now) {
    if (!can_port_) {
        return;
    }

    for (std::size_t frame_index = 0U; frame_index < kMaxRxFramesPerCycle; ++frame_index) {
        CanFrame frame{};
        if (!can_port_->read(frame, 0U)) {
            break;
        }
        process_async_rx_frame(frame, adapters_by_can_id, now);
    }
}

void MksCanBusManager::process_async_rx_frame(
    const CanFrame& frame,
    const std::unordered_map<std::uint16_t, MksAxisAdapter*>& adapters_by_can_id,
    const std::chrono::steady_clock::time_point now) {
    if (frame.id > 0x07FFU || frame.dlc < 3U || frame.dlc > 8U) {
        return;
    }

    const auto can_id = static_cast<std::uint16_t>(frame.id);
    const auto adapter_it = adapters_by_can_id.find(can_id);
    if (adapter_it == adapters_by_can_id.end() || adapter_it->second == nullptr) {
        return;
    }
    auto* adapter = adapter_it->second;

    std::vector<std::uint8_t> bytes_without_crc{};
    bytes_without_crc.reserve(frame.dlc - 1U);
    for (std::uint8_t index = 0U; index + 1U < frame.dlc; ++index) {
        bytes_without_crc.push_back(frame.data[index]);
    }

    const auto expected_crc = MksProtocol::computeCrc(can_id, bytes_without_crc);
    const auto actual_crc = frame.data[frame.dlc - 1U];
    if (expected_crc != actual_crc) {
        return;
    }

    const auto cmd = frame.data[0];
    const auto* payload = frame.data + 1U;
    const auto payload_size = static_cast<std::size_t>(frame.dlc - 2U);

    {
        std::lock_guard<std::mutex> sync_lock(sync_request_.mutex);
        if (sync_request_.active && sync_request_.can_id == can_id && sync_request_.expected_cmd == cmd) {
            sync_request_.payload.assign(payload, payload + payload_size);
            sync_request_.completed = true;
            sync_request_.cv.notify_all();
            return; // Consumed as a synchronous response
        }
    }

    bool mark_status_rx = false;
    bool mark_protection_rx = false;
    bool mark_speed_rx = false;
    bool mark_position_rx = false;

    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        const auto telemetry_it = telemetry_state_.find(can_id);
        if (telemetry_it == telemetry_state_.end()) {
            return;
        }

        auto& schedule = telemetry_it->second;
        auto& telemetry = schedule.telemetry;
        bool telemetry_updated = false;

        switch (cmd) {
            case static_cast<std::uint8_t>(MksCommand::QueryMotorStatus): {
                if (payload_size >= 1U) {
                    telemetry.motion_status_code = payload[payload_size - 1U];
                    schedule.has_status = true;
                    telemetry_updated = true;
                    mark_status_rx = true;
                }
                break;
            }
            case static_cast<std::uint8_t>(MksCommand::ReadProtectionState): {
                if (payload_size >= 1U) {
                    telemetry.protection_code = payload[payload_size - 1U];
                    telemetry.status_word = payload[payload_size - 1U];
                    schedule.has_protection = true;
                    telemetry_updated = true;
                    mark_protection_rx = true;
                }
                break;
            }
            case static_cast<std::uint8_t>(MksCommand::ReadMotorSpeed): {
                if (payload_size >= 2U) {
                    const auto motor_rpm = static_cast<double>(MksProtocol::readBe16s(payload));
                    const auto gear_ratio = std::max(0.001, adapter->software_gear_ratio());
                    double velocity_deg_per_sec = (motor_rpm * 6.0) / gear_ratio;
                    if (adapter->invert_direction()) {
                        velocity_deg_per_sec = -velocity_deg_per_sec;
                    }
                    telemetry.actual_velocity_deg_per_sec = velocity_deg_per_sec;
                    telemetry_updated = true;
                    mark_speed_rx = true;
                }
                break;
            }
            case static_cast<std::uint8_t>(MksCommand::ReadEncoderAddition):
            case static_cast<std::uint8_t>(MksCommand::ReadRawEncoderAddition): {
                if (payload_size >= 4U) {
                    std::int64_t raw_position_i64 = 0;
                    if (payload_size >= 6U) {
                        raw_position_i64 = MksProtocol::readBe48s(payload);
                    } else {
                        raw_position_i64 = static_cast<std::int64_t>(MksProtocol::readBe32s(payload));
                    }

                    telemetry.raw_axis_position = raw_position_i64;

                    const auto raw_position = static_cast<double>(raw_position_i64);
                    const auto axis_units_per_degree = adapter->axis_units_per_degree();
                    if (std::abs(axis_units_per_degree) > 1e-9) {
                        double position_deg = raw_position / axis_units_per_degree;
                        if (adapter->invert_direction()) {
                            position_deg = -position_deg;
                        }

                        telemetry.actual_position_deg = position_deg;
                        if (adapter->has_requested_target_position()) {
                            telemetry.target_position_deg = adapter->requested_target_position_deg();
                        } else {
                            telemetry.target_position_deg = position_deg;
                        }
                    }

                    schedule.has_position = true;
                    telemetry_updated = true;
                    mark_position_rx = true;
                }
                break;
            }
            case static_cast<std::uint8_t>(MksCommand::SetSlaveRespondActive): {
                if (payload_size >= 2U && payload[0] == 1U && payload[1] == 0U) {
                    schedule.response_policy_retries_remaining = 0U;
                }
                break;
            }
            default:
                break;
        }

        if (!telemetry_updated) {
            return;
        }

        if (schedule.has_status || schedule.has_protection) {
            telemetry.state = decode_axis_state_from_status(
                static_cast<std::uint8_t>(telemetry.motion_status_code & 0xFFU),
                static_cast<std::uint8_t>(telemetry.protection_code & 0xFFU));
        } else {
            telemetry.state = motion_core::AxisState::Unknown;
        }

        telemetry.timestamp_ns = to_timestamp_ns(now);
        schedule.dirty = true;
    }

    if (mark_status_rx) {
        adapter->mark_status_rx();
    }
    if (mark_protection_rx) {
        adapter->mark_protection_rx();
    }
    if (mark_speed_rx) {
        adapter->mark_speed_rx();
    }
    if (mark_position_rx) {
        adapter->mark_position_rx();
    }
}

void MksCanBusManager::publish_dirty_telemetry(
    const std::vector<std::pair<std::uint16_t, MksAxisAdapter*>>& adapters,
    const std::chrono::steady_clock::time_point now) {
    for (const auto& [can_id, adapter] : adapters) {
        if (!adapter) {
            continue;
        }

        bool should_publish = false;
        motion_core::AxisTelemetry telemetry{};
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            const auto telemetry_it = telemetry_state_.find(can_id);
            if (telemetry_it == telemetry_state_.end() || !telemetry_it->second.dirty) {
                continue;
            }

            telemetry = telemetry_it->second.telemetry;
            telemetry.mode = adapter->current_mode();
            telemetry.timestamp_ns = to_timestamp_ns(now);
            telemetry_it->second.telemetry = telemetry;
            telemetry_it->second.dirty = false;
            should_publish = true;
        }

        if (should_publish) {
            adapter->mark_telemetry_publish();
            adapter->publish_polled_telemetry(telemetry);
        }
    }
}

void MksCanBusManager::poll_cycle() {
    std::lock_guard<std::mutex> io_lock(io_mutex_);
    const bool can_sample_io_stats = static_cast<bool>(protocol_);
    update_stats(can_sample_io_stats);
    if (!can_sample_io_stats) {
        return;
    }

    std::vector<std::pair<std::uint16_t, MksAxisAdapter*>> adapters{};
    std::unordered_map<std::uint16_t, MksAxisAdapter*> adapters_by_can_id{};
    MksProtocol* protocol = nullptr;
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        if (!is_started_locked() || axis_order_.empty()) {
            return;
        }

        protocol = protocol_.get();
        adapters.reserve(axis_order_.size());
        for (const auto can_id : axis_order_) {
            const auto it = adapter_registry_.find(can_id);
            if (it != adapter_registry_.end() && it->second != nullptr) {
                adapters.push_back({can_id, it->second});
                adapters_by_can_id.emplace(can_id, it->second);
            }
        }
    }

    if (protocol == nullptr || adapters.empty()) {
        return;
    }

    const auto now = std::chrono::steady_clock::now();

    for (const auto& [can_id, adapter] : adapters) {
        static_cast<void>(can_id);
        if (adapter) {
            adapter->step_worker(now);
        }
    }

    bool has_pending_sync_motion = false;
    const auto tx_commands_sent = dispatch_worker_commands(adapters, *protocol, has_pending_sync_motion, now);
    if (has_pending_sync_motion) {
        (void)send_async_command(*protocol,
                                 0x000,
                                 MksCommand::ExecuteMultiMotorSync,
                                 {});
    }
    schedule_async_telemetry_requests(adapters, *protocol, tx_commands_sent, now);
    drain_async_rx_frames(adapters_by_can_id, now);
    publish_dirty_telemetry(adapters, now);
}

std::shared_ptr<MksCanBusManager> make_mks_can_bus_manager(MksCanBusConfig config) {
    return std::make_shared<MksCanBusManager>(std::move(config));
}

} // namespace mks
