#include "mks/manager/mks_can_bus_manager.h"

#include "mks/adapter/mks_axis_adapter.h"
#include "mks/internal/port/can_port_interface.h"
#include "mks/internal/port/gs_usb_can_port.h"
#include "mks/internal/protocol/mks_protocol.h"
#include "mks/internal/port/sim_can_port.h"

#include <algorithm>
#include <chrono>

namespace {

bool validate_response_crc(const std::uint16_t can_id, const mks::CanFrame& frame) {
    std::vector<std::uint8_t> bytes_without_crc;
    bytes_without_crc.reserve(frame.dlc - 1U);
    for (std::uint8_t i = 0; i + 1U < frame.dlc; ++i) {
        bytes_without_crc.push_back(frame.data[i]);
    }

    const auto expected_crc = mks::MksProtocol::computeCrc(can_id, bytes_without_crc);
    const auto actual_crc = frame.data[frame.dlc - 1U];
    return expected_crc == actual_crc;
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
        if (started_) return motion_core::Result<void>::success();
    }

    last_stats_time_ = std::chrono::steady_clock::now();
    cycles_since_last_stats_ = 0;
    tx_bits_last_ = 0;
    rx_bits_last_ = 0;
    current_stats_ = motion_core::BusStatistics{};

    if (config_.device_path.empty()) {
        return motion_core::Result<void>::failure({motion_core::ErrorCode::InvalidArgument, "device_path is empty"});
    }

    std::unique_ptr<ICanPort> can_port;
    if (config_.simulated || config_.device_path.rfind("sim", 0) == 0) {
        can_port = std::make_unique<SimCanPort>();
    } else {
        can_port = std::make_unique<GsUsbCanPort>();
    }
    
    if (!can_port->open(config_.device_path.c_str(), config_.baud_rate)) {
        return motion_core::Result<void>::failure({motion_core::ErrorCode::TransportFailure, "failed to open CAN port"});
    }

    auto protocol = std::make_unique<MksProtocol>(*can_port);

    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        can_port_ = std::move(can_port);
        protocol_ = std::move(protocol);
        started_ = true;
    }

    const auto loop_result = runtime_loop_.start(config_.cycle_time, [this]() { poll_cycle(); });
    if (!loop_result.ok()) {
        std::lock_guard<std::mutex> lock(state_mutex_);
        started_ = false;
        protocol_.reset();
        if (can_port_) { can_port_->close(); can_port_.reset(); }
        return loop_result;
    }

    return motion_core::Result<void>::success();
}

motion_core::Result<void> MksCanBusManager::stop() {
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        if (!started_) return motion_core::Result<void>::success();
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
    rr_index_ = 0;

    return motion_core::Result<void>::success();
}

motion_core::Result<void> MksCanBusManager::register_adapter(const std::uint16_t can_id, MksAxisAdapter* adapter) {
    if (can_id == 0 || !adapter) return motion_core::Result<void>::failure({motion_core::ErrorCode::InvalidArgument, "invalid adapter or can_id"});

    std::lock_guard<std::mutex> lock(state_mutex_);
    if (!is_started_locked()) return motion_core::Result<void>::failure({motion_core::ErrorCode::NotConnected, "not started"});
    const auto existing_it = adapter_registry_.find(can_id);
    if (existing_it != adapter_registry_.end()) {
        if (existing_it->second == adapter) {
            return motion_core::Result<void>::success();
        }
        return motion_core::Result<void>::failure({motion_core::ErrorCode::AlreadyExists, "exists"});
    }

    adapter_registry_.emplace(can_id, adapter);
    axis_order_.push_back(can_id);
    return motion_core::Result<void>::success();
}

motion_core::Result<void> MksCanBusManager::unregister_adapter(const std::uint16_t can_id, MksAxisAdapter* adapter) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    const auto it = adapter_registry_.find(can_id);
    if (it == adapter_registry_.end() || it->second != adapter) return motion_core::Result<void>::failure({motion_core::ErrorCode::NotFound, "not registered"});

    adapter_registry_.erase(it);
    axis_order_.erase(std::remove(axis_order_.begin(), axis_order_.end(), can_id), axis_order_.end());
    if (rr_index_ >= axis_order_.size()) rr_index_ = 0;
    return motion_core::Result<void>::success();
}

motion_core::Result<void> MksCanBusManager::remap_adapter_can_id(const std::uint16_t old_can_id,
                                                                 const std::uint16_t new_can_id,
                                                                 MksAxisAdapter* adapter) {
    if (!adapter) {
        return motion_core::Result<void>::failure(
            {motion_core::ErrorCode::InvalidArgument, "adapter is null"});
    }
    if (new_can_id == 0U || new_can_id > 0x7FFU) {
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
            return motion_core::Result<void>::failure({motion_core::ErrorCode::NotConnected, "not started"});
        }
    }

    std::vector<std::uint8_t> response;
    if (!protocol_->sendCommand(can_id, static_cast<MksCommand>(cmd_byte), data, response, 0xFF, 0, false)) {
        return motion_core::Result<void>::failure(
            {motion_core::ErrorCode::TransportFailure, "async write failed"});
    }

    return motion_core::Result<void>::success();
}

motion_core::Result<std::vector<std::uint8_t>> MksCanBusManager::execute_raw_command_sync(const std::uint16_t can_id,
                                                                                            const std::uint8_t cmd_byte,
                                                                                            const std::vector<std::uint8_t>& data) {
    const auto validation = validate_sync_request(can_id);
    if (!validation.ok()) {
        return motion_core::Result<std::vector<std::uint8_t>>::failure(validation.error());
    }
    
    std::lock_guard<std::mutex> io_lock(io_mutex_);
    std::vector<std::uint8_t> response;
    if (protocol_->sendCommand(can_id, static_cast<MksCommand>(cmd_byte), data, response, cmd_byte, 100, true)) {
        return motion_core::Result<std::vector<std::uint8_t>>::success(std::move(response));
    }
    return motion_core::Result<std::vector<std::uint8_t>>::failure({motion_core::ErrorCode::TransportFailure, "sync write failed"});
}

motion_core::Result<std::vector<std::uint8_t>> MksCanBusManager::read_system_parameter(std::uint16_t can_id, std::uint8_t parameter_cmd) {
    const auto validation = validate_sync_request(can_id);
    if (!validation.ok()) {
        return motion_core::Result<std::vector<std::uint8_t>>::failure(validation.error());
    }

    std::lock_guard<std::mutex> io_lock(io_mutex_);
    std::vector<std::uint8_t> response;
    if (protocol_->sendCommand(can_id, static_cast<MksCommand>(0x00), {parameter_cmd}, response, parameter_cmd, 100, true)) {
        return motion_core::Result<std::vector<std::uint8_t>>::success(std::move(response));
    }
    return motion_core::Result<std::vector<std::uint8_t>>::failure({motion_core::ErrorCode::TransportFailure, "sync read failed"});
}

bool MksCanBusManager::is_started_locked() const noexcept {
    return started_ && can_port_ && protocol_;
}

motion_core::Result<void> MksCanBusManager::validate_sync_request(const std::uint16_t can_id) const {
    std::lock_guard<std::mutex> state_lock(state_mutex_);
    if (!is_started_locked()) {
        return motion_core::Result<void>::failure({motion_core::ErrorCode::NotConnected, "not started"});
    }
    if (adapter_registry_.find(can_id) == adapter_registry_.end()) {
        return motion_core::Result<void>::failure({motion_core::ErrorCode::NotFound, "not registered"});
    }
    return motion_core::Result<void>::success();
}

motion_core::Result<motion_core::BusStatistics> MksCanBusManager::get_statistics() const {
    std::lock_guard<std::mutex> lock(stats_mutex_);
    return motion_core::Result<motion_core::BusStatistics>::success(current_stats_);
}

void MksCanBusManager::update_stats(const bool can_sample_io_stats) {
    auto now = std::chrono::steady_clock::now();
    if (auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_stats_time_).count(); elapsed >= 1000) {
        std::lock_guard<std::mutex> stat_lock(stats_mutex_);
        
        current_stats_.cycle_rate_hz = static_cast<double>(cycles_since_last_stats_) * 1000.0 / static_cast<double>(elapsed);
        
        if (can_sample_io_stats && protocol_) {
            const auto& io = protocol_->ioStats();
            uint64_t tx_diff = io.tx_bits - tx_bits_last_;
            uint64_t rx_diff = io.rx_bits - rx_bits_last_;
            
            double total_bits_per_sec = static_cast<double>(tx_diff + rx_diff) * 1000.0 / static_cast<double>(elapsed);
            if (config_.baud_rate > 0) {
                current_stats_.bus_load_percent = (total_bits_per_sec / static_cast<double>(config_.baud_rate)) * 100.0;
            } else {
                current_stats_.bus_load_percent = 0.0;
            }

            tx_bits_last_ = io.tx_bits;
            rx_bits_last_ = io.rx_bits;
        }

        cycles_since_last_stats_ = 0;
        last_stats_time_ = now;
    }
    ++cycles_since_last_stats_;
}

void MksCanBusManager::poll_cycle() {
    std::lock_guard<std::mutex> io_lock(io_mutex_);
    const bool can_sample_io_stats = static_cast<bool>(protocol_);
    update_stats(can_sample_io_stats);
    if (!can_sample_io_stats) return;

    // Round-robin one adapter per cycle to keep deterministic CAN bus load.
    MksAxisAdapter* adapter_to_process = nullptr;
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        if (!is_started_locked() || axis_order_.empty()) return;

        if (rr_index_ >= axis_order_.size()) {
            rr_index_ = 0;
        }
        const auto selected_can_id = axis_order_[rr_index_];
        rr_index_ = (rr_index_ + 1U) % axis_order_.size();

        const auto it = adapter_registry_.find(selected_can_id);
        if (it != adapter_registry_.end()) {
            adapter_to_process = it->second;
        }
    }

    if (adapter_to_process) {
        adapter_to_process->process_cycle(*protocol_);
    }
}

std::shared_ptr<MksCanBusManager> make_mks_can_bus_manager(MksCanBusConfig config) {
    return std::make_shared<MksCanBusManager>(std::move(config));
}

} // namespace mks
