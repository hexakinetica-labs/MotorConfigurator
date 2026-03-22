#pragma once

#include "motion_core/result.h"
#include "motion_core/runtime_loop.h"
#include "motion_core/bus_manager_interface.h"

#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <unordered_map>
#include <vector>
#include <string>

namespace mks {

class ICanPort;
class MksProtocol;
class MksAxisAdapter;

struct MksCanBusConfig {
    std::string device_path{};
    unsigned int baud_rate{1'000'000U};
    std::chrono::milliseconds cycle_time{4};
    bool simulated{false};
};

class MksCanBusManager : public motion_core::IBusManager {
public:
    explicit MksCanBusManager(MksCanBusConfig config);
    ~MksCanBusManager();

    MksCanBusManager(const MksCanBusManager&) = delete;
    MksCanBusManager& operator=(const MksCanBusManager&) = delete;

    [[nodiscard]] std::string get_name() const override { return "MKS CAN " + config_.device_path; }

    motion_core::Result<void> start() override;
    motion_core::Result<void> stop() override;

    motion_core::Result<void> register_adapter(std::uint16_t can_id, MksAxisAdapter* adapter);
    motion_core::Result<void> unregister_adapter(std::uint16_t can_id, MksAxisAdapter* adapter);
    motion_core::Result<void> remap_adapter_can_id(std::uint16_t old_can_id,
                                                   std::uint16_t new_can_id,
                                                   MksAxisAdapter* adapter);
    [[nodiscard]] bool is_can_id_registered(std::uint16_t can_id) const;

    // Completely generalized dispatch (Asynchronous) - Immediate flush in same thread
    motion_core::Result<void> send_raw_command(std::uint16_t can_id,
                                               std::uint8_t cmd_byte,
                                               const std::vector<std::uint8_t>& data);

    // Synchronous execute, used for safe parameter writing where we must wait for a response
    motion_core::Result<std::vector<std::uint8_t>> execute_raw_command_sync(
        std::uint16_t can_id, std::uint8_t cmd_byte, const std::vector<std::uint8_t>& data);

    // Synchronous read for parameters without specific motor structs
    [[nodiscard]] motion_core::Result<std::vector<std::uint8_t>> read_system_parameter(std::uint16_t can_id, std::uint8_t parameter_cmd);

    [[nodiscard]] motion_core::Result<motion_core::BusStatistics> get_statistics() const override;

private:
    [[nodiscard]] bool is_started_locked() const noexcept;
    [[nodiscard]] motion_core::Result<void> validate_sync_request(std::uint16_t can_id) const;

    void update_stats(bool can_sample_io_stats);
    void poll_cycle();

    MksCanBusConfig config_{};

    mutable std::mutex state_mutex_;
    mutable std::mutex io_mutex_;

    bool started_{false};
    std::unordered_map<std::uint16_t, MksAxisAdapter*> adapter_registry_;
    std::vector<std::uint16_t> axis_order_;
    std::size_t rr_index_{0};

    std::unique_ptr<ICanPort> can_port_;
    std::unique_ptr<MksProtocol> protocol_;
    motion_core::RuntimeLoop runtime_loop_;

    // Statistics
    std::chrono::steady_clock::time_point last_stats_time_;
    std::uint64_t cycles_since_last_stats_{0};
    std::uint64_t tx_bits_last_{0};
    std::uint64_t rx_bits_last_{0};
    
    mutable std::mutex stats_mutex_;
    motion_core::BusStatistics current_stats_{};
};

[[nodiscard]] std::shared_ptr<MksCanBusManager> make_mks_can_bus_manager(MksCanBusConfig config);

} // namespace mks
