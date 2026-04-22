#pragma once

#include "mks_can/axis/mks_axis_state_cache.h"
#include "mks_can/manager/mks_can_bus_manager.h"
#include "motion_core/axis_interface.h"

#include <atomic>
#include <cstdint>
#include <functional>
#include <memory>

namespace mks {

class MksAxisConfigurator final {
public:
    struct Config {
        motion_core::AxisId axis_id{};
        std::uint16_t can_id{1};
        std::shared_ptr<MksCanBusManager> bus_manager{};
        MksAxisStateCache* state_cache{nullptr};
        double axis_units_per_degree{(16384.0 * 100.0) / 360.0};
        double software_gear_ratio{100.0};
        bool invert_direction{false};
        std::uint16_t default_speed_rpm{1800};
        std::uint8_t default_accel_byte{255};
        std::function<motion_core::Result<void>(std::uint16_t)> on_runtime_can_id_changed{};
    };

    explicit MksAxisConfigurator(Config config);

    motion_core::Result<void> start();
    motion_core::Result<void> stop();

    [[nodiscard]] motion_core::Result<std::vector<motion_core::ParameterDescriptor>> list_parameters() const;
    [[nodiscard]] motion_core::Result<motion_core::ParameterSet> read_parameters() const;
    motion_core::Result<void> apply_parameter_patch(const motion_core::ParameterPatch& patch);
    [[nodiscard]] motion_core::Result<motion_core::PersistentWriteReport> set_persistent(
        motion_core::PersistentCommand command,
        const motion_core::ParameterValue& value);

    [[nodiscard]] double software_gear_ratio() const;
    [[nodiscard]] std::uint32_t software_encoder_resolution_bits() const;
    [[nodiscard]] double axis_units_per_degree() const;
    [[nodiscard]] bool software_invert_direction() const;
    [[nodiscard]] double limits_max_velocity_deg_per_sec() const;
    [[nodiscard]] double limits_max_accel_percent() const;

private:
    [[nodiscard]] motion_core::Result<void> ensure_started() const;
    [[nodiscard]] std::uint16_t runtime_can_id() const;

    Config config_{};
    std::atomic<bool> started_{false};
    std::atomic<std::uint16_t> runtime_can_id_{1};

    std::atomic<double> software_gear_ratio_{100.0};
    std::atomic<std::uint32_t> software_encoder_resolution_bits_{14};
    std::atomic<double> axis_units_per_degree_{(16384.0 * 100.0) / 360.0};
    std::atomic<bool> software_invert_direction_{false};
    std::atomic<double> limits_max_velocity_deg_per_sec_{108.0};
    std::atomic<double> limits_max_accel_percent_{100.0};
};

} // namespace mks
