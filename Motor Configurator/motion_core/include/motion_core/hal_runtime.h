#pragma once

#include "motion_core/axis_interface.h"
#include "motion_core/bus_manager_interface.h"
#include "motion_core/config/axis_config.h"
#include "motion_core/config/hal_runtime_config.h"
#include "motion_core/result.h"
#include "motion_core/runtime_factory_registry.h"

#include <cstdint>
#include <memory>
#include <mutex>
#include <unordered_map>
#include <vector>

namespace motion_core {

class HalRuntime {
public:
    HalRuntime() = default;

    Result<void> open_from_config(const HalRuntimeConfig& config);
    [[nodiscard]] Result<HalRuntimeConfig> scan_mks_topology(const MksScanRequest& request) const;
    [[nodiscard]] Result<HalRuntimeConfig> scan_ethercat_topology(const EthercatScanRequest& request) const;
    Result<void> close();
    Result<void> start();
    Result<void> stop();

    [[nodiscard]] Result<std::shared_ptr<IAxis>> find_axis(std::uint16_t axis_id) const;
    [[nodiscard]] Result<std::vector<AxisInfo>> list_axes() const;
    [[nodiscard]] std::vector<std::shared_ptr<IBusManager>> bus_managers_snapshot() const;

    [[nodiscard]] Result<void> export_axis_config_to_file(std::uint16_t axis_id, const std::string& path) const;
    [[nodiscard]] Result<ParameterPatch> build_axis_config_patch(std::uint16_t axis_id, const std::string& path) const;
    [[nodiscard]] Result<void> apply_axis_config_patch(std::uint16_t axis_id, const ParameterPatch& patch);
    [[nodiscard]] Result<void> apply_axis_config_file(std::uint16_t axis_id, const std::string& path);

    [[nodiscard]] bool is_open() const;
    [[nodiscard]] bool is_active() const;

private:
    Result<void> merge_runtime_build(const RuntimeBuildResult& build,
                                     std::vector<std::shared_ptr<IBusManager>>& buses,
                                     std::unordered_map<std::uint16_t, std::shared_ptr<IAxis>>& axes) const;

    mutable std::mutex mutex_;
    std::unordered_map<std::uint16_t, std::shared_ptr<IAxis>> axes_;
    std::vector<std::shared_ptr<IBusManager>> buses_;
    HalRuntimeConfig runtime_config_{};
    bool runtime_open_{false};
    bool runtime_active_{false};
};

} // namespace motion_core
