#include "motion_core/config_manager.h"
#include "motion_core/axis_config_json.h"
#include "motion_core/hal_runtime_config_json.h"

#include <filesystem>
#include <unordered_set>

namespace {
namespace fs = std::filesystem;

constexpr const char* kHalProjectMasterFileName = "hal_config.json";
constexpr const char* kHalProjectAxisDirectoryName = "axes";
constexpr const char* kMksTransportBusRef = motion_core::kMksTransportBusRef;
constexpr const char* kEthercatTransportBusRef = motion_core::kEthercatTransportBusRef;

[[nodiscard]] static fs::path hal_project_master_path(const fs::path& project_dir) {
    if (!fs::is_directory(project_dir)) {
        return project_dir;
    }
    return project_dir / kHalProjectMasterFileName;
}

[[nodiscard]] static fs::path hal_project_axis_relative_path(const std::uint16_t axis_id) {
    return fs::path(kHalProjectAxisDirectoryName) / (std::string("axis_") + std::to_string(axis_id) + ".json");
}

[[nodiscard]] static fs::path resolve_config_reference(const fs::path& master_config_path,
                                                       const std::string& config_reference) {
    const fs::path config_path(config_reference);
    if (config_path.is_absolute()) {
        return config_path;
    }
    return master_config_path.parent_path() / config_path;
}

[[nodiscard]] static bool has_duplicate_axis_ids(const motion_core::HalRuntimeConfig& cfg,
                                                 std::uint16_t* duplicate_id = nullptr) {
    std::unordered_set<std::uint16_t> ids;
    ids.reserve(cfg.axes.size());
    for (const auto& axis : cfg.axes) {
        if (!axis.axis_id.valid()) {
            continue;
        }
        if (!ids.insert(axis.axis_id.value).second) {
            if (duplicate_id) {
                *duplicate_id = axis.axis_id.value;
            }
            return true;
        }
    }
    return false;
}
} // namespace

namespace motion_core {

ConfigManager::ConfigManager(HalRuntime& runtime) : runtime_(runtime) {}

Result<HalRuntimeConfig> ConfigManager::load_master_config(const std::string& project_path) {
    const fs::path master_config_path = hal_project_master_path(fs::path(project_path));
    
    auto hal_cfg_res = load_hal_runtime_config_from_file(master_config_path.string());
    if (!hal_cfg_res.ok()) {
        return hal_cfg_res;
    }

    auto hal_cfg = hal_cfg_res.value();
    std::uint16_t duplicate_id = 0U;
    if (has_duplicate_axis_ids(hal_cfg, &duplicate_id)) {
        return Result<HalRuntimeConfig>::failure({ErrorCode::InternalError, 
            "Duplicate global axis_id"});
    }

    for (auto& bus : hal_cfg.mks_buses) {
        bus.interface_id = kMksTransportBusRef;
    }
    for (auto& axis : hal_cfg.axes) {
        if (axis.transport == AxisTransportKind::CanBus) {
            axis.bus_ref = kMksTransportBusRef;
        } else if (axis.transport == AxisTransportKind::Ethercat) {
            axis.bus_ref = kEthercatTransportBusRef;
        }

        if (!axis.config_file.empty()) {
            const fs::path resolved = resolve_config_reference(master_config_path, axis.config_file);
            axis.config_file = resolved.string();
        }
    }

    return Result<HalRuntimeConfig>::success(std::move(hal_cfg));
}

Result<void> ConfigManager::save_master_config(const std::string& project_dir_str) {
    if (!runtime_.is_active()) {
        return Result<void>::failure({ErrorCode::InternalError, "Runtime must be active to export per-axis configs"});
    }

    auto current_config = runtime_.current_config();
    const fs::path project_dir(project_dir_str);
    std::error_code ec;
    fs::create_directories(project_dir / kHalProjectAxisDirectoryName, ec);
    if (ec) {
        return Result<void>::failure({ErrorCode::InternalError, "Cannot create project directory"});
    }

    for (auto& axis : current_config.axes) {
        if (!axis.axis_id.valid()) {
            continue;
        }

        const auto relative_axis_config_path = hal_project_axis_relative_path(axis.axis_id.value);
        const auto absolute_axis_config_path = project_dir / relative_axis_config_path;
        
        auto export_res = runtime_.export_axis_config_to_file(axis.axis_id.value, absolute_axis_config_path.string());
        if (!export_res.ok()) {
            return export_res;
        }

        axis.config_file = relative_axis_config_path.generic_string();
    }

    const auto master_config_path = hal_project_master_path(project_dir);
    return save_hal_runtime_config_to_file(master_config_path.string(), current_config);
}

Result<void> ConfigManager::export_axis_config(std::uint16_t axis_id, const std::string& path) {
    return runtime_.export_axis_config_to_file(axis_id, path);
}

Result<ParameterPatch> ConfigManager::preview_axis_config(std::uint16_t axis_id, const std::string& path) {
    return runtime_.build_axis_config_patch(axis_id, path);
}

Result<void> ConfigManager::apply_axis_config(std::uint16_t axis_id, const std::string& path) {
    return runtime_.apply_axis_config_file(axis_id, path);
}

} // namespace motion_core
