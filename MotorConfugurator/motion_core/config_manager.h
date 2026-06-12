#pragma once

#include "motion_core/hal_runtime.h"
#include "motion_core/result.h"
#include "motion_core/hal_runtime_config.h"
#include <string>

namespace motion_core {

class ConfigManager {
public:
    explicit ConfigManager(HalRuntime& runtime);

    // Loads HAL config from JSON project directory or master JSON file.
    // Automatically resolves relative axis config paths, checks for duplicate IDs,
    // and canonicalizes bus references.
    Result<HalRuntimeConfig> load_master_config(const std::string& project_path);

    // Saves the current active runtime config to the specified project directory.
    // It creates an 'axes' subdirectory and exports individual axis JSONs,
    // then writes the master JSON.
    Result<void> save_master_config(const std::string& project_dir);

    Result<void> export_axis_config(std::uint16_t axis_id, const std::string& path);
    Result<ParameterPatch> preview_axis_config(std::uint16_t axis_id, const std::string& path);
    Result<void> apply_axis_config(std::uint16_t axis_id, const std::string& path);

private:
    HalRuntime& runtime_;
};

} // namespace motion_core
