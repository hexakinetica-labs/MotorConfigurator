#pragma once

#include "motion_core/config/axis_config.h"
#include "motion_core/result.h"
#include <string>

namespace motion_core {

/**
 * @brief Загрузка конфигурации оси из JSON файла.
 */
Result<AxisConfig> load_axis_config_from_file(const std::string& path);

/**
 * @brief Сохранение конфигурации оси в JSON файл.
 */
Result<void> save_axis_config_to_file(const std::string& path, const AxisConfig& config);

} // namespace motion_core
