#pragma once

#include "motion_core/config/hal_runtime_config.h"
#include "motion_core/result.h"
#include <string>

namespace motion_core {

/**
 * @brief Загрузка мастер-конфигурации HAL из JSON файла.
 */
Result<HalRuntimeConfig> load_hal_runtime_config_from_file(const std::string& path);

/**
 * @brief Сохранение мастер-конфигурации HAL в JSON файл.
 */
Result<void> save_hal_runtime_config_to_file(const std::string& path, const HalRuntimeConfig& config);

} // namespace motion_core
