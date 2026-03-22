#pragma once

#include "mks/manager/mks_can_bus_manager.h"
#include "mks/adapter/mks_runtime_config.h"
#include "motion_core/axis_interface.h"
#include "motion_core/config/hal_runtime_config.h"
#include "motion_core/runtime_factory_registry.h"

#include <memory>
#include <vector>

namespace mks {

// MksRuntimeBuildResult removed since we use motion_core::RuntimeBuildResult

[[nodiscard]] motion_core::Result<motion_core::RuntimeBuildResult> build_mks_runtime(const motion_core::HalRuntimeConfig& config);

} // namespace mks
