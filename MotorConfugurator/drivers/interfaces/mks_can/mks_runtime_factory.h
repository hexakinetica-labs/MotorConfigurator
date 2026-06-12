#pragma once

#include "mks_can/mks_runtime_config.h"
#include "motion_core/hal_runtime_config.h"
#include "motion_core/result.h"
#include "motion_core/runtime_factory_registry.h"

namespace mks {

[[nodiscard]] motion_core::Result<motion_core::RuntimeBuildResult> build_mks_runtime(
    const motion_core::HalRuntimeConfig& config);

} // namespace mks
