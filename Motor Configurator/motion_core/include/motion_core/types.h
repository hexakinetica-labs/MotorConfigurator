#pragma once

#include <cstdint>
#include <string>

namespace motion_core {

struct AxisId {
    std::uint16_t value{0};

    [[nodiscard]] bool valid() const noexcept { return value > 0; }

    friend bool operator==(const AxisId& lhs, const AxisId& rhs) noexcept {
        return lhs.value == rhs.value;
    }

    friend bool operator!=(const AxisId& lhs, const AxisId& rhs) noexcept {
        return !(lhs == rhs);
    }
};

enum class AxisTransportKind {
    Ethercat = 0,
    CanBus,
    Unknown
};

enum class AxisState {
    Unknown = 0,
    Disabled,
    Ready,
    OperationEnabled,
    Fault
};

enum class AxisMode {
    ProfilePosition = 0,
    ProfileVelocity,
    CyclicSyncPosition,
    CyclicSyncVelocity,
    CyclicSyncTorque,
    Homing,
    VendorSpecific
};

struct AxisName {
    std::string value;
};

} // namespace motion_core
