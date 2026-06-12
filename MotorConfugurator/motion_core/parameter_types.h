#pragma once

#include "motion_core/parameter_id.h"
#include "motion_core/types.h"

#include <cstdint>
#include <string>
#include <vector>

namespace motion_core {

enum class ParameterValueType {
    SignedInteger = 0,
    UnsignedInteger,
    FloatingPoint,
    Boolean
};

struct ParameterValue {
    ParameterValueType type{ParameterValueType::SignedInteger};
    std::int64_t signed_value{0};
    std::uint64_t unsigned_value{0};
    double floating_value{0.0};
    bool bool_value{false};

    static constexpr ParameterValue from_signed(std::int64_t value) {
        ParameterValue out{};
        out.type = ParameterValueType::SignedInteger;
        out.signed_value = value;
        return out;
    }

    static constexpr ParameterValue from_unsigned(std::uint64_t value) {
        ParameterValue out{};
        out.type = ParameterValueType::UnsignedInteger;
        out.unsigned_value = value;
        return out;
    }

    static constexpr ParameterValue from_floating(double value) {
        ParameterValue out{};
        out.type = ParameterValueType::FloatingPoint;
        out.floating_value = value;
        return out;
    }

    static constexpr ParameterValue from_bool(bool value) {
        ParameterValue out{};
        out.type = ParameterValueType::Boolean;
        out.bool_value = value;
        return out;
    }
};

struct ParameterDescriptor {
    ParameterId id{};
    const char* name{""};
    const char* group{""};
    const char* unit{""};
    bool read_only{false};
    bool persistable{true}; // Должен ли параметр сохраняться в AxisConfig
    bool has_min{false};
    bool has_max{false};
    ParameterValue min_value{};
    ParameterValue max_value{};
    const char* raw_unit{""};
    const char* display_unit{""};
    const char* conversion_id{"NoConversion"};
    const char* semantic_scope{"DriveRaw"};
    const char* conversion_formula{""};
    bool conversion_depends_on_gear_ratio{false};
    bool conversion_depends_on_axial_resolution{false};
};

struct ParameterEntry {
    ParameterId id{};
    ParameterValue value{};
};

struct ParameterPatch {
    std::vector<ParameterEntry> entries;
};

struct ParameterSet {
    std::vector<ParameterEntry> entries;
};

[[nodiscard]] inline std::string build_parameter_group_for_display(const AxisTransportKind transport,
                                                                   const ParameterDescriptor& descriptor) {
    const auto starts_with = [](const std::string& value, const std::string_view prefix) {
        return value.size() >= prefix.size() && value.compare(0, prefix.size(), prefix.data(), prefix.size()) == 0;
    };

    const auto strip_group_root = [&starts_with](const char* raw_group) -> std::string {
        std::string group = raw_group ? std::string(raw_group) : std::string{};
        if (starts_with(group, "Common/")) return group.substr(7);
        if (starts_with(group, "Software Defined HAL/")) return group.substr(21);
        if (starts_with(group, "Drive/EtherCAT/")) return group.substr(15);
        if (starts_with(group, "Drive/MKS/")) return group.substr(10);
        if (starts_with(group, "Drive/")) return group.substr(6);
        return group;
    };

    const auto is_ethercat_software_defined_parameter = [](const ParameterId id) {
        if (id.domain != ParameterDomain::Common) return false;
        return id.value == static_cast<std::uint32_t>(CommonParameter::HardwareGearRatio)
            || id.value == static_cast<std::uint32_t>(CommonParameter::HomeSwitchToZeroShiftDeg);
    };

    const std::string suffix = strip_group_root(descriptor.group);
    const auto append_suffix = [&suffix](const std::string& root) {
        return suffix.empty() ? root : root + "/" + suffix;
    };

    if (transport == AxisTransportKind::CanBus) {
        if (descriptor.id.domain == ParameterDomain::Common) {
            return append_suffix("Software Defined HAL");
        }
        if (descriptor.group && descriptor.group[0] != '\0') {
            return descriptor.group;
        }
        return append_suffix("Drive/MKS");
    }

    if (transport == AxisTransportKind::Ethercat) {
        if (is_ethercat_software_defined_parameter(descriptor.id)) {
            return append_suffix("Software Defined HAL");
        }
        return append_suffix("Drive/EtherCAT");
    }

    if (descriptor.group && descriptor.group[0] != '\0') {
        return descriptor.group;
    }
    return "Ungrouped";
}

} // namespace motion_core
