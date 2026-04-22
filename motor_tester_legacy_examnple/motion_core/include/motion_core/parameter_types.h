#pragma once

#include "motion_core/parameter_id.h"

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

} // namespace motion_core
