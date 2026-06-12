#pragma once

#include <QString>
#include <QVariant>
#include "motion_core/parameter_types.h"

// Named constants for service command kinds passed through QVariantMap.
// These map to motion_core::ServiceCommandType in axis_motion_controller.cpp.
enum class ServiceCommandKind : int {
    ClearMotionQueue = 0,
    Enable          = 1,
    Disable         = 2,
    ClearErrors     = 3,
    Home            = 4,
    SetZero         = 5,
};

// Named constants for motion point kinds passed through QVariantMap.
enum class MotionPointKind : int {
    Stream   = 0,
    Position = 1,
    Velocity = 2,
    Relative = 3,
};

namespace mks {

[[nodiscard]] inline QString param_value_to_string(const motion_core::ParameterValue& v) {
    switch (v.type) {
        case motion_core::ParameterValueType::SignedInteger:   return QString::number(v.signed_value);
        case motion_core::ParameterValueType::UnsignedInteger: return QString::number(v.unsigned_value);
        case motion_core::ParameterValueType::FloatingPoint:   return QString::number(v.floating_value);
        case motion_core::ParameterValueType::Boolean:          return v.bool_value ? QStringLiteral("true") : QStringLiteral("false");
    }
    return {};
}

[[nodiscard]] inline motion_core::ParameterValue qvariant_to_param_value(const QVariant& data) {
    if (data.typeId() == QMetaType::Bool) {
        return motion_core::ParameterValue::from_bool(data.toBool());
    } else if (data.typeId() == QMetaType::Double || data.typeId() == QMetaType::Float) {
        return motion_core::ParameterValue::from_floating(data.toDouble());
    } else {
        bool ok = false;
        const qlonglong sv = data.toLongLong(&ok);
        return ok ? motion_core::ParameterValue::from_signed(static_cast<std::int64_t>(sv))
                  : motion_core::ParameterValue::from_floating(data.toDouble());
    }
}

} // namespace mks
