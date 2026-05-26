#include "mks/axis_manager/axis_manager.h"

#include "motion_core/parameter_types.h"

#include <QVariantList>
#include <QVariantMap>
#include <thread>

namespace mks {

namespace {

[[nodiscard]] static QString param_value_to_string(const motion_core::ParameterValue& v) {
    switch (v.type) {
        case motion_core::ParameterValueType::SignedInteger:   return QString::number(v.signed_value);
        case motion_core::ParameterValueType::UnsignedInteger: return QString::number(v.unsigned_value);
        case motion_core::ParameterValueType::FloatingPoint:   return QString::number(v.floating_value);
        case motion_core::ParameterValueType::Boolean:          return v.bool_value ? QStringLiteral("true") : QStringLiteral("false");
    }
    return {};
}

[[nodiscard]] static QString strip_group_root(const QString& group) {
    if (group.startsWith(QStringLiteral("Common/"))) {
        return group.mid(7);
    }
    if (group.startsWith(QStringLiteral("Software Defined HAL/"))) {
        return group.mid(21);
    }
    if (group.startsWith(QStringLiteral("Drive/EtherCAT/"))) {
        return group.mid(15);
    }
    if (group.startsWith(QStringLiteral("Drive/MKS/"))) {
        return group.mid(10);
    }
    if (group.startsWith(QStringLiteral("Drive/"))) {
        return group.mid(6);
    }
    return group;
}

[[nodiscard]] static QString normalize_group_for_display(const motion_core::AxisTransportKind transport,
                                                         const motion_core::ParameterDescriptor& descriptor) {
    const QString raw_group = QString::fromLatin1(descriptor.group).trimmed();
    const QString suffix = strip_group_root(raw_group);
    const auto append_suffix = [&suffix](const QString& root) {
        return suffix.isEmpty() ? root : root + QStringLiteral("/") + suffix;
    };

    if (transport == motion_core::AxisTransportKind::CanBus) {
        if (descriptor.id.domain == motion_core::ParameterDomain::Common) {
            return append_suffix(QStringLiteral("Software Defined HAL"));
        }
        return raw_group.isEmpty() ? append_suffix(QStringLiteral("Drive/MKS")) : raw_group;
    }

    if (transport == motion_core::AxisTransportKind::Ethercat) {
        const bool software_defined = descriptor.id.domain == motion_core::ParameterDomain::Common
            && (descriptor.id.value == static_cast<std::uint32_t>(motion_core::CommonParameter::HardwareGearRatio)
                || descriptor.id.value == static_cast<std::uint32_t>(motion_core::CommonParameter::HomeSwitchToZeroShiftDeg));
        if (software_defined) {
            return append_suffix(QStringLiteral("Software Defined HAL"));
        }
        return append_suffix(QStringLiteral("Drive/EtherCAT"));
    }

    return raw_group.isEmpty() ? QStringLiteral("Ungrouped") : raw_group;
}

template <typename Func, typename Callback>
void runAsyncAxisTask(AxisManager* manager, int axis_id, Func&& bg_task, Callback&& main_callback) {
    std::thread([manager, axis_id, bg = std::forward<Func>(bg_task), cb = std::forward<Callback>(main_callback)]() mutable {
        auto axis_ptr = manager->getAxis(axis_id);
        if (!axis_ptr) {
            using RetType = std::invoke_result_t<Func, std::shared_ptr<motion_core::IAxis>>;
            RetType err = RetType::failure({motion_core::ErrorCode::NotFound, "Axis not found"});
            QMetaObject::invokeMethod(manager, [cb = std::move(cb), err = std::move(err)]() mutable {
                cb(std::move(err));
            }, Qt::QueuedConnection);
            return;
        }
        auto result = bg(axis_ptr);
        QMetaObject::invokeMethod(manager, [cb = std::move(cb), result = std::move(result)]() mutable {
            cb(std::move(result));
        }, Qt::QueuedConnection);
    }).detach();
}

} // namespace

// ---------------------------------------------------------------------------
// requestListParameters
// ---------------------------------------------------------------------------
void AxisManager::requestListParameters(int axis_id) {
    if (parameter_reads_in_progress_.contains(axis_id)) return;
    parameter_reads_in_progress_.insert(axis_id);

    runAsyncAxisTask(this, axis_id,
        [](std::shared_ptr<motion_core::IAxis> axis) {
            return axis->list_parameters();
        },
        [this, axis_id](motion_core::Result<std::vector<motion_core::ParameterDescriptor>> result) {
            parameter_reads_in_progress_.remove(axis_id);
            if (!result.ok()) {
                emit logMessage(QStringLiteral("hal"),
                                QString("Axis %1 list parameters failed: %2")
                                    .arg(axis_id).arg(QString::fromStdString(result.error().message)));
                return;
            }

            const auto axis_ptr = getAxis(axis_id);
            const auto transport = axis_ptr ? axis_ptr->info().transport : motion_core::AxisTransportKind::CanBus;

            QVariantList out;
            for (const auto& pd : result.value()) {
                QVariantMap item;
                item.insert(QStringLiteral("domain"),      static_cast<int>(pd.id.domain));
                item.insert(QStringLiteral("value"),       static_cast<int>(pd.id.value));
                item.insert(QStringLiteral("name"),        QString::fromLatin1(pd.name));
                item.insert(QStringLiteral("group"),       normalize_group_for_display(transport, pd));
                item.insert(QStringLiteral("unit"),        QString::fromLatin1(pd.unit));
                item.insert(QStringLiteral("read_only"),   pd.read_only);
                item.insert(QStringLiteral("persistable"), pd.persistable);
                item.insert(QStringLiteral("has_min"),     pd.has_min);
                item.insert(QStringLiteral("has_max"),     pd.has_max);
                if (pd.has_min) item.insert(QStringLiteral("min_value"), param_value_to_string(pd.min_value));
                if (pd.has_max) item.insert(QStringLiteral("max_value"), param_value_to_string(pd.max_value));
                out.push_back(item);
            }
            emit parameterListReady(axis_id, out);
        });
}

// ---------------------------------------------------------------------------
// requestReadParameters
// ---------------------------------------------------------------------------
void AxisManager::requestReadParameters(int axis_id) {
    if (parameter_reads_in_progress_.contains(axis_id)) return;
    parameter_reads_in_progress_.insert(axis_id);

    runAsyncAxisTask(this, axis_id,
        [](std::shared_ptr<motion_core::IAxis> axis) {
            return axis->read_parameters();
        },
        [this, axis_id](motion_core::Result<motion_core::ParameterSet> read_res) {
            parameter_reads_in_progress_.remove(axis_id);
            if (!read_res.ok()) {
                emit logMessage(QStringLiteral("hal"),
                                QString("Axis %1 read parameters failed: %2")
                                    .arg(axis_id).arg(QString::fromStdString(read_res.error().message)));
                return;
            }

            QVariantList out;
            for (const auto& entry : read_res.value().entries) {
                QVariantMap item;
                item.insert(QStringLiteral("domain"), static_cast<int>(entry.id.domain));
                item.insert(QStringLiteral("value"),  static_cast<int>(entry.id.value));
                item.insert(QStringLiteral("data"),   param_value_to_string(entry.value));
                out.push_back(item);
            }
            emit parametersRead(axis_id, out);
        });
}

// ---------------------------------------------------------------------------
// applyParameterPatch
// ---------------------------------------------------------------------------
void AxisManager::applyParameterPatch(int axis_id, const QVariantList& patch) {
    motion_core::ParameterPatch param_patch{};
    for (const QVariant& entry_v : patch) {
        const QVariantMap entry = entry_v.toMap();
        motion_core::ParameterEntry pe{};
        pe.id.domain = static_cast<motion_core::ParameterDomain>(
            entry.value(QStringLiteral("domain")).toInt());
        pe.id.value  = static_cast<std::uint32_t>(entry.value(QStringLiteral("value")).toInt());

        const QVariant dv = entry.value(QStringLiteral("data"));
        if (dv.typeId() == QMetaType::Bool)
            pe.value = motion_core::ParameterValue::from_bool(dv.toBool());
        else if (dv.typeId() == QMetaType::Double || dv.typeId() == QMetaType::Float)
            pe.value = motion_core::ParameterValue::from_floating(dv.toDouble());
        else {
            bool ok = false;
            const qlonglong sv = dv.toLongLong(&ok);
            pe.value = ok ? motion_core::ParameterValue::from_signed(static_cast<std::int64_t>(sv))
                          : motion_core::ParameterValue::from_floating(dv.toDouble());
        }
        param_patch.entries.push_back(pe);
    }

    parameter_writes_in_progress_.insert(axis_id);
    
    runAsyncAxisTask(this, axis_id,
        [param_patch](std::shared_ptr<motion_core::IAxis> axis) {
            return axis->apply_parameter_patch(param_patch);
        },
        [this, axis_id, param_patch_size = param_patch.entries.size()](motion_core::Result<void> write_res) {
            parameter_writes_in_progress_.remove(axis_id);
            if (!write_res.ok())
                emit logMessage(QStringLiteral("hal"),
                                QString("Axis %1 parameter patch failed: %2")
                                    .arg(axis_id).arg(QString::fromStdString(write_res.error().message)));
            else
                emit logMessage(QStringLiteral("hal"),
                                QString("Axis %1 parameter patch applied (%2 entries)")
                                    .arg(axis_id).arg(param_patch_size));
        });
}

// ---------------------------------------------------------------------------
// setPersistentParameter — write single persistent parameter via patch
// ---------------------------------------------------------------------------
void AxisManager::setPersistentParameter(int axis_id, int domain, int value,
                                         const QString& /*name*/, const QVariant& data) {
    motion_core::ParameterEntry pe{};
    pe.id.domain = static_cast<motion_core::ParameterDomain>(domain);
    pe.id.value  = static_cast<std::uint32_t>(value);

    if (data.typeId() == QMetaType::Bool)
        pe.value = motion_core::ParameterValue::from_bool(data.toBool());
    else if (data.typeId() == QMetaType::Double || data.typeId() == QMetaType::Float)
        pe.value = motion_core::ParameterValue::from_floating(data.toDouble());
    else {
        bool ok = false;
        const qlonglong sv = data.toLongLong(&ok);
        pe.value = ok ? motion_core::ParameterValue::from_signed(static_cast<std::int64_t>(sv))
                      : motion_core::ParameterValue::from_floating(data.toDouble());
    }

    motion_core::ParameterPatch param_patch{};
    param_patch.entries.push_back(pe);

    runAsyncAxisTask(this, axis_id,
        [param_patch](std::shared_ptr<motion_core::IAxis> axis) {
            return axis->apply_parameter_patch(param_patch);
        },
        [this, axis_id, domain, value](motion_core::Result<void> write_res) {
            if (!write_res.ok())
                emit logMessage(QStringLiteral("hal"),
                                QString("Axis %1 set persistent %2/%3 failed: %4")
                                    .arg(axis_id).arg(domain).arg(value)
                                    .arg(QString::fromStdString(write_res.error().message)));
        });
}

} // namespace mks
