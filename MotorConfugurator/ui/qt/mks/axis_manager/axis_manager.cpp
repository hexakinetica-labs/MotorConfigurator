#include "mks/axis_manager/axis_manager.h"
#include "hal_host_service/hal_host_service.h"
#include "mks/axis_manager/axis_lifecycle_controller.h"
#include "mks/axis_manager/axis_motion_controller.h"
#include "mks/axis_manager/axis_config_controller.h"
#include "mks/axis_manager/axis_telemetry_controller.h"

namespace mks {

namespace {
[[nodiscard]] static QString owner_role_to_string(const hal_ipc::OwnerRole role) {
    switch (role) {
        case hal_ipc::OwnerRole::HexaMotion:   return QStringLiteral("hexamotion");
        case hal_ipc::OwnerRole::MotorTesterUi: return QStringLiteral("ui");
        default:                                return QStringLiteral("none");
    }
}
} // namespace

AxisManager::AxisManager(
    hal_host::HalHostService& host_service,
    QObject* parent)
    : QObject(parent)
    , host_service_(host_service)
{
    qRegisterMetaType<motion_core::SineConfig>("motion_core::SineConfig");

    // Construct single-responsibility sub-controllers
    lifecycle_ = std::make_unique<AxisLifecycleController>(
        host_service,
        this);

    motion_ = std::make_unique<AxisMotionController>(
        host_service.runtime(),
        host_service.orchestrator(),
        host_service.trajectory_generator(),
        this);

    config_ = std::make_unique<AxisConfigController>(
        host_service.runtime(),
        host_service.config(),
        this);

    telemetry_ctrl_ = std::make_unique<AxisTelemetryController>(
        host_service.runtime(),
        host_service.telemetry(),
        host_service.parameters(),
        this);

    // Forward logging & connection signals from Lifecycle Controller
    connect(lifecycle_.get(), &AxisLifecycleController::logMessage, this, &AxisManager::logMessage);
    connect(lifecycle_.get(), &AxisLifecycleController::connectionChanged, this, &AxisManager::connectionChanged);
    connect(lifecycle_.get(), &AxisLifecycleController::transportOpenStateChanged, this, &AxisManager::transportOpenStateChanged);
    connect(lifecycle_.get(), &AxisLifecycleController::scanFinished, this, &AxisManager::scanFinished);
    connect(lifecycle_.get(), &AxisLifecycleController::manualTakeoverChanged, this, &AxisManager::manualTakeoverChanged);
    connect(lifecycle_.get(), &AxisLifecycleController::busStatisticsUpdated, this, &AxisManager::busStatisticsUpdated);
    connect(lifecycle_.get(), &AxisLifecycleController::hostStateUpdated, this, &AxisManager::publishHostState);

    // Sync Lifecycle Controller watched axes with Telemetry Controller
    connect(lifecycle_.get(), &AxisLifecycleController::watchedAxesChanged, this, [this]() {
        const auto& watched = lifecycle_->watchedAxes();
        std::vector<uint16_t> ids;
        ids.reserve(watched.size());
        for (int id : watched) {
            ids.push_back(static_cast<uint16_t>(id));
        }
        telemetry_ctrl_->setWatchedAxes(ids);
    });

    // Forward logging & state from Motion Controller
    connect(motion_.get(), &AxisMotionController::logMessage, this, &AxisManager::logMessage);
    connect(motion_.get(), &AxisMotionController::hostStateUpdated, this, &AxisManager::publishHostState);

    // Forward Config Controller signals & sync HAL configs back to lifecycle
    connect(config_.get(), &AxisConfigController::logMessage, this, &AxisManager::logMessage);
    connect(config_.get(), &AxisConfigController::axisConfigPreviewReady, this, &AxisManager::axisConfigPreviewReady);
    connect(config_.get(), &AxisConfigController::halConfigLoaded, this, [this](const motion_core::HalRuntimeConfig& cfg) {
        lifecycle_->setHalConfig(cfg);
        lifecycle_->rebuildRuntimeFromCurrentConfig();
    });

    // Forward Telemetry signals
    connect(telemetry_ctrl_.get(), &AxisTelemetryController::logMessage, this, &AxisManager::logMessage);
    connect(telemetry_ctrl_.get(), &AxisTelemetryController::telemetryUpdated, this, &AxisManager::telemetryUpdated);
    connect(telemetry_ctrl_.get(), &AxisTelemetryController::motionQueueStatsUpdated, this, &AxisManager::motionQueueStatsUpdated);
    connect(telemetry_ctrl_.get(), &AxisTelemetryController::parameterListReady, this, &AxisManager::parameterListReady);
    connect(telemetry_ctrl_.get(), &AxisTelemetryController::parametersRead, this, &AxisManager::parametersRead);
    connect(telemetry_ctrl_.get(), &AxisTelemetryController::parameterPatchCompleted, this, &AxisManager::parameterPatchCompleted);
    connect(telemetry_ctrl_.get(), &AxisTelemetryController::persistentParameterCompleted, this, &AxisManager::persistentParameterCompleted);

    // Orchestrator callbacks
    host_service_.orchestrator().set_log_callback([this](const std::string& tag, const std::string& msg) {
        emit logMessage(QString::fromStdString(tag), QString::fromStdString(msg));
    });

    host_service_.orchestrator().set_homing_progress_callback([this](const motion_core::HomingSequenceProgress& p) {
        QVariantMap progress;
        progress.insert(QStringLiteral("running"), p.running);
        progress.insert(QStringLiteral("status_text"), QString::fromStdString(p.status_text));
        progress.insert(QStringLiteral("total_axes"), p.total_axes);
        progress.insert(QStringLiteral("current_index"), p.current_index);
        if (p.current_index >= 0) {
            progress.insert(QStringLiteral("current_axis_id"), p.current_axis_id);
        }
        emit mksHomingSequenceProgress(progress);
    });
}

AxisManager::~AxisManager() = default;

// ---------------------------------------------------------------------------
// Lifecycle Delegation
// ---------------------------------------------------------------------------

void AxisManager::openDevice(const QString& device_path, int baud_rate) {
    lifecycle_->openDevice(device_path, baud_rate);
}

void AxisManager::openEthercatDevice(const QString& interface_name) {
    lifecycle_->openEthercatDevice(interface_name);
}

void AxisManager::closeMksDevice() {
    lifecycle_->closeMksDevice();
}

void AxisManager::closeEthercatDevice() {
    lifecycle_->closeEthercatDevice();
}

void AxisManager::closeDevice() {
    lifecycle_->closeDevice();
}

void AxisManager::scanMotors(int max_id) {
    lifecycle_->scanMotors(max_id);
}

void AxisManager::scanEthercatMotors() {
    lifecycle_->scanEthercatMotors();
}

void AxisManager::watchAxis(int axis_id, bool enabled) {
    lifecycle_->watchAxis(axis_id, enabled);
}

void AxisManager::setMotionSource(motion_core::ControlOwner source) {
    lifecycle_->setMotionSource(source);
}

motion_core::ControlOwner AxisManager::activeSource() const {
    return lifecycle_->activeSource();
}

void AxisManager::requestManualTakeover(bool enable) {
    lifecycle_->requestManualTakeover(enable);
}

void AxisManager::startRuntime() {
    lifecycle_->startRuntime();
}

void AxisManager::stopRuntime() {
    lifecycle_->stopRuntime();
}

std::shared_ptr<motion_core::IAxis> AxisManager::getAxis(int axis_id) const {
    return lifecycle_->getAxis(axis_id);
}

// ---------------------------------------------------------------------------
// Motion Delegation
// ---------------------------------------------------------------------------

void AxisManager::enqueueMotionBatch(int axis_id, const QVariantList& points) {
    motion_->enqueueMotionBatch(axis_id, points);
}

void AxisManager::enqueueServiceBatch(int axis_id, const QVariantList& commands) {
    motion_->enqueueServiceBatch(axis_id, commands);
}

void AxisManager::emergencyStop(int axis_id) {
    motion_->emergencyStop(axis_id);
}

void AxisManager::startMksHomingSequence(const QList<int>& axis_ids) {
    motion_->startMksHomingSequence(axis_ids);
}

void AxisManager::stopMksHomingSequence() {
    motion_->stopMksHomingSequence();
}

void AxisManager::startSineMode(int axis_id, const motion_core::SineConfig& config) {
    motion_->startSineMode(axis_id, config);
}

void AxisManager::stopSineMode(int axis_id) {
    motion_->stopSineMode(axis_id);
}

bool AxisManager::isSineModeActive(int axis_id) const {
    return motion_->isSineModeActive(axis_id);
}

// ---------------------------------------------------------------------------
// Config Delegation
// ---------------------------------------------------------------------------

void AxisManager::loadHalConfig(const QString& config_path) {
    lifecycle_->resetRuntimeState();
    config_->loadHalConfig(config_path);
}

void AxisManager::saveHalConfig(const QString& config_path) {
    config_->saveHalConfig(config_path);
    // Keep lifecycle's local config synchronized
    lifecycle_->setHalConfig(host_service_.runtime().current_config());
}

void AxisManager::exportAxisConfig(int axis_id, const QString& path) {
    config_->exportAxisConfig(axis_id, path);
}

void AxisManager::importAxisConfigPreview(int axis_id, const QString& path) {
    config_->importAxisConfigPreview(axis_id, path);
}

void AxisManager::importAxisConfig(int axis_id, const QString& path) {
    config_->importAxisConfig(axis_id, path);
}

// ---------------------------------------------------------------------------
// Telemetry Delegation
// ---------------------------------------------------------------------------

void AxisManager::requestListParameters(int axis_id) {
    telemetry_ctrl_->requestListParameters(axis_id);
}

void AxisManager::requestReadParameters(int axis_id) {
    telemetry_ctrl_->requestReadParameters(axis_id);
}

void AxisManager::applyParameterPatch(int axis_id, const QVariantList& patch) {
    telemetry_ctrl_->applyParameterPatch(axis_id, patch);
}

void AxisManager::setPersistentParameter(int axis_id, int domain, int value, const QString& name, const QVariant& data) {
    telemetry_ctrl_->setPersistentParameter(axis_id, domain, value, name, data);
}

// ---------------------------------------------------------------------------
// Host State Publisher
// ---------------------------------------------------------------------------

void AxisManager::publishHostState() {
    motion_core::ControlOwner control_source = lifecycle_->activeSource();
    hal_ipc::OwnerRole motion_owner = (control_source == motion_core::ControlOwner::HexaMotion)
        ? hal_ipc::OwnerRole::HexaMotion
        : hal_ipc::OwnerRole::MotorTesterUi;
    bool manual_override_active = (control_source == motion_core::ControlOwner::UI);
    bool estop_active = host_service_.orchestrator().estop_active();
    bool mks_homing_sequence_ui_lock_active = host_service_.orchestrator().homing_sequence_running();

    bool ipc_server_running = host_service_.ipc().is_running();
    int connected_hexamotion_client_count = host_service_.ipc().connected_client_count();

    QVariantMap state;
    state.insert(QStringLiteral("motion_owner"),             owner_role_to_string(motion_owner));
    state.insert(QStringLiteral("manual_override_active"),   manual_override_active);
    state.insert(QStringLiteral("estop_active"),             estop_active);
    const bool hexamotion_owns = (control_source == motion_core::ControlOwner::HexaMotion);
    state.insert(QStringLiteral("control_source"),
                 hexamotion_owns ? QStringLiteral("hexamotion") : QStringLiteral("ui"));
    state.insert(QStringLiteral("hexamotion_ipc_running"),        ipc_server_running);
    state.insert(QStringLiteral("hexamotion_connected_clients"),  connected_hexamotion_client_count);
    state.insert(QStringLiteral("mks_homing_sequence_active"),    mks_homing_sequence_ui_lock_active);
    emit hostStateUpdated(state);
}

} // namespace mks