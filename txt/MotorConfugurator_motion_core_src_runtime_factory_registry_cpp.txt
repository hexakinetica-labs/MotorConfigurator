#include "motion_core/runtime_factory_registry.h"
#include <unordered_map>
#include <mutex>

namespace motion_core {

namespace {
    std::mutex registry_mutex;
    std::unordered_map<AxisTransportKind, RuntimeFactoryFn> registry;
    MksTopologyScannerFn mks_scanner;
    EthercatTopologyScannerFn ethercat_scanner;
}

void RuntimeFactoryRegistry::register_factory(AxisTransportKind kind, RuntimeFactoryFn factory) {
    std::lock_guard<std::mutex> lock(registry_mutex);
    registry[kind] = std::move(factory);
}

RuntimeFactoryFn RuntimeFactoryRegistry::get_factory(AxisTransportKind kind) {
    std::lock_guard<std::mutex> lock(registry_mutex);
    auto it = registry.find(kind);
    if (it != registry.end()) {
        return it->second;
    }
    return nullptr;
}

void RuntimeFactoryRegistry::register_mks_topology_scanner(MksTopologyScannerFn scanner) {
    std::lock_guard<std::mutex> lock(registry_mutex);
    mks_scanner = std::move(scanner);
}

MksTopologyScannerFn RuntimeFactoryRegistry::get_mks_topology_scanner() {
    std::lock_guard<std::mutex> lock(registry_mutex);
    return mks_scanner;
}

void RuntimeFactoryRegistry::register_ethercat_topology_scanner(EthercatTopologyScannerFn scanner) {
    std::lock_guard<std::mutex> lock(registry_mutex);
    ethercat_scanner = std::move(scanner);
}

EthercatTopologyScannerFn RuntimeFactoryRegistry::get_ethercat_topology_scanner() {
    std::lock_guard<std::mutex> lock(registry_mutex);
    return ethercat_scanner;
}

} // namespace motion_core
