#include "hal_host_service/hal_host_service.h"
#include "drivers/interfaces/init_all_drivers.h"
#include "motion_core/config/hal_runtime_config_json.h"

#include <atomic>
#include <chrono>
#include <csignal>
#include <iostream>
#include <string>
#include <thread>

namespace {

std::atomic<bool> g_running{true};

void handle_signal(const int) {
    g_running.store(false, std::memory_order_release);
}

} // namespace

int main(int argc, char** argv) {
    const std::string config_path = (argc > 1 && argv != nullptr && argv[1] != nullptr)
        ? std::string(argv[1])
        : std::string("config/mixed_runtime_sample.json");

    std::signal(SIGINT, handle_signal);
    std::signal(SIGTERM, handle_signal);

    hal_host_service::HalHostServiceConfig service_cfg{};

    drivers::init_all_drivers();
    motion_core::HalRuntime runtime;
    
    const auto cfg_res = motion_core::load_hal_runtime_config_from_file(config_path);
    if (!cfg_res.ok()) {
        std::cerr << "[motor_hal_host] load config failed: " << cfg_res.error().message << std::endl;
        return 1;
    }
    if (const auto open_res = runtime.open_from_config(cfg_res.value()); !open_res.ok()) {
        std::cerr << "[motor_hal_host] open runtime failed: " << open_res.error().message << std::endl;
        return 1;
    }
    if (const auto start_res = runtime.start(); !start_res.ok()) {
        std::cerr << "[motor_hal_host] start runtime failed: " << start_res.error().message << std::endl;
        return 1;
    }

    hal_host_service::HalHostService service(runtime);

    if (const auto start_res = service.start(service_cfg); !start_res.ok()) {
        std::cerr << "[motor_hal_host] service start failed"
                  << " | code=" << static_cast<int>(start_res.error().code)
                  << " | msg=" << start_res.error().message << std::endl;
        return 2;
    }

    std::cout << "[motor_hal_host] started with config: " << config_path << std::endl;
    std::cout << "[motor_hal_host] waiting for SIGINT/SIGTERM..." << std::endl;

    while (g_running.load(std::memory_order_acquire)) {
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }

    (void)service.stop();
    (void)runtime.stop();
    (void)runtime.close();
    std::cout << "[motor_hal_host] stopped" << std::endl;
    return 0;
}
