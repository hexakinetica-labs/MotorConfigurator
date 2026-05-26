#include "ethercat/manager/ethercat_bus_manager.h"
#include "ethercat/adapter/ethercat_axis_adapter.h"

#include <algorithm>
#include <cstring>
#include <cstdio>
#include <iostream>
#include <thread>

namespace ethercat_driver {

namespace {

constexpr uint32_t kSupportedVendorId = 0x00445653;
constexpr uint32_t kSupportedProductCode = 0x00009252;

} // namespace

EthercatBusManager::EthercatBusManager(EthercatBusConfig config)
    : config_(std::move(config)) {}

EthercatBusManager::~EthercatBusManager() {
    (void)close();
}

motion_core::Result<void> EthercatBusManager::open() {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (master_) {
        return motion_core::Result<void>::success();
    }

    master_ = ecrt_request_master(0);
    if (!master_) {
        return motion_core::Result<void>::failure(
            {motion_core::ErrorCode::TransportFailure, "Failed to request EtherCAT master 0"});
    }

    domain_ = ecrt_master_create_domain(master_);
    if (!domain_) {
        ecrt_release_master(master_);
        master_ = nullptr;
        return motion_core::Result<void>::failure(
            {motion_core::ErrorCode::TransportFailure, "Failed to create EtherCAT domain"});
    }

    discovered_slaves_.clear();
    axis_map_.clear();
    position_map_.clear();
    registered_adapters_.clear();
    rt_adapters_dense_.clear();
    rt_master_ = nullptr;
    rt_domain_ = nullptr;
    rt_domain_pd_ = nullptr;
    stats_cycle_rate_hz_.store(0.0, std::memory_order_release);
    stats_bus_load_percent_.store(0.0, std::memory_order_release);

    return motion_core::Result<void>::success();
}

motion_core::Result<std::vector<std::uint8_t>> EthercatBusManager::sdo_upload(
    const std::uint16_t bus_position,
    const std::uint16_t index,
    const std::uint8_t sub_index,
    const std::size_t expected_size) const {
    if (expected_size == 0U || expected_size > 8U) {
        return motion_core::Result<std::vector<std::uint8_t>>::failure(
            {motion_core::ErrorCode::InvalidArgument, "SDO upload expected size must be in range [1..8]"});
    }

    ec_master_t* master = nullptr;
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        if (!master_) {
            return motion_core::Result<std::vector<std::uint8_t>>::failure(
                {motion_core::ErrorCode::NotConnected, "EtherCAT master is not open"});
        }
        master = master_;
    }

    std::uint8_t buffer[8] = {};
    std::size_t result_size = 0;
    std::uint32_t abort_code = 0;

    {
        std::lock_guard<std::mutex> ecrt_lock(ecrt_api_mutex_);
        if (ecrt_master_sdo_upload(master,
                                   bus_position,
                                   index,
                                   sub_index,
                                   buffer,
                                   expected_size,
                                   &result_size,
                                   &abort_code) != 0) {
            return motion_core::Result<std::vector<std::uint8_t>>::failure(
                {motion_core::ErrorCode::TransportFailure, "EtherCAT SDO upload transport failure"});
        }
    }

    if (abort_code != 0U) {
        char err_msg[128];
        std::snprintf(err_msg, sizeof(err_msg), "EtherCAT SDO upload aborted by slave (abort code: 0x%08X)", abort_code);
        return motion_core::Result<std::vector<std::uint8_t>>::failure(
            {motion_core::ErrorCode::ProtocolFailure, err_msg});
    }

    std::vector<std::uint8_t> out(expected_size, 0U);
    std::memcpy(out.data(), buffer, std::min(result_size, expected_size));

    return motion_core::Result<std::vector<std::uint8_t>>::success(std::move(out));
}

motion_core::Result<void> EthercatBusManager::sdo_download(
    const std::uint16_t bus_position,
    const std::uint16_t index,
    const std::uint8_t sub_index,
    const std::vector<std::uint8_t>& payload) {
    if (payload.empty() || payload.size() > 8U) {
        return motion_core::Result<void>::failure(
            {motion_core::ErrorCode::InvalidArgument, "SDO download payload size must be in range [1..8]"});
    }

    ec_master_t* master = nullptr;
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        if (!master_) {
            return motion_core::Result<void>::failure(
                {motion_core::ErrorCode::NotConnected, "EtherCAT master is not open"});
        }
        master = master_;
    }

    std::uint32_t abort_code = 0;
    {
        std::lock_guard<std::mutex> ecrt_lock(ecrt_api_mutex_);
        if (ecrt_master_sdo_download(master,
                                     bus_position,
                                     index,
                                     sub_index,
                                     payload.data(),
                                     payload.size(),
                                     &abort_code) != 0) {
            return motion_core::Result<void>::failure(
                {motion_core::ErrorCode::TransportFailure, "EtherCAT SDO download transport failure"});
        }
    }

    if (abort_code != 0U) {
        char err_msg[128];
        std::snprintf(err_msg, sizeof(err_msg), "EtherCAT SDO download aborted by slave (abort code: 0x%08X)", abort_code);
        return motion_core::Result<void>::failure(
            {motion_core::ErrorCode::ProtocolFailure, err_msg});
    }

    return motion_core::Result<void>::success();
}

motion_core::Result<void> EthercatBusManager::close() {
    (void)stop();

    std::lock_guard<std::mutex> lock(state_mutex_);
    if (master_) {
        ecrt_release_master(master_);
        master_ = nullptr;
        domain_ = nullptr;
    }
    discovered_slaves_.clear();
    axis_map_.clear();
    position_map_.clear();
    registered_adapters_.clear();
    rt_adapters_dense_.clear();
    rt_master_ = nullptr;
    rt_domain_ = nullptr;
    rt_domain_pd_ = nullptr;
    stats_cycle_rate_hz_.store(0.0, std::memory_order_release);
    stats_bus_load_percent_.store(0.0, std::memory_order_release);

    return motion_core::Result<void>::success();
}

bool EthercatBusManager::is_open() const noexcept {
    return master_ != nullptr;
}

motion_core::Result<std::vector<std::uint16_t>> EthercatBusManager::scan_axes() {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (!master_) {
        return motion_core::Result<std::vector<std::uint16_t>>::failure(
            {motion_core::ErrorCode::NotConnected, "EtherCAT master is not open"});
    }
    if (started_.load(std::memory_order_acquire) || rt_loop_active_.load(std::memory_order_acquire)) {
        return motion_core::Result<std::vector<std::uint16_t>>::failure(
            {motion_core::ErrorCode::Busy,
             "EtherCAT runtime is active; scan is disabled to protect RT loop"});
    }

    ec_master_info_t master_info{};
    bool info_ok = false;
    for (int attempt = 0; attempt < 5; ++attempt) {
        if (ecrt_master(master_, &master_info) == 0) {
            info_ok = true;
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    if (!info_ok) {
        return motion_core::Result<std::vector<std::uint16_t>>::failure(
            {motion_core::ErrorCode::TransportFailure, "Failed to get master info"});
    }

    discovered_slaves_.clear();
    axis_map_.clear();
    position_map_.clear();
    std::vector<std::uint16_t> out;

    int runtime_index = 0;
    for (uint16_t i = 0; i < master_info.slave_count; ++i) {
        ec_slave_info_t slave_info{};
        if (ecrt_master_get_slave(master_, i, &slave_info) != 0) continue;
        if (slave_info.vendor_id != kSupportedVendorId ||
            slave_info.product_code != kSupportedProductCode) {
            continue;
        }

        SlaveInfo info;
        info.axis_id = i + 1; // Assuming sequential ID starting from 1
        info.bus_position = i;
        info.vendor_id = slave_info.vendor_id;
        info.product_code = slave_info.product_code;
        discovered_slaves_.push_back(info);

        SlaveBusInfo bus_info;
        bus_info.runtime_index = runtime_index;
        bus_info.bus_position = info.bus_position;
        axis_map_[info.axis_id] = bus_info;
        position_map_[info.bus_position] = bus_info;
        
        out.push_back(info.axis_id);
        ++runtime_index;
    }

    return motion_core::Result<std::vector<std::uint16_t>>::success(std::move(out));
}

motion_core::Result<motion_core::HalBusConfigEthercat> EthercatBusManager::discover_ethercat_topology(
    const std::string& interface_name) {
    if (interface_name.empty()) {
        return motion_core::Result<motion_core::HalBusConfigEthercat>::failure(
            {motion_core::ErrorCode::InvalidArgument, "interface_name is empty"});
    }

    ec_master_t* master = ecrt_request_master(0);
    if (!master) {
        return motion_core::Result<motion_core::HalBusConfigEthercat>::failure(
            {motion_core::ErrorCode::TransportFailure, "Failed to request EtherCAT master 0 for scanning"});
    }

    ec_master_info_t master_info{};
    bool info_ok = false;
    for (int attempt = 0; attempt < 5; ++attempt) {
        if (ecrt_master(master, &master_info) == 0) {
            info_ok = true;
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    if (!info_ok) {
        ecrt_release_master(master);
        return motion_core::Result<motion_core::HalBusConfigEthercat>::failure(
            {motion_core::ErrorCode::TransportFailure, "Failed to get master info"});
    }

    motion_core::HalBusConfigEthercat bus_cfg{};
    bus_cfg.interface_name = interface_name;

    for (uint16_t i = 0; i < master_info.slave_count; ++i) {
        ec_slave_info_t slave_info{};
        if (ecrt_master_get_slave(master, i, &slave_info) != 0) continue;
        if (slave_info.vendor_id != kSupportedVendorId ||
            slave_info.product_code != kSupportedProductCode) {
            continue;
        }

        uint16_t axis_id = i + 1; // Basic sequential ID mapping
        motion_core::HalAxisRuntimeEntry entry{};
        entry.axis_id = motion_core::AxisId{axis_id};
        entry.axis_name.value = "ECAT " + std::to_string(axis_id);
        entry.transport = motion_core::AxisTransportKind::Ethercat;
        entry.bus_ref = interface_name;
        entry.transport_address = i;
        entry.enable_on_start = false;
        // The frontend loop will merge these discovered axes into a HalRuntimeConfig

        bus_cfg.axes.push_back(std::move(entry));
    }

    ecrt_release_master(master);
    return motion_core::Result<motion_core::HalBusConfigEthercat>::success(std::move(bus_cfg));
}

motion_core::Result<SlaveBusInfo> EthercatBusManager::get_slave_bus_info(const std::uint16_t axis_id) const {
    std::lock_guard<std::mutex> lock(state_mutex_);
    const auto it = axis_map_.find(axis_id);
    if (it == axis_map_.end()) {
        return motion_core::Result<SlaveBusInfo>::failure(
            {motion_core::ErrorCode::NotFound, "Axis ID not found on EtherCAT bus"});
    }
    return motion_core::Result<SlaveBusInfo>::success(it->second);
}

motion_core::Result<SlaveBusInfo> EthercatBusManager::get_slave_bus_info_by_position(
    const std::uint16_t bus_position) const {
    std::lock_guard<std::mutex> lock(state_mutex_);
    const auto it = position_map_.find(bus_position);
    if (it == position_map_.end()) {
        return motion_core::Result<SlaveBusInfo>::failure(
            {motion_core::ErrorCode::NotFound, "Bus position not found on EtherCAT bus"});
    }
    return motion_core::Result<SlaveBusInfo>::success(it->second);
}

motion_core::Result<motion_core::BusStatistics> EthercatBusManager::get_statistics() const {
    motion_core::BusStatistics stats{};
    stats.cycle_rate_hz = stats_cycle_rate_hz_.load(std::memory_order_acquire);
    stats.bus_load_percent = stats_bus_load_percent_.load(std::memory_order_acquire);
    return motion_core::Result<motion_core::BusStatistics>::success(stats);
}

void EthercatBusManager::register_adapter(const int axis_index, EthercatAxisAdapter* adapter) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (started_.load(std::memory_order_acquire) || rt_loop_active_.load(std::memory_order_acquire)) {
        return;
    }
    if (static_cast<std::size_t>(axis_index) >= registered_adapters_.size()) {
        registered_adapters_.resize(axis_index + 1, nullptr);
    }
    registered_adapters_[axis_index] = adapter;
}

motion_core::Result<void> EthercatBusManager::start() {
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        if (started_.load(std::memory_order_acquire)) {
            return motion_core::Result<void>::success();
        }
        if (!master_ || !domain_) {
            return motion_core::Result<void>::failure(
                {motion_core::ErrorCode::NotConnected, "Master or domain not initialized"});
        }

        if (ecrt_master_activate(master_)) {
            return motion_core::Result<void>::failure(
                {motion_core::ErrorCode::TransportFailure, "Failed to activate master"});
        }

        domain_pd_ = ecrt_domain_data(domain_);
        if (!domain_pd_) {
            return motion_core::Result<void>::failure(
                {motion_core::ErrorCode::TransportFailure, "Failed to get domain data pointer"});
        }

        rt_adapters_dense_.clear();
        rt_adapters_dense_.reserve(registered_adapters_.size());
        for (auto* adapter : registered_adapters_) {
            if (adapter != nullptr) {
                rt_adapters_dense_.push_back(adapter);
            }
        }

        rt_master_ = master_;
        rt_domain_ = domain_;
        rt_domain_pd_ = domain_pd_;

        last_stats_time_ = std::chrono::steady_clock::now();
        cycles_since_last_stats_ = 0;
        accrued_cycle_time_us_ = 0;
        last_cycle_time_ = last_stats_time_;
        dc_sync_counter_ = 0;
        stats_cycle_rate_hz_.store(0.0, std::memory_order_release);
        stats_bus_load_percent_.store(0.0, std::memory_order_release);
        started_.store(true, std::memory_order_release);
        rt_loop_active_.store(true, std::memory_order_release);
    }

    const auto loop_result = runtime_loop_.start(config_.cycle_time, [this]() { poll_cycle(); });
    if (!loop_result.ok()) {
        std::lock_guard<std::mutex> lock(state_mutex_);
        started_.store(false, std::memory_order_release);
        rt_loop_active_.store(false, std::memory_order_release);
        rt_master_ = nullptr;
        rt_domain_ = nullptr;
        rt_domain_pd_ = nullptr;
        return loop_result;
    }

    return motion_core::Result<void>::success();
}

motion_core::Result<void> EthercatBusManager::stop() {
    bool was_started = false;
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        if (!started_.load(std::memory_order_acquire)) {
            return motion_core::Result<void>::success();
        }
        was_started = true;
        rt_loop_active_.store(false, std::memory_order_release);
    }

    if (was_started) {
        (void)runtime_loop_.stop();
    }

    std::lock_guard<std::mutex> lock(state_mutex_);
    started_.store(false, std::memory_order_release);
    rt_master_ = nullptr;
    rt_domain_ = nullptr;
    rt_domain_pd_ = nullptr;
    rt_adapters_dense_.clear();
    return motion_core::Result<void>::success();
}

void EthercatBusManager::poll_cycle() {
    if (!rt_loop_active_.load(std::memory_order_acquire)) {
        return;
    }

    ec_master_t* const master = rt_master_;
    ec_domain_t* const domain = rt_domain_;
    uint8_t* const domain_pd = rt_domain_pd_;
    if (!master || !domain || !domain_pd) {
        return;
    }

    auto cycle_start = std::chrono::steady_clock::now();

    // Calculate dt_s for speed calculation
    double dt_s = 0.0;
    if (cycles_since_last_stats_ > 0) { // skip first cycle dt calculation
        dt_s = std::chrono::duration_cast<std::chrono::nanoseconds>(cycle_start - last_cycle_time_).count() * 1e-9;
    }
    last_cycle_time_ = cycle_start;

    if (auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(cycle_start - last_stats_time_).count(); elapsed >= 1000) {
        const double cycle_rate_hz =
            static_cast<double>(cycles_since_last_stats_) * 1000.0 / static_cast<double>(elapsed);
        const double bus_load_percent =
            (static_cast<double>(accrued_cycle_time_us_) / 1000.0) / static_cast<double>(elapsed) * 100.0;
        stats_cycle_rate_hz_.store(cycle_rate_hz, std::memory_order_release);
        stats_bus_load_percent_.store(bus_load_percent, std::memory_order_release);

        cycles_since_last_stats_ = 0;
        accrued_cycle_time_us_ = 0;
        last_stats_time_ = cycle_start;
    }
    ++cycles_since_last_stats_;

    const auto app_time_ns = static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            cycle_start.time_since_epoch())
            .count());
    ecrt_master_application_time(master, app_time_ns);
    ecrt_master_receive(master);
    ecrt_domain_process(domain);

    ecrt_master_sync_reference_clock(master);
    if ((dc_sync_counter_ & 0x03U) == 0U) {
        ecrt_master_sync_slave_clocks(master);
    }
    ++dc_sync_counter_;

    for (auto* adapter : rt_adapters_dense_) {
        adapter->process_cycle(domain_pd, dt_s);
    }

    ecrt_domain_queue(domain);
    ecrt_master_send(master);

    auto cycle_end = std::chrono::steady_clock::now();
    accrued_cycle_time_us_ += std::chrono::duration_cast<std::chrono::microseconds>(cycle_end - cycle_start).count();
}

std::shared_ptr<EthercatBusManager> make_ethercat_bus_manager(EthercatBusConfig config) {
    return std::make_shared<EthercatBusManager>(std::move(config));
}

} // namespace ethercat_driver
