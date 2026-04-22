#pragma once

#include "hal_ipc/protocol.h"
#include "motion_core/result.h"

#include <atomic>
#include <cstdint>
#include <functional>
#include <string>
#include <thread>
#include <vector>
#include <mutex>

namespace hal_ipc {

class HalIpcServer {
public:
    using ControlHandler = std::function<HalStateFrameDto(const HalControlFrameDto&)>;
    using DisconnectHandler = std::function<void()>;

    HalIpcServer() = default;
    ~HalIpcServer();

    HalIpcServer(const HalIpcServer&) = delete;
    HalIpcServer& operator=(const HalIpcServer&) = delete;

    motion_core::Result<void> start(const std::string& bind_host,
                                    std::uint16_t port,
                                    ControlHandler handler,
                                    DisconnectHandler on_disconnect = nullptr);
    motion_core::Result<void> stop();

    [[nodiscard]] bool is_running() const noexcept;
    [[nodiscard]] int connected_client_count() const noexcept;

private:
    void worker_loop();
    void client_worker(int client_fd);

    motion_core::Result<int> accept_client_with_timeout(int timeout_ms) const;
    motion_core::Result<std::string> recv_line(int client_fd, std::string& rx_buffer) const;
    motion_core::Result<void> send_line(int client_fd, const std::string& line) const;

    int listen_fd_{-1};
    std::atomic<bool> running_{false};
    std::thread worker_{};
    ControlHandler handler_{};
    DisconnectHandler disconnect_handler_{};

    std::vector<std::thread> client_threads_{};
    std::mutex threads_mutex_{};
    std::atomic<int> connected_clients_{0};
};

} // namespace hal_ipc
