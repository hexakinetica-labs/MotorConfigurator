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

#ifndef HAL_IPC_SOCKET_HANDLE_DECLARED
#define HAL_IPC_SOCKET_HANDLE_DECLARED
using SocketHandle = std::intptr_t;
constexpr SocketHandle kInvalidSocketHandle = static_cast<SocketHandle>(-1);
#endif

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
    [[nodiscard]] int connected_hexamotion_client_count() const noexcept;

private:
    void worker_loop();
    void client_worker(SocketHandle client_fd);

    motion_core::Result<SocketHandle> accept_client_with_timeout(int timeout_ms) const;
    motion_core::Result<std::string> recv_line(SocketHandle client_fd, std::string& rx_buffer) const;
    motion_core::Result<void> send_line(SocketHandle client_fd, const std::string& line) const;

    SocketHandle listen_fd_{kInvalidSocketHandle};
    std::atomic<bool> running_{false};
    std::thread worker_{};
    ControlHandler handler_{};
    DisconnectHandler disconnect_handler_{};

    std::atomic<int> connected_clients_{0};
    std::atomic<int> connected_hexamotion_clients_{0};
};

} // namespace hal_ipc
