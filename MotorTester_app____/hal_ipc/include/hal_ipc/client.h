#pragma once

#include "hal_ipc/protocol.h"
#include "motion_core/result.h"

#include <cstdint>
#include <string>

namespace hal_ipc {

class HalIpcClient {
public:
    HalIpcClient() = default;
    ~HalIpcClient();

    HalIpcClient(const HalIpcClient&) = delete;
    HalIpcClient& operator=(const HalIpcClient&) = delete;

    motion_core::Result<void> connect_to(const std::string& host, std::uint16_t port, int timeout_ms);
    motion_core::Result<void> disconnect();
    [[nodiscard]] bool is_connected() const noexcept;

    motion_core::Result<HalStateFrameDto> exchange_control_frame(const HalControlFrameDto& frame);
    motion_core::Result<void> send_control_frame(const HalControlFrameDto& frame);
    motion_core::Result<HalStateFrameDto> request_state_snapshot();

private:
    motion_core::Result<void> send_line(const std::string& line);
    motion_core::Result<std::string> recv_line();

    int socket_fd_{-1};
    std::string rx_buffer_{};
};

} // namespace hal_ipc
