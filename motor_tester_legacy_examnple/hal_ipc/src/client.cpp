#include "hal_ipc/client.h"

#include <cerrno>
#include <cstring>

#ifndef _WIN32
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <netinet/tcp.h>
#include <unistd.h>
#endif

namespace hal_ipc {

HalIpcClient::~HalIpcClient() {
    (void)disconnect();
}

motion_core::Result<void> HalIpcClient::connect_to(const std::string& host, const std::uint16_t port, const int timeout_ms) {
#ifdef _WIN32
    (void)host;
    (void)port;
    (void)timeout_ms;
    return motion_core::Result<void>::failure({motion_core::ErrorCode::Unsupported, "HalIpcClient is not implemented for Windows"});
#else
    if (socket_fd_ >= 0) {
        return motion_core::Result<void>::success();
    }

    const int fd = ::socket(AF_INET, SOCK_STREAM, 0);
    if (fd < 0) {
        return motion_core::Result<void>::failure({motion_core::ErrorCode::TransportFailure, "socket() failed"});
    }

    timeval tv{};
    tv.tv_sec = timeout_ms / 1000;
    tv.tv_usec = (timeout_ms % 1000) * 1000;
    (void)::setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    (void)::setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);
    if (::inet_pton(AF_INET, host.c_str(), &addr.sin_addr) != 1) {
        ::close(fd);
        return motion_core::Result<void>::failure({motion_core::ErrorCode::InvalidArgument, "invalid host IPv4 address"});
    }

    if (::connect(fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0) {
        ::close(fd);
        return motion_core::Result<void>::failure({motion_core::ErrorCode::NotConnected, "connect() failed"});
    }

    const int one = 1;
    (void)::setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));

    socket_fd_ = fd;
    rx_buffer_.clear();
    return motion_core::Result<void>::success();
#endif
}

motion_core::Result<void> HalIpcClient::disconnect() {
#ifdef _WIN32
    return motion_core::Result<void>::success();
#else
    if (socket_fd_ >= 0) {
        ::close(socket_fd_);
        socket_fd_ = -1;
    }
    return motion_core::Result<void>::success();
#endif
}

bool HalIpcClient::is_connected() const noexcept {
    return socket_fd_ >= 0;
}

motion_core::Result<HalStateFrameDto> HalIpcClient::exchange_control_frame(const HalControlFrameDto& frame) {
    const auto serialized = serialize_control_frame(frame);
    if (!serialized.ok()) {
        return motion_core::Result<HalStateFrameDto>::failure(serialized.error());
    }

    const auto send_res = send_line(serialized.value());
    if (!send_res.ok()) {
        return motion_core::Result<HalStateFrameDto>::failure(send_res.error());
    }

    const auto line = recv_line();
    if (!line.ok()) {
        return motion_core::Result<HalStateFrameDto>::failure(line.error());
    }

    return deserialize_state_frame(line.value());
}

motion_core::Result<HalStateFrameDto> HalIpcClient::request_state_snapshot() {
    const auto sent = send_line("STATE_SNAPSHOT");
    if (!sent.ok()) {
        return motion_core::Result<HalStateFrameDto>::failure(sent.error());
    }

    const auto line = recv_line();
    if (!line.ok()) {
        return motion_core::Result<HalStateFrameDto>::failure(line.error());
    }

    return deserialize_state_frame(line.value());
}

motion_core::Result<void> HalIpcClient::send_line(const std::string& line) {
#ifdef _WIN32
    (void)line;
    return motion_core::Result<void>::failure({motion_core::ErrorCode::Unsupported, "HalIpcClient is not implemented for Windows"});
#else
    if (socket_fd_ < 0) {
        return motion_core::Result<void>::failure({motion_core::ErrorCode::NotConnected, "client is not connected"});
    }

    const std::string payload = line + "\n";
    const auto* ptr = payload.data();
    std::size_t left = payload.size();
    while (left > 0U) {
        const auto written = ::send(socket_fd_, ptr, left, 0);
        if (written <= 0) {
            return motion_core::Result<void>::failure({motion_core::ErrorCode::TransportFailure, "send() failed"});
        }
        ptr += written;
        left -= static_cast<std::size_t>(written);
    }
    return motion_core::Result<void>::success();
#endif
}

motion_core::Result<std::string> HalIpcClient::recv_line() {
#ifdef _WIN32
    return motion_core::Result<std::string>::failure({motion_core::ErrorCode::Unsupported, "HalIpcClient is not implemented for Windows"});
#else
    if (socket_fd_ < 0) {
        return motion_core::Result<std::string>::failure({motion_core::ErrorCode::NotConnected, "client is not connected"});
    }

    while (true) {
        if (rx_buffer_.size() > 65536) {
            rx_buffer_.clear();
            return motion_core::Result<std::string>::failure({motion_core::ErrorCode::TransportFailure, "rx buffer overflow"});
        }
        const std::size_t newline_pos = rx_buffer_.find('\n');
        if (newline_pos != std::string::npos) {
            std::string line = rx_buffer_.substr(0, newline_pos);
            rx_buffer_.erase(0, newline_pos + 1);
            return motion_core::Result<std::string>::success(std::move(line));
        }

        char tmp[4096];
        const auto received = ::recv(socket_fd_, tmp, sizeof(tmp), 0);
        if (received <= 0) {
            return motion_core::Result<std::string>::failure({motion_core::ErrorCode::TransportFailure, "recv() failed or connection closed"});
        }
        rx_buffer_.append(tmp, received);
    }
#endif
}

} // namespace hal_ipc
