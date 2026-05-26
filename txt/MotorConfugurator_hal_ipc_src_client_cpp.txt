#include "hal_ipc/client.h"

#include <algorithm>
#include <limits>

#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <winsock2.h>
#include <ws2tcpip.h>
#else
#include <arpa/inet.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>
#endif

namespace hal_ipc {

namespace {

#ifdef _WIN32

[[nodiscard]] motion_core::Result<void> ensure_winsock_started() {
    static const struct WinsockRuntime final {
        int startup_result{0};

        WinsockRuntime() {
            WSADATA data{};
            startup_result = ::WSAStartup(MAKEWORD(2, 2), &data);
        }

        ~WinsockRuntime() {
            if (startup_result == 0) {
                ::WSACleanup();
            }
        }
    } runtime{};

    if (runtime.startup_result != 0) {
        return motion_core::Result<void>::failure(
            {motion_core::ErrorCode::TransportFailure, "WSAStartup failed in HalIpcClient"});
    }
    return motion_core::Result<void>::success();
}

[[nodiscard]] SOCKET to_native_socket(const SocketHandle handle) {
    return static_cast<SOCKET>(handle);
}

void close_socket(SocketHandle& handle) {
    if (handle != kInvalidSocketHandle) {
        (void)::closesocket(to_native_socket(handle));
        handle = kInvalidSocketHandle;
    }
}

#else

void close_socket(SocketHandle& handle) {
    if (handle != kInvalidSocketHandle) {
        (void)::close(static_cast<int>(handle));
        handle = kInvalidSocketHandle;
    }
}

#endif

} // namespace

HalIpcClient::~HalIpcClient() {
    (void)disconnect();
}

motion_core::Result<void> HalIpcClient::connect_to(const std::string& host, const std::uint16_t port, const int timeout_ms) {
#ifdef _WIN32
    const auto winsock = ensure_winsock_started();
    if (!winsock.ok()) {
        return winsock;
    }
#endif

    if (socket_fd_ != kInvalidSocketHandle) {
        return motion_core::Result<void>::success();
    }

#ifdef _WIN32
    const SOCKET fd = ::socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (fd == INVALID_SOCKET) {
        return motion_core::Result<void>::failure({motion_core::ErrorCode::TransportFailure, "socket() failed in HalIpcClient"});
    }

    const DWORD timeout = static_cast<DWORD>(timeout_ms > 0 ? timeout_ms : 1);
    (void)::setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, reinterpret_cast<const char*>(&timeout), sizeof(timeout));
    (void)::setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, reinterpret_cast<const char*>(&timeout), sizeof(timeout));
#else
    const int fd = ::socket(AF_INET, SOCK_STREAM, 0);
    if (fd < 0) {
        return motion_core::Result<void>::failure({motion_core::ErrorCode::TransportFailure, "socket() failed in HalIpcClient"});
    }

    timeval tv{};
    tv.tv_sec = timeout_ms / 1000;
    tv.tv_usec = (timeout_ms % 1000) * 1000;
    (void)::setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    (void)::setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));
#endif

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);
    if (::inet_pton(AF_INET, host.c_str(), &addr.sin_addr) != 1) {
#ifdef _WIN32
        (void)::closesocket(fd);
#else
        (void)::close(fd);
#endif
        return motion_core::Result<void>::failure({motion_core::ErrorCode::InvalidArgument, "invalid host IPv4 address"});
    }

    if (::connect(fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0) {
#ifdef _WIN32
        (void)::closesocket(fd);
#else
        (void)::close(fd);
#endif
        return motion_core::Result<void>::failure({motion_core::ErrorCode::NotConnected, "connect() failed in HalIpcClient"});
    }

    const int one = 1;
    (void)::setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, reinterpret_cast<const char*>(&one), sizeof(one));

    socket_fd_ = static_cast<SocketHandle>(fd);
    rx_buffer_.clear();
    return motion_core::Result<void>::success();
}

motion_core::Result<void> HalIpcClient::disconnect() {
    close_socket(socket_fd_);
    rx_buffer_.clear();
    return motion_core::Result<void>::success();
}

bool HalIpcClient::is_connected() const noexcept {
    return socket_fd_ != kInvalidSocketHandle;
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
    if (socket_fd_ == kInvalidSocketHandle) {
        return motion_core::Result<void>::failure({motion_core::ErrorCode::NotConnected, "client is not connected"});
    }

    const std::string payload = line + "\n";
    const char* ptr = payload.data();
    std::size_t left = payload.size();
    while (left > 0U) {
#ifdef _WIN32
        const int chunk = static_cast<int>(std::min<std::size_t>(left, static_cast<std::size_t>(std::numeric_limits<int>::max())));
        const int written = ::send(to_native_socket(socket_fd_), ptr, chunk, 0);
#else
        const auto written = ::send(static_cast<int>(socket_fd_), ptr, left, 0);
#endif
        if (written <= 0) {
            return motion_core::Result<void>::failure({motion_core::ErrorCode::TransportFailure, "send() failed in HalIpcClient"});
        }
        ptr += written;
        left -= static_cast<std::size_t>(written);
    }
    return motion_core::Result<void>::success();
}

motion_core::Result<std::string> HalIpcClient::recv_line() {
    if (socket_fd_ == kInvalidSocketHandle) {
        return motion_core::Result<std::string>::failure({motion_core::ErrorCode::NotConnected, "client is not connected"});
    }

    while (true) {
        if (rx_buffer_.size() > 65536U) {
            rx_buffer_.clear();
            return motion_core::Result<std::string>::failure({motion_core::ErrorCode::TransportFailure, "rx buffer overflow"});
        }
        const std::size_t newline_pos = rx_buffer_.find('\n');
        if (newline_pos != std::string::npos) {
            std::string line = rx_buffer_.substr(0, newline_pos);
            rx_buffer_.erase(0, newline_pos + 1U);
            return motion_core::Result<std::string>::success(std::move(line));
        }

        char tmp[4096];
#ifdef _WIN32
        const int received = ::recv(to_native_socket(socket_fd_), tmp, static_cast<int>(sizeof(tmp)), 0);
#else
        const auto received = ::recv(static_cast<int>(socket_fd_), tmp, sizeof(tmp), 0);
#endif
        if (received <= 0) {
            return motion_core::Result<std::string>::failure({motion_core::ErrorCode::TransportFailure, "recv() failed or connection closed in HalIpcClient"});
        }
        rx_buffer_.append(tmp, static_cast<std::size_t>(received));
    }
}

} // namespace hal_ipc
