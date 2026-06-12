#include "hal_ipc/server/server.h"

#include <algorithm>
#include <chrono>
#include <limits>

#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <winsock2.h>
#include <ws2tcpip.h>
#else
#include <arpa/inet.h>
#include <cerrno>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/select.h>
#include <sys/socket.h>
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
            {motion_core::ErrorCode::TransportFailure, "WSAStartup failed in HalIpcServer"});
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

[[nodiscard]] bool is_recv_timeout() {
    const int error = ::WSAGetLastError();
    return error == WSAEWOULDBLOCK || error == WSAETIMEDOUT;
}

#else

void close_socket(SocketHandle& handle) {
    if (handle != kInvalidSocketHandle) {
        (void)::close(static_cast<int>(handle));
        handle = kInvalidSocketHandle;
    }
}

[[nodiscard]] bool is_recv_timeout() {
    return errno == EAGAIN || errno == EWOULDBLOCK;
}

#endif

} // namespace

HalIpcServer::~HalIpcServer() {
    (void)stop();
}

motion_core::Result<void> HalIpcServer::start(const std::string& bind_host,
                                              const std::uint16_t port,
                                              ControlHandler handler,
                                              DisconnectHandler on_disconnect) {
#ifdef _WIN32
    const auto winsock = ensure_winsock_started();
    if (!winsock.ok()) {
        return winsock;
    }
#endif

    if (running_.load(std::memory_order_acquire)) {
        return motion_core::Result<void>::success();
    }
    if (!handler) {
        return motion_core::Result<void>::failure({motion_core::ErrorCode::InvalidArgument, "server handler is empty"});
    }

#ifdef _WIN32
    const SOCKET fd = ::socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (fd == INVALID_SOCKET) {
        return motion_core::Result<void>::failure({motion_core::ErrorCode::TransportFailure, "socket() failed in HalIpcServer"});
    }
#else
    const int fd = ::socket(AF_INET, SOCK_STREAM, 0);
    if (fd < 0) {
        return motion_core::Result<void>::failure({motion_core::ErrorCode::TransportFailure, "socket() failed in HalIpcServer"});
    }
#endif

    const int one = 1;
    (void)::setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, reinterpret_cast<const char*>(&one), sizeof(one));

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);
    if (::inet_pton(AF_INET, bind_host.c_str(), &addr.sin_addr) != 1) {
#ifdef _WIN32
        (void)::closesocket(fd);
#else
        (void)::close(fd);
#endif
        return motion_core::Result<void>::failure({motion_core::ErrorCode::InvalidArgument, "invalid bind host"});
    }

    if (::bind(fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0) {
#ifdef _WIN32
        (void)::closesocket(fd);
#else
        (void)::close(fd);
#endif
        return motion_core::Result<void>::failure({motion_core::ErrorCode::TransportFailure, "bind() failed in HalIpcServer"});
    }
    if (::listen(fd, 8) != 0) {
#ifdef _WIN32
        (void)::closesocket(fd);
#else
        (void)::close(fd);
#endif
        return motion_core::Result<void>::failure({motion_core::ErrorCode::TransportFailure, "listen() failed in HalIpcServer"});
    }

    listen_fd_ = static_cast<SocketHandle>(fd);
    handler_ = std::move(handler);
    disconnect_handler_ = std::move(on_disconnect);
    running_.store(true, std::memory_order_release);
    worker_ = std::thread(&HalIpcServer::worker_loop, this);
    return motion_core::Result<void>::success();
}

motion_core::Result<void> HalIpcServer::stop() {
    const bool was_running = running_.exchange(false, std::memory_order_acq_rel);
    if (!was_running) {
        return motion_core::Result<void>::success();
    }

    close_socket(listen_fd_);
    if (worker_.joinable()) {
        worker_.join();
    }

    while (connected_clients_.load(std::memory_order_acquire) > 0) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    handler_ = {};
    disconnect_handler_ = {};
    return motion_core::Result<void>::success();
}

bool HalIpcServer::is_running() const noexcept {
    return running_.load(std::memory_order_acquire);
}

int HalIpcServer::connected_client_count() const noexcept {
    return connected_clients_.load(std::memory_order_acquire);
}

int HalIpcServer::connected_hexamotion_client_count() const noexcept {
    return connected_hexamotion_clients_.load(std::memory_order_acquire);
}

void HalIpcServer::worker_loop() {
    while (running_.load(std::memory_order_acquire)) {
        const auto accepted = accept_client_with_timeout(200);
        if (!accepted.ok()) {
            continue;
        }

        connected_clients_.fetch_add(1, std::memory_order_acq_rel);
        std::thread worker_thread(&HalIpcServer::client_worker, this, accepted.value());
        worker_thread.detach();
    }
}

void HalIpcServer::client_worker(SocketHandle client_fd) {
    bool hexamotion_connected = false;
    std::string rx_buffer;
    rx_buffer.reserve(4096U);

    while (running_.load(std::memory_order_acquire)) {
        const auto line_res = recv_line(client_fd, rx_buffer);
        if (!line_res.ok()) {
            break;
        }

        if (line_res.value() == "STATE_SNAPSHOT") {
            HalControlFrameDto no_op{};
            no_op.op = ControlOp::None;
            const auto state = handler_(no_op);
            const auto state_json = serialize_state_frame(state);
            if (!state_json.ok() || !send_line(client_fd, state_json.value()).ok()) {
                break;
            }
            continue;
        }

        const auto control = deserialize_control_frame(line_res.value());
        if (!control.ok()) {
            break;
        }

        if (!hexamotion_connected && control.value().client_id == kClientIdHexaMotion) {
            connected_hexamotion_clients_.fetch_add(1, std::memory_order_acq_rel);
            hexamotion_connected = true;
        }

        const auto state = handler_(control.value());
        const auto state_json = serialize_state_frame(state);
        if (!state_json.ok() || !send_line(client_fd, state_json.value()).ok()) {
            break;
        }
    }

    close_socket(client_fd);
    if (hexamotion_connected) {
        connected_hexamotion_clients_.fetch_sub(1, std::memory_order_acq_rel);
    }
    connected_clients_.fetch_sub(1, std::memory_order_acq_rel);
    if (disconnect_handler_) {
        disconnect_handler_();
    }
}

motion_core::Result<SocketHandle> HalIpcServer::accept_client_with_timeout(const int timeout_ms) const {
    if (listen_fd_ == kInvalidSocketHandle) {
        return motion_core::Result<SocketHandle>::failure({motion_core::ErrorCode::NotConnected, "listen socket is closed"});
    }

    fd_set read_set;
    FD_ZERO(&read_set);
#ifdef _WIN32
    const SOCKET native_listen = to_native_socket(listen_fd_);
    FD_SET(native_listen, &read_set);
#else
    const int native_listen = static_cast<int>(listen_fd_);
    FD_SET(native_listen, &read_set);
#endif
    timeval tv{};
    tv.tv_sec = timeout_ms / 1000;
    tv.tv_usec = (timeout_ms % 1000) * 1000;

#ifdef _WIN32
    const int sel = ::select(0, &read_set, nullptr, nullptr, &tv);
#else
    const int sel = ::select(native_listen + 1, &read_set, nullptr, nullptr, &tv);
#endif
    if (sel <= 0) {
        return motion_core::Result<SocketHandle>::failure({motion_core::ErrorCode::Timeout, "accept timeout"});
    }

#ifdef _WIN32
    const SOCKET client_fd = ::accept(native_listen, nullptr, nullptr);
    if (client_fd == INVALID_SOCKET) {
        return motion_core::Result<SocketHandle>::failure({motion_core::ErrorCode::TransportFailure, "accept() failed in HalIpcServer"});
    }
#else
    const int client_fd = ::accept(native_listen, nullptr, nullptr);
    if (client_fd < 0) {
        return motion_core::Result<SocketHandle>::failure({motion_core::ErrorCode::TransportFailure, "accept() failed in HalIpcServer"});
    }
#endif

    const int one = 1;
    (void)::setsockopt(client_fd, IPPROTO_TCP, TCP_NODELAY, reinterpret_cast<const char*>(&one), sizeof(one));

#ifdef _WIN32
    const DWORD timeout = 200U;
    (void)::setsockopt(client_fd, SOL_SOCKET, SO_RCVTIMEO, reinterpret_cast<const char*>(&timeout), sizeof(timeout));
#else
    tv.tv_sec = 0;
    tv.tv_usec = 200 * 1000;
    (void)::setsockopt(client_fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
#endif

    return motion_core::Result<SocketHandle>::success(static_cast<SocketHandle>(client_fd));
}

motion_core::Result<std::string> HalIpcServer::recv_line(const SocketHandle client_fd, std::string& rx_buffer) const {
    while (true) {
        if (rx_buffer.size() > 65536U) {
            rx_buffer.clear();
            return motion_core::Result<std::string>::failure({motion_core::ErrorCode::TransportFailure, "rx buffer overflow"});
        }
        const std::size_t newline_pos = rx_buffer.find('\n');
        if (newline_pos != std::string::npos) {
            std::string line = rx_buffer.substr(0, newline_pos);
            rx_buffer.erase(0, newline_pos + 1U);
            return motion_core::Result<std::string>::success(std::move(line));
        }

        char tmp[4096];
#ifdef _WIN32
        const int read_n = ::recv(to_native_socket(client_fd), tmp, static_cast<int>(sizeof(tmp)), 0);
#else
        const auto read_n = ::recv(static_cast<int>(client_fd), tmp, sizeof(tmp), 0);
#endif
        if (read_n < 0) {
            if (is_recv_timeout()) {
                if (!running_.load(std::memory_order_acquire)) {
                    return motion_core::Result<std::string>::failure({motion_core::ErrorCode::TransportFailure, "server shutting down"});
                }
                continue;
            }
            return motion_core::Result<std::string>::failure({motion_core::ErrorCode::TransportFailure, "recv() failed in HalIpcServer"});
        }
        if (read_n == 0) {
            return motion_core::Result<std::string>::failure({motion_core::ErrorCode::TransportFailure, "client disconnected"});
        }
        rx_buffer.append(tmp, static_cast<std::size_t>(read_n));
    }
}

motion_core::Result<void> HalIpcServer::send_line(const SocketHandle client_fd, const std::string& line) const {
    const std::string payload = line + "\n";
    const char* ptr = payload.data();
    std::size_t left = payload.size();
    while (left > 0U) {
#ifdef _WIN32
        const int chunk = static_cast<int>(std::min<std::size_t>(left, static_cast<std::size_t>(std::numeric_limits<int>::max())));
        const int written = ::send(to_native_socket(client_fd), ptr, chunk, 0);
#else
        const auto written = ::send(static_cast<int>(client_fd), ptr, left, 0);
#endif
        if (written <= 0) {
            return motion_core::Result<void>::failure({motion_core::ErrorCode::TransportFailure, "send() failed in HalIpcServer"});
        }
        ptr += written;
        left -= static_cast<std::size_t>(written);
    }
    return motion_core::Result<void>::success();
}

} // namespace hal_ipc
