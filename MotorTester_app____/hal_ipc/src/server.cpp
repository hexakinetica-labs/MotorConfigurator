#include "hal_ipc/server.h"

#ifndef _WIN32
#include <arpa/inet.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <unistd.h>
#include <cerrno>
#endif

namespace hal_ipc {

HalIpcServer::~HalIpcServer() {
    (void)stop();
}

motion_core::Result<void> HalIpcServer::start(const std::string& bind_host,
                                              const std::uint16_t port,
                                              ControlHandler handler,
                                              DisconnectHandler on_disconnect) {
#ifdef _WIN32
    (void)bind_host;
    (void)port;
    (void)handler;
    return motion_core::Result<void>::failure({motion_core::ErrorCode::Unsupported, "HalIpcServer is not implemented for Windows"});
#else
    if (running_.load(std::memory_order_acquire)) {
        return motion_core::Result<void>::success();
    }
    if (!handler) {
        return motion_core::Result<void>::failure({motion_core::ErrorCode::InvalidArgument, "server handler is empty"});
    }

    const int fd = ::socket(AF_INET, SOCK_STREAM, 0);
    if (fd < 0) {
        return motion_core::Result<void>::failure({motion_core::ErrorCode::TransportFailure, "socket() failed"});
    }

    const int one = 1;
    (void)::setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);
    if (::inet_pton(AF_INET, bind_host.c_str(), &addr.sin_addr) != 1) {
        ::close(fd);
        return motion_core::Result<void>::failure({motion_core::ErrorCode::InvalidArgument, "invalid bind host"});
    }

    if (::bind(fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0) {
        ::close(fd);
        return motion_core::Result<void>::failure({motion_core::ErrorCode::TransportFailure, "bind() failed"});
    }
    if (::listen(fd, 8) != 0) {
        ::close(fd);
        return motion_core::Result<void>::failure({motion_core::ErrorCode::TransportFailure, "listen() failed"});
    }

    listen_fd_ = fd;
    handler_ = std::move(handler);
    disconnect_handler_ = std::move(on_disconnect);
    running_.store(true, std::memory_order_release);
    worker_ = std::thread(&HalIpcServer::worker_loop, this);
    return motion_core::Result<void>::success();
#endif
}

motion_core::Result<void> HalIpcServer::stop() {
#ifdef _WIN32
    return motion_core::Result<void>::success();
#else
    const bool was_running = running_.exchange(false, std::memory_order_acq_rel);
    if (!was_running) {
        return motion_core::Result<void>::success();
    }

    if (listen_fd_ >= 0) {
        ::close(listen_fd_);
        listen_fd_ = -1;
    }
    if (worker_.joinable()) {
        worker_.join();
    }
    
    {
        std::lock_guard<std::mutex> lock(threads_mutex_);
        for (auto& t : client_threads_) {
            if (t.joinable()) {
                t.join();
            }
        }
        client_threads_.clear();
    }

    handler_ = {};
    disconnect_handler_ = {};
    return motion_core::Result<void>::success();
#endif
}

bool HalIpcServer::is_running() const noexcept {
    return running_.load(std::memory_order_acquire);
}

int HalIpcServer::connected_client_count() const noexcept {
    return connected_clients_.load(std::memory_order_acquire);
}

void HalIpcServer::worker_loop() {
#ifndef _WIN32
    while (running_.load(std::memory_order_acquire)) {
        const auto accepted = accept_client_with_timeout(200);
        if (!accepted.ok()) {
            continue;
        }

        const int client_fd = accepted.value();
        
        std::lock_guard<std::mutex> lock(threads_mutex_);
        client_threads_.emplace_back(&HalIpcServer::client_worker, this, client_fd);
    }
#endif
}

void HalIpcServer::client_worker(int client_fd) {
#ifndef _WIN32
    connected_clients_.fetch_add(1, std::memory_order_acq_rel);
    std::string rx_buffer;
    rx_buffer.reserve(4096);

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

        const auto state = handler_(control.value());
        const auto state_json = serialize_state_frame(state);
        if (!state_json.ok() || !send_line(client_fd, state_json.value()).ok()) {
            break;
        }
    }
    ::close(client_fd);
    connected_clients_.fetch_sub(1, std::memory_order_acq_rel);
    if (disconnect_handler_) {
        disconnect_handler_();
    }
#endif
}

motion_core::Result<int> HalIpcServer::accept_client_with_timeout(const int timeout_ms) const {
#ifdef _WIN32
    (void)timeout_ms;
    return motion_core::Result<int>::failure({motion_core::ErrorCode::Unsupported, "HalIpcServer is not implemented for Windows"});
#else
    if (listen_fd_ < 0) {
        return motion_core::Result<int>::failure({motion_core::ErrorCode::NotConnected, "listen socket is closed"});
    }

    fd_set read_set;
    FD_ZERO(&read_set);
    FD_SET(listen_fd_, &read_set);
    timeval tv{};
    tv.tv_sec = timeout_ms / 1000;
    tv.tv_usec = (timeout_ms % 1000) * 1000;
    const int sel = ::select(listen_fd_ + 1, &read_set, nullptr, nullptr, &tv);
    if (sel <= 0) {
        return motion_core::Result<int>::failure({motion_core::ErrorCode::Timeout, "accept timeout"});
    }

    const int client_fd = ::accept(listen_fd_, nullptr, nullptr);
    if (client_fd < 0) {
        return motion_core::Result<int>::failure({motion_core::ErrorCode::TransportFailure, "accept() failed"});
    }
    
    const int one = 1;
    (void)::setsockopt(client_fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));
    
    tv.tv_sec = 0;
    tv.tv_usec = 200 * 1000;
    (void)::setsockopt(client_fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    
    return motion_core::Result<int>::success(client_fd);
#endif
}

motion_core::Result<std::string> HalIpcServer::recv_line(const int client_fd, std::string& rx_buffer) const {
#ifdef _WIN32
    (void)client_fd;
    (void)rx_buffer;
    return motion_core::Result<std::string>::failure({motion_core::ErrorCode::Unsupported, "HalIpcServer is not implemented for Windows"});
#else
    while (true) {
        if (rx_buffer.size() > 65536) {
            rx_buffer.clear();
            return motion_core::Result<std::string>::failure({motion_core::ErrorCode::TransportFailure, "rx buffer overflow"});
        }
        const std::size_t newline_pos = rx_buffer.find('\n');
        if (newline_pos != std::string::npos) {
            std::string line = rx_buffer.substr(0, newline_pos);
            rx_buffer.erase(0, newline_pos + 1);
            return motion_core::Result<std::string>::success(std::move(line));
        }

        char tmp[4096];
        const auto read_n = ::recv(client_fd, tmp, sizeof(tmp), 0);
        if (read_n < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                if (!running_.load(std::memory_order_acquire)) {
                    return motion_core::Result<std::string>::failure({motion_core::ErrorCode::TransportFailure, "Server shutting down"});
                }
                continue;
            }
            return motion_core::Result<std::string>::failure({motion_core::ErrorCode::TransportFailure, "recv() failed"});
        }
        if (read_n == 0) {
            return motion_core::Result<std::string>::failure({motion_core::ErrorCode::TransportFailure, "client disconnected"});
        }
        rx_buffer.append(tmp, read_n);
    }
#endif
}

motion_core::Result<void> HalIpcServer::send_line(const int client_fd, const std::string& line) const {
#ifdef _WIN32
    (void)client_fd;
    (void)line;
    return motion_core::Result<void>::failure({motion_core::ErrorCode::Unsupported, "HalIpcServer is not implemented for Windows"});
#else
    const std::string payload = line + "\n";
    const char* ptr = payload.data();
    std::size_t left = payload.size();
    while (left > 0U) {
        const auto written = ::send(client_fd, ptr, left, 0);
        if (written <= 0) {
            return motion_core::Result<void>::failure({motion_core::ErrorCode::TransportFailure, "send() failed"});
        }
        ptr += written;
        left -= static_cast<std::size_t>(written);
    }
    return motion_core::Result<void>::success();
#endif
}

} // namespace hal_ipc
