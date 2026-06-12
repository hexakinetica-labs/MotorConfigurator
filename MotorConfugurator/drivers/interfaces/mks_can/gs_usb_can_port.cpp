#include "mks_can/gs_usb_can_port.h"

#include <libusb.h>

#include <array>
#include <atomic>
#include <chrono>
#include <cstring>
#include <deque>
#include <mutex>
#include <sstream>
#include <thread>

namespace mks {

namespace {
constexpr uint8_t GS_USB_BREQ_HOST_FORMAT = 0;
constexpr uint8_t GS_USB_BREQ_BITTIMING = 1;
constexpr uint8_t GS_USB_BREQ_MODE = 2;
constexpr uint8_t GS_USB_BREQ_BT_CONST = 4;

constexpr uint32_t GS_USB_MODE_RESET = 0;
constexpr uint32_t GS_USB_MODE_START = 1;
constexpr uint32_t GS_USB_ECHO_ID_UNUSED = 0xFFFFFFFFu;

struct GsUsbFrame {
    uint32_t echo_id;
    uint32_t can_id;
    uint8_t can_dlc;
    uint8_t channel;
    uint8_t flags;
    uint8_t reserved;
    uint8_t data[8];
};

struct UsbInterfaceEndpoints {
    uint8_t interface_number{0};
    uint8_t bulk_in_endpoint{0};
    uint8_t bulk_out_endpoint{0};
};

[[nodiscard]] std::string make_libusb_error_message(const char* step, const int code) {
    std::ostringstream oss;
    oss << step << " failed: " << libusb_error_name(code) << " (" << code << ")";
    return oss.str();
}

} // namespace

struct GsUsbCanPort::Impl {
    libusb_context* ctx{nullptr};
    libusb_device_handle* dev{nullptr};
    uint8_t intf_num{0};
    uint8_t bulk_in_ep{0x81};
    uint8_t bulk_out_ep{0x02};
    uint8_t rx_buf[2048]{};
    std::string last_error{};

    std::thread rx_thread{};
    std::atomic<bool> rx_running{false};
    std::mutex rx_mutex{};
    std::deque<CanFrame> rx_queue{};

    Impl() {
        const int rc = libusb_init(&ctx);
        if (rc != 0) {
            last_error = make_libusb_error_message("libusb_init", rc);
            ctx = nullptr;
        }
    }
    ~Impl() {
        if (dev) {
            stop_rx_thread();
            set_mode(GS_USB_MODE_RESET, 0);
            libusb_release_interface(dev, intf_num);
            libusb_close(dev);
        }
        if (ctx) {
            libusb_exit(ctx);
        }
    }

    static uint32_t read_u32(const uint8_t* p) {
        return uint32_t(p[0]) | (uint32_t(p[1]) << 8) | (uint32_t(p[2]) << 16) | (uint32_t(p[3]) << 24);
    }

    static void write_u32(uint8_t* p, uint32_t v) {
        p[0] = static_cast<uint8_t>(v & 0xFF);
        p[1] = static_cast<uint8_t>((v >> 8) & 0xFF);
        p[2] = static_cast<uint8_t>((v >> 16) & 0xFF);
        p[3] = static_cast<uint8_t>((v >> 24) & 0xFF);
    }

    void set_error(std::string message) {
        last_error = std::move(message);
    }

    [[nodiscard]] bool has_context() const noexcept {
        return ctx != nullptr;
    }

    [[nodiscard]] static bool discover_endpoints(libusb_device* device, UsbInterfaceEndpoints* endpoints_out) {
        libusb_config_descriptor* config = nullptr;
        if (libusb_get_active_config_descriptor(device, &config) != 0 || config == nullptr) {
            if (libusb_get_config_descriptor(device, 0, &config) != 0 || config == nullptr) {
                return false;
            }
        }

        bool found = false;
        for (uint8_t interface_index = 0; interface_index < config->bNumInterfaces; ++interface_index) {
            const auto& interface_group = config->interface[interface_index];
            for (int alt_index = 0; alt_index < interface_group.num_altsetting; ++alt_index) {
                const auto& alt = interface_group.altsetting[alt_index];
                UsbInterfaceEndpoints candidate{};
                candidate.interface_number = alt.bInterfaceNumber;

                for (uint8_t endpoint_index = 0; endpoint_index < alt.bNumEndpoints; ++endpoint_index) {
                    const auto& endpoint = alt.endpoint[endpoint_index];
                    const uint8_t transfer_type = endpoint.bmAttributes & LIBUSB_TRANSFER_TYPE_MASK;
                    if (transfer_type != LIBUSB_TRANSFER_TYPE_BULK) {
                        continue;
                    }

                    const uint8_t address = endpoint.bEndpointAddress;
                    if ((address & LIBUSB_ENDPOINT_DIR_MASK) == LIBUSB_ENDPOINT_IN) {
                        candidate.bulk_in_endpoint = address;
                    } else {
                        candidate.bulk_out_endpoint = address;
                    }
                }

                if (candidate.bulk_in_endpoint != 0U && candidate.bulk_out_endpoint != 0U) {
                    if (endpoints_out != nullptr) {
                        *endpoints_out = candidate;
                    }
                    found = true;
                    break;
                }
            }

            if (found) {
                break;
            }
        }

        libusb_free_config_descriptor(config);
        return found;
    }

    bool set_mode(uint32_t mode, uint32_t flags) {
        struct {
            uint32_t mode;
            uint32_t flags;
        } m = {mode, flags};

        return libusb_control_transfer(dev,
                                       static_cast<uint8_t>(LIBUSB_ENDPOINT_OUT) | static_cast<uint8_t>(LIBUSB_REQUEST_TYPE_VENDOR) |
                                           static_cast<uint8_t>(LIBUSB_RECIPIENT_INTERFACE),
                                       GS_USB_BREQ_MODE,
                                       0,
                                       intf_num,
                                       reinterpret_cast<uint8_t*>(&m),
                                       sizeof(m),
                                       1000) >= 0;
    }

    void start_rx_thread() {
        rx_running.store(true, std::memory_order_release);
        rx_thread = std::thread([this]() {
            uint8_t thread_rx_buf[2048]{};
            while (rx_running.load(std::memory_order_acquire)) {
                int transferred = 0;
                const int r = libusb_bulk_transfer(dev,
                                                   bulk_in_ep,
                                                   thread_rx_buf,
                                                   sizeof(GsUsbFrame),
                                                   &transferred,
                                                   100);
                if (r == 0 && transferred >= 20) {
                    const auto* in = reinterpret_cast<const GsUsbFrame*>(thread_rx_buf);
                    if (in->echo_id == GS_USB_ECHO_ID_UNUSED && in->can_dlc <= 8 && !(in->can_id & 0x80000000u)) {
                        CanFrame frame{};
                        frame.id = in->can_id;
                        frame.dlc = in->can_dlc;
                        frame.is_extended = false;
                        frame.is_rtr = false;
                        std::memcpy(frame.data, in->data, 8);

                        std::lock_guard<std::mutex> lock(rx_mutex);
                        if (rx_queue.size() < 10000) {
                            rx_queue.push_back(frame);
                        }
                    }
                }
            }
        });
    }

    void stop_rx_thread() {
        if (rx_running.exchange(false, std::memory_order_acq_rel)) {
            if (rx_thread.joinable()) {
                rx_thread.join();
            }
        }
        std::lock_guard<std::mutex> lock(rx_mutex);
        rx_queue.clear();
    }
};

GsUsbCanPort::GsUsbCanPort() : impl_(std::make_unique<Impl>()) {}
GsUsbCanPort::~GsUsbCanPort() = default;

const std::string& GsUsbCanPort::lastError() const { return impl_->last_error; }

bool GsUsbCanPort::isOpen() const { return impl_->dev != nullptr; }

void GsUsbCanPort::close() {
    if (impl_->dev) {
        impl_->stop_rx_thread();
        impl_->set_mode(GS_USB_MODE_RESET, 0);
        libusb_release_interface(impl_->dev, impl_->intf_num);
        libusb_close(impl_->dev);
        impl_->dev = nullptr;
    }
}

bool GsUsbCanPort::probeAccess(const char* device_path) {
    if (isOpen()) {
        close();
    }

    impl_->last_error.clear();
    if (!impl_->has_context()) {
        if (impl_->last_error.empty()) {
            impl_->set_error("libusb context is not available");
        }
        return false;
    }

    libusb_device** devs = nullptr;
    const ssize_t cnt = libusb_get_device_list(impl_->ctx, &devs);
    if (cnt < 0) {
        impl_->set_error(make_libusb_error_message("libusb_get_device_list", static_cast<int>(cnt)));
        return false;
    }

    const std::string target_path = device_path ? device_path : "";
    libusb_device* target = nullptr;
    for (ssize_t i = 0; i < cnt; ++i) {
        libusb_device_descriptor desc{};
        if (libusb_get_device_descriptor(devs[i], &desc) != 0) {
            continue;
        }
        if (desc.idVendor != 0x1d50 || desc.idProduct != 0x606f) {
            continue;
        }

        if (target_path.empty()) {
            target = devs[i];
            break;
        }

        const uint8_t bus = libusb_get_bus_number(devs[i]);
        const uint8_t addr = libusb_get_device_address(devs[i]);
        std::stringstream ss;
        ss << "usb:" << static_cast<int>(bus) << ":" << static_cast<int>(addr);
        if (ss.str() == target_path) {
            target = devs[i];
            break;
        }
    }

    if (!target) {
        libusb_free_device_list(devs, 1);
        impl_->set_error("GS-USB target device was not found");
        return false;
    }

    libusb_device_handle* handle = nullptr;
    const int open_rc = libusb_open(target, &handle);
    libusb_free_device_list(devs, 1);
    if (open_rc != 0) {
        impl_->set_error(make_libusb_error_message("libusb_open", open_rc));
        return false;
    }

    libusb_close(handle);
    return true;
}

bool GsUsbCanPort::open(const char* device_path, unsigned int baud_rate) {
    if (isOpen()) {
        close();
    }

    impl_->last_error.clear();
    if (!impl_->has_context()) {
        if (impl_->last_error.empty()) {
            impl_->set_error("libusb context is not available");
        }
        return false;
    }

    libusb_device** devs = nullptr;
    const ssize_t cnt = libusb_get_device_list(impl_->ctx, &devs);
    if (cnt < 0) {
        impl_->set_error(make_libusb_error_message("libusb_get_device_list", static_cast<int>(cnt)));
        return false;
    }

    const std::string target_path = device_path ? device_path : "";
    libusb_device* target = nullptr;

    for (ssize_t i = 0; i < cnt; ++i) {
        libusb_device_descriptor desc{};
        if (libusb_get_device_descriptor(devs[i], &desc) != 0) {
            continue;
        }
        if (desc.idVendor == 0x1d50 && desc.idProduct == 0x606f) {
            if (target_path.empty()) {
                target = devs[i];
                break;
            }

            const uint8_t bus = libusb_get_bus_number(devs[i]);
            const uint8_t addr = libusb_get_device_address(devs[i]);
            std::stringstream ss;
            ss << "usb:" << static_cast<int>(bus) << ":" << static_cast<int>(addr);
            if (ss.str() == target_path) {
                target = devs[i];
                break;
            }
        }
    }

    if (!target) {
        libusb_free_device_list(devs, 1);
        impl_->set_error("GS-USB target device was not found");
        return false;
    }

    UsbInterfaceEndpoints endpoints{};
    if (!Impl::discover_endpoints(target, &endpoints)) {
        libusb_free_device_list(devs, 1);
        impl_->set_error("failed to discover GS-USB bulk endpoints from USB descriptors");
        return false;
    }

    const int open_rc = libusb_open(target, &impl_->dev);
    if (open_rc != 0) {
        libusb_free_device_list(devs, 1);
        impl_->set_error(make_libusb_error_message("libusb_open", open_rc));
        return false;
    }
    libusb_free_device_list(devs, 1);

    impl_->intf_num = endpoints.interface_number;
    impl_->bulk_in_ep = endpoints.bulk_in_endpoint;
    impl_->bulk_out_ep = endpoints.bulk_out_endpoint;

#ifndef _WIN32
    if (libusb_kernel_driver_active(impl_->dev, impl_->intf_num) == 1) {
        const int detach_rc = libusb_detach_kernel_driver(impl_->dev, impl_->intf_num);
        if (detach_rc != 0) {
            impl_->set_error(make_libusb_error_message("libusb_detach_kernel_driver", detach_rc));
            close();
            return false;
        }
    }
#endif
    const int claim_rc = libusb_claim_interface(impl_->dev, impl_->intf_num);
    if (claim_rc < 0) {
        impl_->set_error(make_libusb_error_message("libusb_claim_interface", claim_rc));
        close();
        return false;
    }

    uint32_t host_format = 0x0000BEEF;
    const int host_format_rc = libusb_control_transfer(impl_->dev,
                                                       static_cast<uint8_t>(LIBUSB_ENDPOINT_OUT) | static_cast<uint8_t>(LIBUSB_REQUEST_TYPE_VENDOR) |
                                                           static_cast<uint8_t>(LIBUSB_RECIPIENT_INTERFACE),
                                                       GS_USB_BREQ_HOST_FORMAT,
                                                       1,
                                                       impl_->intf_num,
                                                       reinterpret_cast<uint8_t*>(&host_format),
                                                       sizeof(host_format),
                                                       1000);
    if (host_format_rc < 0) {
        impl_->set_error(make_libusb_error_message("GS_USB_BREQ_HOST_FORMAT", host_format_rc));
        close();
        return false;
    }

    uint8_t bt_const[40]{};
    const int bt_const_rc = libusb_control_transfer(impl_->dev,
                                                    static_cast<uint8_t>(LIBUSB_ENDPOINT_IN) | static_cast<uint8_t>(LIBUSB_REQUEST_TYPE_VENDOR) |
                                                        static_cast<uint8_t>(LIBUSB_RECIPIENT_INTERFACE),
                                                    GS_USB_BREQ_BT_CONST,
                                                    0,
                                                    impl_->intf_num,
                                                    bt_const,
                                                    sizeof(bt_const),
                                                    1000);
    if (bt_const_rc < 0) {
        impl_->set_error(make_libusb_error_message("GS_USB_BREQ_BT_CONST", bt_const_rc));
        close();
        return false;
    }

    const uint32_t fclk_can = Impl::read_u32(bt_const + 4);
    std::array<uint8_t, 20> bit_timing{};

    uint32_t prop = 1;
    uint32_t ph1 = 12;
    uint32_t ph2 = 2;
    uint32_t sjw = 1;
    uint32_t brp = 3;

    // Start from known-good 1Mbps settings per clock, then scale BRP.
    uint32_t base_1m_brp = 3;
    if (fclk_can == 80000000u) {
        base_1m_brp = 5;
    } else if (fclk_can == 170000000u) {
        // 1 + PROP + PH1 + PH2 = 17 tq
        prop = 6;
        ph1 = 8;
        ph2 = 2;
        base_1m_brp = 10;
    } else if (fclk_can == 48000000u) {
        base_1m_brp = 3;
    }

    uint32_t scale = 0;
    switch (baud_rate) {
        case 1000000:
            scale = 1;
            break;
        case 500000:
            scale = 2;
            break;
        case 250000:
            scale = 4;
            break;
        case 125000:
            scale = 8;
            break;
        default:
            impl_->set_error("unsupported CAN baud rate for GS-USB open");
            close();
            return false;
    }

    brp = base_1m_brp * scale;

    Impl::write_u32(bit_timing.data() + 0, prop);
    Impl::write_u32(bit_timing.data() + 4, ph1);
    Impl::write_u32(bit_timing.data() + 8, ph2);
    Impl::write_u32(bit_timing.data() + 12, sjw);
    Impl::write_u32(bit_timing.data() + 16, brp);

    const int bit_timing_rc = libusb_control_transfer(impl_->dev,
                                                      static_cast<uint8_t>(LIBUSB_ENDPOINT_OUT) | static_cast<uint8_t>(LIBUSB_REQUEST_TYPE_VENDOR) |
                                                          static_cast<uint8_t>(LIBUSB_RECIPIENT_INTERFACE),
                                                      GS_USB_BREQ_BITTIMING,
                                                      0,
                                                      impl_->intf_num,
                                                      bit_timing.data(),
                                                      static_cast<uint16_t>(bit_timing.size()),
                                                      1000);
    if (bit_timing_rc < 0) {
        impl_->set_error(make_libusb_error_message("GS_USB_BREQ_BITTIMING", bit_timing_rc));
        close();
        return false;
    }

    if (!impl_->set_mode(GS_USB_MODE_START, 0)) {
        impl_->set_error("GS_USB_BREQ_MODE start failed");
        close();
        return false;
    }

    impl_->start_rx_thread();

    return true;
}

bool GsUsbCanPort::write(const CanFrame& frame) {
    if (!isOpen()) {
        return false;
    }

    GsUsbFrame out{};
    // Use unused echo id marker; TX echoes are filtered in read().
    out.echo_id = GS_USB_ECHO_ID_UNUSED;
    out.can_id = frame.id;
    out.can_dlc = frame.dlc;
    std::memcpy(out.data, frame.data, frame.dlc);

    int transferred = 0;
    const int r = libusb_bulk_transfer(impl_->dev,
                                       impl_->bulk_out_ep,
                                       reinterpret_cast<uint8_t*>(&out),
                                       sizeof(out),
                                       &transferred,
                                       100);
    if (r != 0) {
        impl_->set_error(make_libusb_error_message("libusb_bulk_transfer write", r));
    }
    return (r == 0 && transferred == sizeof(out));
}

bool GsUsbCanPort::read(CanFrame& frame, unsigned int timeout_ms) {
    if (!isOpen()) {
        return false;
    }

    const auto started = std::chrono::steady_clock::now();
    const auto total_wait = std::chrono::milliseconds(timeout_ms == 0U ? 1U : timeout_ms);

    while (true) {
        {
            std::lock_guard<std::mutex> lock(impl_->rx_mutex);
            if (!impl_->rx_queue.empty()) {
                frame = impl_->rx_queue.front();
                impl_->rx_queue.pop_front();
                return true;
            }
        }

        if (timeout_ms == 0U) {
            return false;
        }

        const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - started);
        if (elapsed >= total_wait) {
            return false;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}

std::vector<GsUsbDeviceInfo> GsUsbCanPort::enumerateDevices() {
    std::vector<GsUsbDeviceInfo> out;

    libusb_context* ctx = nullptr;
    if (libusb_init(&ctx) != 0) {
        return out;
    }

    libusb_device** devs = nullptr;
    const ssize_t cnt = libusb_get_device_list(ctx, &devs);
    if (cnt < 0) {
        libusb_exit(ctx);
        return out;
    }

    for (ssize_t i = 0; i < cnt; ++i) {
        libusb_device_descriptor desc{};
        if (libusb_get_device_descriptor(devs[i], &desc) != 0) {
            continue;
        }

        if (desc.idVendor == 0x1d50 && desc.idProduct == 0x606f) {
            GsUsbDeviceInfo info{};
            const uint8_t bus = libusb_get_bus_number(devs[i]);
            const uint8_t addr = libusb_get_device_address(devs[i]);

            std::stringstream path;
            path << "usb:" << static_cast<int>(bus) << ":" << static_cast<int>(addr);
            info.path = path.str();
            info.description = "GS-USB CAN Device";

            libusb_device_handle* h = nullptr;
            if (libusb_open(devs[i], &h) == 0) {
                unsigned char product[256]{};
                if (libusb_get_string_descriptor_ascii(h, desc.iProduct, product, sizeof(product)) > 0) {
                    info.description = reinterpret_cast<const char*>(product);
                }
                libusb_close(h);
            }

            out.push_back(info);
        }
    }

    libusb_free_device_list(devs, 1);
    libusb_exit(ctx);
    return out;
}

} // namespace mks
