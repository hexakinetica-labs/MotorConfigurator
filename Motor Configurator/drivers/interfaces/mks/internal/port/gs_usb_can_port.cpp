#include "mks/internal/port/gs_usb_can_port.h"

#include <libusb.h>

#include <array>
#include <cstring>
#include <sstream>

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
} // namespace

struct GsUsbCanPort::Impl {
    libusb_context* ctx{nullptr};
    libusb_device_handle* dev{nullptr};
    uint8_t intf_num{0};
    uint8_t rx_buf[2048]{};

    Impl() { libusb_init(&ctx); }
    ~Impl() {
        if (dev) {
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
};

GsUsbCanPort::GsUsbCanPort() : impl_(std::make_unique<Impl>()) {}
GsUsbCanPort::~GsUsbCanPort() = default;

bool GsUsbCanPort::isOpen() const { return impl_->dev != nullptr; }

void GsUsbCanPort::close() {
    if (impl_->dev) {
        impl_->set_mode(GS_USB_MODE_RESET, 0);
        libusb_release_interface(impl_->dev, impl_->intf_num);
        libusb_close(impl_->dev);
        impl_->dev = nullptr;
    }
}

bool GsUsbCanPort::open(const char* device_path, unsigned int baud_rate) {
    if (isOpen()) {
        close();
    }

    libusb_device** devs = nullptr;
    const ssize_t cnt = libusb_get_device_list(impl_->ctx, &devs);
    if (cnt < 0) {
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
        return false;
    }

    if (libusb_open(target, &impl_->dev) != 0) {
        libusb_free_device_list(devs, 1);
        return false;
    }
    libusb_free_device_list(devs, 1);

    if (libusb_kernel_driver_active(impl_->dev, 0) == 1) {
        libusb_detach_kernel_driver(impl_->dev, 0);
    }
    if (libusb_claim_interface(impl_->dev, 0) < 0) {
        close();
        return false;
    }

    uint32_t host_format = 0x0000BEEF;
    libusb_control_transfer(impl_->dev,
                            static_cast<uint8_t>(LIBUSB_ENDPOINT_OUT) | static_cast<uint8_t>(LIBUSB_REQUEST_TYPE_VENDOR) |
                                static_cast<uint8_t>(LIBUSB_RECIPIENT_INTERFACE),
                            GS_USB_BREQ_HOST_FORMAT,
                            1,
                            impl_->intf_num,
                            reinterpret_cast<uint8_t*>(&host_format),
                            sizeof(host_format),
                            1000);

    uint8_t bt_const[40]{};
    if (libusb_control_transfer(impl_->dev,
                                static_cast<uint8_t>(LIBUSB_ENDPOINT_IN) | static_cast<uint8_t>(LIBUSB_REQUEST_TYPE_VENDOR) |
                                    static_cast<uint8_t>(LIBUSB_RECIPIENT_INTERFACE),
                                GS_USB_BREQ_BT_CONST,
                                0,
                                impl_->intf_num,
                                bt_const,
                                sizeof(bt_const),
                                1000) < 0) {
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
            close();
            return false;
    }

    brp = base_1m_brp * scale;

    Impl::write_u32(bit_timing.data() + 0, prop);
    Impl::write_u32(bit_timing.data() + 4, ph1);
    Impl::write_u32(bit_timing.data() + 8, ph2);
    Impl::write_u32(bit_timing.data() + 12, sjw);
    Impl::write_u32(bit_timing.data() + 16, brp);

    if (libusb_control_transfer(impl_->dev,
                                static_cast<uint8_t>(LIBUSB_ENDPOINT_OUT) | static_cast<uint8_t>(LIBUSB_REQUEST_TYPE_VENDOR) |
                                    static_cast<uint8_t>(LIBUSB_RECIPIENT_INTERFACE),
                                GS_USB_BREQ_BITTIMING,
                                0,
                                impl_->intf_num,
                                bit_timing.data(),
                                static_cast<uint16_t>(bit_timing.size()),
                                1000) < 0) {
        close();
        return false;
    }

    if (!impl_->set_mode(GS_USB_MODE_START, 0)) {
        close();
        return false;
    }

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
                                       static_cast<unsigned char>(0x02 | static_cast<uint8_t>(LIBUSB_ENDPOINT_OUT)),
                                       reinterpret_cast<uint8_t*>(&out),
                                       sizeof(out),
                                       &transferred,
                                       100);
    return (r == 0 && transferred == sizeof(out));
}

bool GsUsbCanPort::read(CanFrame& frame, unsigned int timeout_ms) {
    if (!isOpen()) {
        return false;
    }

    int transferred = 0;
    const int r = libusb_bulk_transfer(impl_->dev,
                                       static_cast<unsigned char>(0x81 | static_cast<uint8_t>(LIBUSB_ENDPOINT_IN)),
                                       impl_->rx_buf,
                                       sizeof(GsUsbFrame),
                                       &transferred,
                                       static_cast<unsigned int>(timeout_ms));
    if (r != 0 || transferred < 20) { // candleLight FW can return 20 or 24 bytes
        return false;
    }

    auto* in = reinterpret_cast<GsUsbFrame*>(impl_->rx_buf);

    // Ignore TX echo confirmations (not real bus responses from motor).
    if (in->echo_id != GS_USB_ECHO_ID_UNUSED) {
        return false;
    }

    if (in->can_dlc > 8 || (in->can_id & 0x80000000u)) {
        return false;
    }

    frame.id = in->can_id;
    frame.dlc = in->can_dlc;
    frame.is_extended = false;
    frame.is_rtr = false;
    std::memcpy(frame.data, in->data, 8);
    return true;
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
