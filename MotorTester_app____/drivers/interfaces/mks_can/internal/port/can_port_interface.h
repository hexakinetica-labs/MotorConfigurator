#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace mks {

struct CanFrame {
    uint32_t id{0};
    uint8_t dlc{0};
    uint8_t data[8]{0};
    bool is_extended{false};
    bool is_rtr{false};
};

class ICanPort {
public:
    virtual ~ICanPort() = default;

    virtual bool open(const char* channel, unsigned int baud_rate) = 0;
    virtual void close() = 0;
    virtual bool isOpen() const = 0;
    virtual bool write(const CanFrame& frame) = 0;
    virtual bool read(CanFrame& frame, unsigned int timeout_ms) = 0;
};

struct GsUsbDeviceInfo {
    std::string path;
    std::string description;
};

} // namespace mks
