#pragma once

#include "mks_can/internal/port/can_port_interface.h"

#include <memory>
#include <vector>

namespace mks {

class GsUsbCanPort : public ICanPort {
public:
    GsUsbCanPort();
    ~GsUsbCanPort() override;

    bool open(const char* device_path, unsigned int baud_rate) override;
    void close() override;
    bool isOpen() const override;
    bool write(const CanFrame& frame) override;
    bool read(CanFrame& frame, unsigned int timeout_ms) override;

    static std::vector<GsUsbDeviceInfo> enumerateDevices();

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace mks
