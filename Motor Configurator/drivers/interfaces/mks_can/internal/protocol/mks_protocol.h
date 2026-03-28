 #pragma once

#include "mks_can/internal/port/can_port_interface.h"

#include <cstdint>
#include <string>
#include <vector>

namespace mks {

enum class MksCommand : uint8_t {
    ReadEncoderCarry = 0x30,
    ReadEncoderAddition = 0x31,
    ReadMotorSpeed = 0x32,
    ReadNumPulsesReceived = 0x33,
    ReadIOStatus = 0x34,
    ReadRawEncoderAddition = 0x35,
    WriteIOPort = 0x36,
    ReadShaftAngleError = 0x39,
    ReadEnPinStatus = 0x3A,
    ReadGoBackToZeroStatus = 0x3B,
    ReleaseStallProtection = 0x3D,
    ReadProtectionState = 0x3E,

    CalibrateEncoder = 0x80,
    SetWorkMode = 0x82,
    SetWorkingCurrent = 0x83,
    SetSubdivision = 0x84,
    SetEnPinActiveLevel = 0x85,
    SetMotorDirection = 0x86,
    SetAutoTurnOffScreen = 0x87,
    SetLockedRotorProtection = 0x88,
    SetSubdivisionInterpolation = 0x89,
    SetCanBitrate = 0x8A,
    SetCanId = 0x8B,
    SetSlaveRespondActive = 0x8C,
    SetGroupId = 0x8D,
    SetKeyLock = 0x8F,
    SetMode0 = 0x9A,
    SetHoldingCurrent = 0x9B,
    SetNoLimitHomeParams = 0x94,
    SetEnTriggerErrorProtection = 0x9D,
    SetLimitPortRemap = 0x9E,
    RestoreDefaultParameters = 0x3F,
    RestartMotor = 0x41,
    SetMultiMotorSyncFlag = 0x4A,
    ExecuteMultiMotorSync = 0x4B,

    SetHomeParameters = 0x90,
    GoHome = 0x91,
    SetCurrentAxisToZero = 0x92,

    QueryMotorStatus = 0xF1,
    EnableMotor = 0xF3,
    RunSpeedMode = 0xF6,
    EmergencyStop = 0xF7,
    RunPositionRelativePulses = 0xFD,
    RunPositionAbsolutePulses = 0xFE,
    RunPositionRelativeAxis = 0xF4,
    RunPositionAbsoluteAxis = 0xF5,
    SaveCleanSpeedMode = 0xFF,
};

class MksProtocol {
public:
    struct LastExchangeTrace {
        std::uint16_t can_id{0};
        std::uint8_t request_cmd{0};
        std::uint8_t expected_response_cmd{0};
        bool expect_response{true};
        std::vector<std::uint8_t> tx_frame_data{}; // [cmd payload... crc]
        std::vector<std::uint8_t> rx_frame_data{}; // [cmd payload... crc]
    };

    struct IoStats {
        uint64_t tx_ok{0};
        uint64_t tx_fail{0};
        uint64_t rx_ok{0};
        uint64_t rx_timeout{0};
        uint64_t rx_crc_error{0};
        uint64_t rx_filtered{0};
        uint64_t tx_bits{0};
        uint64_t rx_bits{0};
    };

    explicit MksProtocol(ICanPort& port, unsigned int default_timeout_ms = 200);

    bool sendCommand(uint16_t can_id,
                     MksCommand cmd,
                     const std::vector<uint8_t>& payload,
                     std::vector<uint8_t>& response_payload,
                     uint8_t expected_response_cmd = 0xFF,
                     unsigned int timeout_ms = 0,
                     bool expect_response = true);

    bool readParameter(uint16_t can_id,
                       uint8_t parameter_cmd,
                       std::vector<uint8_t>& response_payload,
                       unsigned int timeout_ms = 0);

    std::vector<uint16_t> scanBus(uint16_t min_id = 1, uint16_t max_id = 32, unsigned int timeout_ms = 30);

    static uint8_t computeCrc(uint16_t can_id, const std::vector<uint8_t>& bytes_without_crc);

    static void appendBe16(std::vector<uint8_t>& out, uint16_t value);
    static void appendBe24(std::vector<uint8_t>& out, int32_t value);
    static void appendBe24u(std::vector<uint8_t>& out, uint32_t value);
    static void appendBe32(std::vector<uint8_t>& out, int32_t value);

    static uint16_t readBe16(const uint8_t* p);
    static int16_t readBe16s(const uint8_t* p);
    static int32_t readBe24s(const uint8_t* p);
    static uint32_t readBe24u(const uint8_t* p);
    static int32_t readBe32s(const uint8_t* p);
    static int64_t readBe48s(const uint8_t* p);

    const std::string& lastError() const { return last_error_; }
    const LastExchangeTrace& lastExchangeTrace() const { return last_exchange_trace_; }
    [[nodiscard]] std::string lastExchangeTraceSummary() const;
    const IoStats& ioStats() const { return io_stats_; }
    void resetIoStats() { io_stats_ = IoStats{}; }

private:
    ICanPort& port_;
    unsigned int default_timeout_ms_;
    std::string last_error_;
    LastExchangeTrace last_exchange_trace_{};
    IoStats io_stats_{};
};

} // namespace mks
