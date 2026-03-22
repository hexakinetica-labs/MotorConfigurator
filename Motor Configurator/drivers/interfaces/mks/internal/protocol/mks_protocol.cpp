#include "mks/internal/protocol/mks_protocol.h"

#include <algorithm>
#include <chrono>

namespace mks {

MksProtocol::MksProtocol(ICanPort& port, unsigned int default_timeout_ms)
    : port_(port), default_timeout_ms_(default_timeout_ms) {}

uint8_t MksProtocol::computeCrc(uint16_t can_id, const std::vector<uint8_t>& bytes_without_crc) {
    uint32_t sum = can_id & 0x7FF;
    for (uint8_t b : bytes_without_crc) {
        sum += b;
    }
    return static_cast<uint8_t>(sum & 0xFF);
}

void MksProtocol::appendBe16(std::vector<uint8_t>& out, uint16_t value) {
    out.push_back(static_cast<uint8_t>((value >> 8) & 0xFF));
    out.push_back(static_cast<uint8_t>(value & 0xFF));
}

void MksProtocol::appendBe24(std::vector<uint8_t>& out, int32_t value) {
    const uint32_t v = static_cast<uint32_t>(value) & 0x00FFFFFFu;
    out.push_back(static_cast<uint8_t>((v >> 16) & 0xFF));
    out.push_back(static_cast<uint8_t>((v >> 8) & 0xFF));
    out.push_back(static_cast<uint8_t>(v & 0xFF));
}

void MksProtocol::appendBe24u(std::vector<uint8_t>& out, uint32_t value) {
    const uint32_t v = value & 0x00FFFFFFu;
    out.push_back(static_cast<uint8_t>((v >> 16) & 0xFF));
    out.push_back(static_cast<uint8_t>((v >> 8) & 0xFF));
    out.push_back(static_cast<uint8_t>(v & 0xFF));
}

void MksProtocol::appendBe32(std::vector<uint8_t>& out, int32_t value) {
    const uint32_t v = static_cast<uint32_t>(value);
    out.push_back(static_cast<uint8_t>((v >> 24) & 0xFF));
    out.push_back(static_cast<uint8_t>((v >> 16) & 0xFF));
    out.push_back(static_cast<uint8_t>((v >> 8) & 0xFF));
    out.push_back(static_cast<uint8_t>(v & 0xFF));
}

uint16_t MksProtocol::readBe16(const uint8_t* p) {
    return static_cast<uint16_t>((uint16_t(p[0]) << 8) | uint16_t(p[1]));
}

int16_t MksProtocol::readBe16s(const uint8_t* p) {
    return static_cast<int16_t>(readBe16(p));
}

uint32_t MksProtocol::readBe24u(const uint8_t* p) {
    return (uint32_t(p[0]) << 16) | (uint32_t(p[1]) << 8) | uint32_t(p[2]);
}

int32_t MksProtocol::readBe24s(const uint8_t* p) {
    uint32_t v = readBe24u(p);
    if (v & 0x00800000u) {
        v |= 0xFF000000u;
    } else {
        v &= 0x00FFFFFFu;
    }
    return static_cast<int32_t>(v);
}

int32_t MksProtocol::readBe32s(const uint8_t* p) {
    const uint32_t v = (uint32_t(p[0]) << 24) | (uint32_t(p[1]) << 16) | (uint32_t(p[2]) << 8) |
                       uint32_t(p[3]);
    return static_cast<int32_t>(v);
}

int64_t MksProtocol::readBe48s(const uint8_t* p) {
    uint64_t v = (uint64_t(p[0]) << 40) | (uint64_t(p[1]) << 32) | (uint64_t(p[2]) << 24) |
                 (uint64_t(p[3]) << 16) | (uint64_t(p[4]) << 8) | uint64_t(p[5]);
    if (v & (uint64_t(1) << 47)) {
        v |= 0xFFFF000000000000ull;
    }
    return static_cast<int64_t>(v);
}

bool MksProtocol::sendCommand(uint16_t can_id,
                              MksCommand cmd,
                              const std::vector<uint8_t>& payload,
                              std::vector<uint8_t>& response_payload,
                              uint8_t expected_response_cmd,
                              unsigned int timeout_ms,
                              bool expect_response) {
    last_error_.clear();
    response_payload.clear();

    if (!port_.isOpen()) {
        last_error_ = "CAN port is not open";
        return false;
    }

    if (can_id > 0x7FF) {
        last_error_ = "Invalid CAN ID (>0x7FF)";
        return false;
    }

    std::vector<uint8_t> bytes;
    bytes.reserve(1 + payload.size() + 1);
    bytes.push_back(static_cast<uint8_t>(cmd));
    bytes.insert(bytes.end(), payload.begin(), payload.end());
    bytes.push_back(computeCrc(can_id, bytes));

    if (bytes.size() > 8) {
        last_error_ = "Payload too large for single CAN frame";
        return false;
    }

    CanFrame tx{};
    tx.id = can_id;
    tx.dlc = static_cast<uint8_t>(bytes.size());
    std::copy(bytes.begin(), bytes.end(), tx.data);

    if (!port_.write(tx)) {
        ++io_stats_.tx_fail;
        last_error_ = "CAN write failed";
        return false;
    }
    ++io_stats_.tx_ok;
    io_stats_.tx_bits += (47 + 8 * tx.dlc); // Approx bits for standard CAN frame

    if (!expect_response || can_id == 0x000) {
        return true;
    }

    const unsigned int wait_ms = timeout_ms == 0 ? default_timeout_ms_ : timeout_ms;
    const auto started = std::chrono::steady_clock::now();
    const uint8_t wanted_cmd = (expected_response_cmd == 0xFF) ? static_cast<uint8_t>(cmd) : expected_response_cmd;

    while (true) {
        const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                                 std::chrono::steady_clock::now() - started)
                                 .count();
        if (elapsed >= wait_ms) {
            ++io_stats_.rx_timeout;
            last_error_ = "Timeout waiting MKS response";
            return false;
        }

        CanFrame rx{};
        const unsigned int left = wait_ms - static_cast<unsigned int>(elapsed);
        const unsigned int step_timeout = std::min<unsigned int>(left, 20u);
        if (!port_.read(rx, step_timeout)) {
            continue;
        }

        io_stats_.rx_bits += (47 + 8 * rx.dlc);

        // MKS replies always contain at least: [cmd][status/data...][crc],
        // i.e. DLC >= 3. DLC=2 frames are typically TX echoes and must be ignored.
        if (rx.id != can_id || rx.dlc < 3 || rx.dlc > 8) {
            ++io_stats_.rx_filtered;
            continue;
        }

        const uint8_t rx_cmd = rx.data[0];
        if (rx_cmd != wanted_cmd) {
            ++io_stats_.rx_filtered;
            continue;
        }

        std::vector<uint8_t> rx_wo_crc;
        for (uint8_t i = 0; i < rx.dlc - 1; ++i) {
            rx_wo_crc.push_back(rx.data[i]);
        }

        const uint8_t expected_crc = computeCrc(can_id, rx_wo_crc);
        const uint8_t actual_crc = rx.data[rx.dlc - 1];
        if (expected_crc != actual_crc) {
            ++io_stats_.rx_crc_error;
            last_error_ = "CRC mismatch in MKS response";
            return false;
        }

        response_payload.assign(rx.data + 1, rx.data + (rx.dlc - 1));
        ++io_stats_.rx_ok;
        return true;
    }
}

bool MksProtocol::readParameter(uint16_t can_id,
                                uint8_t parameter_cmd,
                                std::vector<uint8_t>& response_payload,
                                unsigned int timeout_ms) {
    std::vector<uint8_t> req{parameter_cmd};
    return sendCommand(can_id,
                       static_cast<MksCommand>(0x00),
                       req,
                       response_payload,
                       parameter_cmd,
                       timeout_ms,
                       true);
}

std::vector<uint16_t> MksProtocol::scanBus(uint16_t min_id, uint16_t max_id, unsigned int timeout_ms) {
    std::vector<uint16_t> found;
    if (min_id == 0) {
        min_id = 1;
    }
    if (max_id > 0x7FF) {
        max_id = 0x7FF;
    }
    if (min_id > max_id) {
        return found;
    }

    for (uint16_t id = min_id; id <= max_id; ++id) {
        std::vector<uint8_t> resp;
        if (sendCommand(id,
                        MksCommand::QueryMotorStatus,
                        {},
                        resp,
                        0xFF,
                        timeout_ms,
                        true)) {
            found.push_back(id);
        }
    }
    return found;
}

} // namespace mks
