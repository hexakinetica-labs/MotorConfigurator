#pragma once

#include <chrono>
#include <cstdint>

namespace mks {

struct MksHomingInputs {
    bool faulted{false};
    bool home_status_active{false};
    double position_deg{0.0};
    double velocity_deg_per_sec{0.0};
    double homing_offset_deg{0.0};
};

struct MksHomingOutputs {
    bool active{false};
    bool request_start_hardware_home{false};
    bool request_move_to_offset{false};
    double move_to_offset_deg{0.0};
    bool request_set_zero{false};
    bool failed{false};
};

class MksHomingStateMachine final {
public:
    enum class Phase : std::uint8_t {
        Idle = 0,
        StartHardwareHome,
        WaitHomingStatusActive,
        WaitHomingStatusInactiveAtZero,
        CommandMoveToOffset,
        WaitMoveToOffset
    };

    void request_start() noexcept;
    void request_cancel() noexcept;
    void reset() noexcept;

    [[nodiscard]] bool active() const noexcept;
    [[nodiscard]] Phase current_phase() const noexcept;
    [[nodiscard]] MksHomingOutputs step(const MksHomingInputs& in,
                                        std::chrono::steady_clock::time_point now) noexcept;

private:
    Phase phase_{Phase::Idle};
    bool start_requested_{false};
    bool cancel_requested_{false};
    bool seen_homing_status_active_{false};
    std::chrono::steady_clock::time_point phase_deadline_{};
};

} // namespace mks