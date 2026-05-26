#pragma once

#include <cstdint>

namespace ethercat_driver {

struct EthercatManualHomingInputs {
    bool enabled{false};
    bool faulted{false};
    std::uint32_t digital_inputs{0U};
    double position_deg{0.0};
    double cycle_dt_sec{0.004};
    double home_offset_deg{0.0};
    double search_velocity_deg_per_sec{100.0};
};

struct EthercatManualHomingOutputs {
    bool active{false};
    bool force_csp_mode{false};
    bool force_pp_mode{false};
    bool target_position_override{false};
    double target_position_deg{0.0};
    bool request_pp_setpoint{false};
    bool apply_set_zero{false};
};

class EthercatManualHomingStateMachine final {
public:
    static constexpr std::uint32_t kDi3Mask = (1U << 2U);
    static constexpr double kDefaultSearchVelocityDegPerSec = 100.0;

    void request_start() noexcept;
    void request_cancel() noexcept;
    void reset() noexcept;

    [[nodiscard]] bool active() const noexcept;
    [[nodiscard]] EthercatManualHomingOutputs step(const EthercatManualHomingInputs& in) noexcept;

private:
    enum class Phase : std::uint8_t {
        Idle = 0,
        SearchForSwitch,
        SwitchDetectedPause,
        MoveToOffset
    };

    Phase phase_{Phase::Idle};
    bool start_requested_{false};
    bool cancel_requested_{false};
    double search_target_deg_{0.0};
    double switch_position_deg_{0.0};
    double offset_target_deg_{0.0};
    double pause_remaining_sec_{0.0};
    bool offset_pp_setpoint_issued_{false};
};

} // namespace ethercat_driver
