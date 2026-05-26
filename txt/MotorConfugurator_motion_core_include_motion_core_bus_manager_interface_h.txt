#pragma once
#include "motion_core/result.h"

namespace motion_core {

struct BusStatistics {
    double cycle_rate_hz{0.0};
    double bus_load_percent{0.0};
};

class IBusManager {
public:
    virtual ~IBusManager() = default;

    [[nodiscard]] virtual std::string get_name() const = 0;
    
    [[nodiscard]] virtual Result<void> start() = 0;
    virtual Result<void> stop() = 0;
    [[nodiscard]] virtual Result<BusStatistics> get_statistics() const { return Result<BusStatistics>::success({}); }
};

} // namespace motion_core
