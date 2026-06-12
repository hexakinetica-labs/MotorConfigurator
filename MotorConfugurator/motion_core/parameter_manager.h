#pragma once

#include "motion_core/hal_runtime.h"
#include "motion_core/parameter_types.h"
#include <functional>
#include <mutex>
#include <thread>
#include <vector>
#include <queue>
#include <atomic>
#include <condition_variable>
#include <unordered_set>

namespace motion_core {

class ParameterManager {
public:
    using ListCallback = std::function<void(Result<std::vector<ParameterDescriptor>>)>;
    using ReadCallback = std::function<void(Result<ParameterSet>)>;
    using WriteCallback = std::function<void(Result<void>)>;

    explicit ParameterManager(HalRuntime& runtime);
    ~ParameterManager();

    [[nodiscard]] Result<void> request_list_parameters(uint16_t axis_id, ListCallback cb);
    [[nodiscard]] Result<void> request_read_parameters(uint16_t axis_id, ReadCallback cb);
    void request_apply_patch(uint16_t axis_id, ParameterPatch patch, WriteCallback cb);

private:
    void worker_loop();

    HalRuntime& runtime_;
    
    struct Task {
        std::function<void()> func;
    };

    std::queue<Task> task_queue_;
    std::mutex queue_mutex_;
    std::condition_variable cv_;
    std::atomic<bool> running_{true};
    std::thread worker_thread_;

    std::unordered_set<uint16_t> active_list_requests_;
    std::unordered_set<uint16_t> active_read_requests_;
    std::mutex state_mutex_;
};

} // namespace motion_core
