#include "motion_core/parameter_manager.h"

namespace motion_core {

ParameterManager::ParameterManager(HalRuntime& runtime) : runtime_(runtime) {
    worker_thread_ = std::thread(&ParameterManager::worker_loop, this);
}

ParameterManager::~ParameterManager() {
    {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        running_ = false;
    }
    cv_.notify_all();
    if (worker_thread_.joinable()) {
        worker_thread_.join();
    }
}

Result<void> ParameterManager::request_list_parameters(uint16_t axis_id, ListCallback cb) {
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        if (active_list_requests_.count(axis_id)) {
            return Result<void>::failure(
                {ErrorCode::Busy, "Parameter list request is already active for this axis"});
        }
        active_list_requests_.insert(axis_id);
    }

    {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        if (!running_.load(std::memory_order_acquire)) {
            std::lock_guard<std::mutex> st_lock(state_mutex_);
            active_list_requests_.erase(axis_id);
            return Result<void>::failure(
                {ErrorCode::InternalError, "Parameter manager is stopping; list request was rejected"});
        }

        task_queue_.push({[this, axis_id, cb = std::move(cb)]() {
            auto axis_res = runtime_.find_axis(axis_id);
            Result<std::vector<ParameterDescriptor>> result = Result<std::vector<ParameterDescriptor>>::failure({ErrorCode::NotFound, "Axis not found"});
            if (axis_res.ok()) {
                result = axis_res.value()->list_parameters();
            }

            {
                std::lock_guard<std::mutex> st_lock(state_mutex_);
                active_list_requests_.erase(axis_id);
            }

            if (cb) {
                cb(std::move(result));
            }
        }});
    }
    cv_.notify_one();
    return Result<void>::success();
}

Result<void> ParameterManager::request_read_parameters(uint16_t axis_id, ReadCallback cb) {
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        if (active_read_requests_.count(axis_id)) {
            return Result<void>::failure(
                {ErrorCode::Busy, "Parameter read request is already active for this axis"});
        }
        active_read_requests_.insert(axis_id);
    }

    {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        if (!running_.load(std::memory_order_acquire)) {
            std::lock_guard<std::mutex> st_lock(state_mutex_);
            active_read_requests_.erase(axis_id);
            return Result<void>::failure(
                {ErrorCode::InternalError, "Parameter manager is stopping; read request was rejected"});
        }

        task_queue_.push({[this, axis_id, cb = std::move(cb)]() {
            auto axis_res = runtime_.find_axis(axis_id);
            Result<ParameterSet> result = Result<ParameterSet>::failure({ErrorCode::NotFound, "Axis not found"});
            if (axis_res.ok()) {
                result = axis_res.value()->read_parameters();
            }

            {
                std::lock_guard<std::mutex> st_lock(state_mutex_);
                active_read_requests_.erase(axis_id);
            }

            if (cb) {
                cb(std::move(result));
            }
        }});
    }
    cv_.notify_one();
    return Result<void>::success();
}

void ParameterManager::request_apply_patch(uint16_t axis_id, ParameterPatch patch, WriteCallback cb) {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    task_queue_.push({[this, axis_id, patch = std::move(patch), cb = std::move(cb)]() {
        auto axis_res = runtime_.find_axis(axis_id);
        Result<void> result = Result<void>::failure({ErrorCode::NotFound, "Axis not found"});
        if (axis_res.ok()) {
            result = axis_res.value()->apply_parameter_patch(patch);
        }

        if (cb) {
            cb(std::move(result));
        }
    }});
    cv_.notify_one();
}

void ParameterManager::worker_loop() {
    while (true) {
        Task task;
        {
            std::unique_lock<std::mutex> lock(queue_mutex_);
            cv_.wait(lock, [this]() { return !running_ || !task_queue_.empty(); });

            if (!running_ && task_queue_.empty()) {
                break;
            }

            task = std::move(task_queue_.front());
            task_queue_.pop();
        }

        if (task.func) {
            task.func();
        }
    }
}

} // namespace motion_core
