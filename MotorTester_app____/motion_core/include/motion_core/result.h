#pragma once

#include <utility>

namespace motion_core {

enum class ErrorCode {
    Ok = 0,
    InvalidArgument,
    NotFound,
    AlreadyExists,
    NotConnected,
    Busy,
    Timeout,
    TransportFailure,
    ProtocolFailure,
    Unsupported,
    PermissionDenied,
    InternalError
};

struct Error {
    ErrorCode code{ErrorCode::Ok};
    const char* message{"ok"};

    [[nodiscard]] bool ok() const noexcept { return code == ErrorCode::Ok; }

    static Error success() noexcept { return {}; }
};

template <typename T>
class Result {
public:
    static Result<T> success(T value) {
        return Result<T>(std::move(value), Error::success(), true);
    }

    static Result<T> failure(Error error) {
        return Result<T>(T{}, error, false);
    }

    [[nodiscard]] bool ok() const noexcept { return ok_; }
    [[nodiscard]] const T& value() const noexcept { return value_; }
    [[nodiscard]] T& value() noexcept { return value_; }
    [[nodiscard]] const Error& error() const noexcept { return error_; }

private:
    Result(T value, Error error, bool ok)
        : value_(std::move(value)), error_(error), ok_(ok) {}

    T value_{};
    Error error_{};
    bool ok_{false};
};

template <>
class Result<void> {
public:
    static Result<void> success() {
        return Result<void>(Error::success(), true);
    }

    static Result<void> failure(Error error) {
        return Result<void>(error, false);
    }

    [[nodiscard]] bool ok() const noexcept { return ok_; }
    [[nodiscard]] const Error& error() const noexcept { return error_; }

private:
    Result(Error error, bool ok) : error_(error), ok_(ok) {}

    Error error_{};
    bool ok_{false};
};

} // namespace motion_core
