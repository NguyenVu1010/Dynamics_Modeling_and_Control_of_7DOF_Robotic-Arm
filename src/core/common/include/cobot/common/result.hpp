#pragma once

#include <optional>
#include <string>
#include <utility>

namespace cobot::common {

// Per-message processing never throws: it returns a Status or Result<T>.
// `reason` is a short human-readable string for the log, not for control flow.
//
// Allocation: ok() never allocates. error(const char* literal) stores only the pointer
// and does not allocate either, so hot paths (per sample, per scan) use it with a string
// LITERAL. Lifetime: the pointer must stay valid for the life of the Status and of every
// copy, so pass string literals only (static storage), never c_str() of a std::string or
// a stack buffer. error(std::string) owns its text and may allocate; use it off the hot
// path, where the reason is built at run time. reason() builds a std::string on each call
// (it may allocate); callers on hot paths only read is_ok() and log/count the reason
// elsewhere.
class Status {
 public:
  static Status ok() { return Status{}; }
  /// Precondition: `literal` points to a string literal (static storage). nullptr is
  /// treated as the literal "unknown error".
  static Status error(const char* literal) {
    Status s;
    s.literal_ = literal != nullptr ? literal : "unknown error";
    return s;
  }
  static Status error(std::string reason) {
    Status s;
    s.owned_ = std::move(reason);
    s.has_owned_ = true;
    return s;
  }

  bool is_ok() const { return literal_ == nullptr && !has_owned_; }
  explicit operator bool() const { return is_ok(); }
  /// The reason text, empty for ok(). Returned by value (built from whichever overload set
  /// it); may allocate.
  std::string reason() const { return literal_ != nullptr ? std::string(literal_) : owned_; }

 private:
  Status() = default;
  const char* literal_ = nullptr;  // set by error(const char*), static storage
  std::string owned_;              // set by error(std::string)
  bool has_owned_ = false;
};

template <typename T>
class Result {
 public:
  static Result ok(T value) { return Result{std::move(value)}; }
  /// No allocation; same lifetime rule as Status::error(const char*): literals only.
  static Result error(const char* literal) { return Result{Status::error(literal)}; }
  static Result error(std::string reason) { return Result{Status::error(std::move(reason))}; }

  bool is_ok() const { return value_.has_value(); }
  explicit operator bool() const { return is_ok(); }
  /// See Status::reason(): by value, empty when ok.
  std::string reason() const { return status_.reason(); }

  // Precondition: is_ok(). Callers check first; there is no exception path.
  const T& value() const& { return *value_; }
  T& value() & { return *value_; }
  T&& value() && { return std::move(*value_); }

 private:
  explicit Result(T value) : value_(std::move(value)), status_(Status::ok()) {}
  explicit Result(Status status) : status_(std::move(status)) {}
  std::optional<T> value_;
  Status status_;
};

}  // namespace cobot::common
