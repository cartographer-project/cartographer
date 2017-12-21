#ifndef CARTOGRAPHER_COMMON_OPTIONAL_H_
#define CARTOGRAPHER_COMMON_OPTIONAL_H_

#include <memory>

#include "common/make_unique.h"
#include "glog/logging.h"

namespace cartographer {
namespace common {

template <class T>
class optional {
 public:
  optional() {}

  optional(const optional& other) {
    if (other.has_value()) {
      value_ = common::make_unique<T>(other.value());
    }
  }

  explicit optional(const T& value) { value_ = common::make_unique<T>(value); }

  bool has_value() const { return value_ != nullptr; }

  const T& value() const {
    CHECK(value_ != nullptr);
    return *value_;
  }

 private:
  std::unique_ptr<T> value_;
};

}  // namespace common
}  // namespace cartographer

#endif  // CARTOGRAPHER_COMMON_OPTIONAL_H_
