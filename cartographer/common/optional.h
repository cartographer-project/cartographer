/*
 * Copyright 2017 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef CARTOGRAPHER_COMMON_OPTIONAL_H_
#define CARTOGRAPHER_COMMON_OPTIONAL_H_

#include <memory>

#include "cartographer/common/make_unique.h"
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

  optional<T>& operator=(const T& other_value) {
    this->value_ = common::make_unique<T>(other_value);
    return *this;
  }

  optional<T>& operator=(const optional<T>& other) {
    if (!other.has_value()) {
      this->value_ = nullptr;
    } else {
      this->value_ = common::make_unique<T>(other.value());
    }
    return *this;
  }

 private:
  std::unique_ptr<T> value_;
};

}  // namespace common
}  // namespace cartographer

#endif  // CARTOGRAPHER_COMMON_OPTIONAL_H_
