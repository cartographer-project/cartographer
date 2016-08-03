/*
 * Copyright 2016 The Cartographer Authors
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

#include "cartographer/common/time.h"

#include <string>

namespace cartographer {
namespace common {

Duration FromSeconds(const double seconds) {
  return Duration(static_cast<int64>(1e7 * seconds));
}

double ToSeconds(const Duration duration) { return duration.count() * 1e-7; }

Time FromUniversal(const int64 ticks) { return Time(Duration(ticks)); }

int64 ToUniversal(const Time time) { return time.time_since_epoch().count(); }

std::ostream& operator<<(std::ostream& os, const Time time) {
  os << std::to_string(ToUniversal(time));
  return os;
}

common::Duration FromMilliseconds(int64 milliseconds) {
  return common::Duration(milliseconds * 10000);
}

}  // namespace common
}  // namespace cartographer
