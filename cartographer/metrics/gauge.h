/*
 * Copyright 2018 The Cartographer Authors
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

#ifndef CARTOGRAPHER_METRICS_GAUGE_H_
#define CARTOGRAPHER_METRICS_GAUGE_H_

#include <map>
#include <vector>

namespace cartographer {
namespace metrics {

class Gauge {
 public:
  // Gauge instance that does nothing. Safe for use in static initializers.
  static Gauge* Null();

  virtual ~Gauge() = default;
  virtual void Increment() = 0;
  virtual void Increment(double by_value) = 0;
  virtual void Decrement() = 0;
  virtual void Decrement(double by_value) = 0;
  virtual void Set(double value) = 0;
};

}  // namespace metrics
}  // namespace cartographer

#endif  // CARTOGRAPHER_METRICS_GAUGE_H_
