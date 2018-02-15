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

#include "cartographer/metrics/gauge.h"

namespace cartographer {
namespace metrics {

namespace {

// Implementation of gauge that does nothing.
class NullGauge : public Gauge {
 public:
  void Increment() override{};
  void Increment(double) override{};
  void Decrement() override{};
  void Decrement(double) override{};
  void Set(double) override{};
};

}  // namespace

Gauge* Gauge::Null() {
  static NullGauge null_gauge;
  return &null_gauge;
}

}  // namespace metrics
}  // namespace cartographer
