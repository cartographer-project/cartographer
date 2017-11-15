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

#include "cartographer/common/fixed_ratio_sampler.h"

#include "glog/logging.h"

namespace cartographer {
namespace common {

FixedRatioSampler::FixedRatioSampler(const double ratio) : ratio_(ratio) {
  CHECK_GT(ratio, 0.);
  CHECK_LE(ratio, 1.);
}

FixedRatioSampler::~FixedRatioSampler() {}

bool FixedRatioSampler::Pulse() {
  ++num_pulses_;
  if (static_cast<double>(num_samples_) / num_pulses_ < ratio_) {
    ++num_samples_;
    return true;
  }
  return false;
}

std::string FixedRatioSampler::DebugString() {
  return std::to_string(num_samples_) + " (" +
         std::to_string(100. * num_samples_ / num_pulses_) + "%)";
}

}  // namespace common
}  // namespace cartographer
