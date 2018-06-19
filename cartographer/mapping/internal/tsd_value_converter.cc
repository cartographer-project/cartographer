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

#include "cartographer/mapping/internal/tsd_value_converter.h"

namespace cartographer {
namespace mapping {

TSDValueConverter::TSDValueConverter(float max_tsd, float max_weight)
    : max_tsd_(max_tsd),
      min_tsd_(-max_tsd),
      max_weight_(max_weight),
      tsd_resolution_(32766.f / (max_tsd_ - min_tsd_)),
      weight_resolution_(32766.f / (max_weight_ - min_weight_)),
      value_to_tsd_(PrecomputeValueToTSD()),
      value_to_weight_(PrecomputeValueToWeight()) {}

// 0 is unknown, [1, 32767] maps to [min_tsd_ max_tsd_].
float TSDValueConverter::SlowValueToTSD(const uint16 value) const {
  CHECK_GE(value, 0);
  CHECK_LE(value, 32767);
  if (value == unknown_tsd_value_) {
    return min_tsd_;
  }
  const float kScale = (max_tsd_ - min_tsd_) / 32766.f;
  return value * kScale + (min_tsd_ - kScale);
}

std::vector<float> TSDValueConverter::PrecomputeValueToTSD() {
  std::vector<float> result;
  size_t num_values = std::numeric_limits<uint16>::max() + 1;
  result.reserve(num_values);
  for (size_t value = 0; value != num_values; ++value) {
    result.push_back(
        SlowValueToTSD(static_cast<uint16>(value) & ~update_marker_));
  }
  return result;
}

// 0 is unknown, [1, 32767] maps to [min_weight_ max_weight_].
float TSDValueConverter::SlowValueToWeight(const uint16 value) const {
  CHECK_GE(value, 0);
  CHECK_LE(value, 32767);
  if (value == unknown_weight_value_) {
    // Unknown cells have min_weight_.
    return min_weight_;
  }
  const float kScale = (max_weight_ - min_weight_) / 32766.f;
  return value * kScale + (min_weight_ - kScale);
}

std::vector<float> TSDValueConverter::PrecomputeValueToWeight() {
  std::vector<float> result;
  size_t num_values = std::numeric_limits<uint16>::max() + 1;
  result.reserve(num_values);
  for (size_t value = 0; value != num_values; ++value) {
    result.push_back(
        SlowValueToWeight(static_cast<uint16>(value) & ~update_marker_));
  }
  return result;
}

}  // namespace mapping
}  // namespace cartographer
