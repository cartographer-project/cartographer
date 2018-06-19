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

#include "cartographer/mapping/tsd_value_converter.h"

namespace cartographer {
namespace mapping {

TSDValueConverter::TSDValueConverter(float max_tsdf, float max_weight)
    : kMaxTSD(max_tsdf),
      kMinTSD(-max_tsdf),
      kMaxWeight(max_weight),
      kValueToTSDF(PrecomputeValueToTSD()),
      kValueToWeight(PrecomputeValueToWeight()) {}

// 0 is unknown, [1, 32767] maps to [kMinTSD kMaxTSD].
float TSDValueConverter::SlowValueToTSD(const uint16 value) const {
  CHECK_GE(value, 0);
  CHECK_LE(value, 32767);
  if (value == kUnknownTSDValue) {
    // Unknown cells have kMinProbability.
    return kMinTSD;
  }
  const float kScale = (kMaxTSD - kMinTSD) / 32766.f;
  return value * kScale + (kMinTSD - kScale);
}

std::vector<float> TSDValueConverter::PrecomputeValueToTSD() {
  std::vector<float> result;
  // Repeat two times, so that both values with and without the update marker
  // can be converted to a tsd.
  for (int repeat = 0; repeat != 2; ++repeat) {
    for (int value = 0; value != 32768; ++value) {
      result.push_back(SlowValueToTSD(value));
    }
  }
  return result;
}

// 0 is unknown, [1, 32767] maps to [kMinWeight kMaxWeight].
float TSDValueConverter::SlowValueToWeight(const uint16 value) const {
  CHECK_GE(value, 0);
  CHECK_LE(value, 32767);
  if (value == kUnknownWeightValue) {
    // Unknown cells have kMinWeight.
    return kMinWeight;
  }
  const float kScale = (kMaxWeight - kMinWeight) / 32766.f;
  return value * kScale + (kMinWeight - kScale);
}

std::vector<float> TSDValueConverter::PrecomputeValueToWeight() {
  std::vector<float> result;
  // Repeat two times, so that both values with and without the update marker
  // can be converted to a weight.
  for (int repeat = 0; repeat != 2; ++repeat) {
    for (int value = 0; value != 32768; ++value) {
      result.push_back(SlowValueToWeight(value));
    }
  }
  return result;
}

}  // namespace mapping
}  // namespace cartographer