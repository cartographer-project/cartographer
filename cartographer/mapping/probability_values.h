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

#ifndef CARTOGRAPHER_MAPPING_PROBABILITY_VALUES_H_
#define CARTOGRAPHER_MAPPING_PROBABILITY_VALUES_H_

#include <cmath>
#include <vector>

#include "cartographer/common/math.h"
#include "cartographer/common/port.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping {

inline float Odds(float probability) {
  return probability / (1.f - probability);
}

inline float ProbabilityFromOdds(const float odds) {
  return odds / (odds + 1.f);
}

constexpr float kMinProbability = 0.1f;
constexpr float kMaxProbability = 1.f - kMinProbability;

// Clamps probability to be in the range [kMinProbability, kMaxProbability].
inline float ClampProbability(const float probability) {
  return common::Clamp(probability, kMinProbability, kMaxProbability);
}

constexpr uint16 kUnknownProbabilityValue = 0;
constexpr uint16 kUpdateMarker = 1u << 15;

// Converts a probability to a uint16 in the [1, 32767] range.
inline uint16 ProbabilityToValue(const float probability) {
  const int value =
      common::RoundToInt((ClampProbability(probability) - kMinProbability) *
                         (32766.f / (kMaxProbability - kMinProbability))) +
      1;
  // DCHECK for performance.
  DCHECK_GE(value, 1);
  DCHECK_LE(value, 32767);
  return value;
}

extern const std::vector<float>* const kValueToProbability;

// Converts a uint16 (which may or may not have the update marker set) to a
// probability in the range [kMinProbability, kMaxProbability].
inline float ValueToProbability(const uint16 value) {
  return (*kValueToProbability)[value];
}

std::vector<uint16> ComputeLookupTableToApplyOdds(float odds);

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_PROBABILITY_VALUES_H_
