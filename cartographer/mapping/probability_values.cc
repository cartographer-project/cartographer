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

#include "cartographer/mapping/probability_values.h"

namespace cartographer {
namespace mapping {

namespace {

// 0 is unknown, [1, 32767] maps to [min, max].
float SlowValueToFloat(const uint16 value, const uint16 unknown_value,
                       const float unknown_return, const float min,
                       const float max) {
  CHECK_GE(value, 0);
  CHECK_LE(value, 32767);
  if (value == unknown_value) {
    // Unknown cells have kMinProbability.
    return unknown_return;
  }
  const float kScale = (max - min) / 32766.f;
  return value * kScale + (min - kScale);
}

const std::vector<float>* PrecomputeValueToFloat(const uint16 unknown_value,
                                                 const float unknown_return,
                                                 const float min,
                                                 const float max) {
  std::vector<float>* result = new std::vector<float>;
  // Repeat two times, so that both values with and without the update marker
  // can be converted to a probability.
  for (int repeat = 0; repeat != 2; ++repeat) {
    for (int value = 0; value != 32768; ++value) {
      result->push_back(
          SlowValueToFloat(value, unknown_value, unknown_return, min, max));
    }
  }
  return result;
}

const std::vector<float>* PrecomputeValueToProbability() {
  return PrecomputeValueToFloat(kUnknownProbabilityValue, kMinProbability,
                                kMinProbability, kMaxProbability);
}

const std::vector<float>* PrecomputeValueToCorrespondenceCost() {
  return PrecomputeValueToFloat(kUnknownCorrespondenceValue,
                                kMaxCorrespondenceCost, kMinCorrespondenceCost,
                                kMaxCorrespondenceCost);
}

}  // namespace

const std::vector<float>* const kValueToProbability =
    PrecomputeValueToProbability();

const std::vector<float>* const kValueToCorrespondenceCost =
    PrecomputeValueToCorrespondenceCost();

std::vector<uint16> ComputeLookupTableToApplyOdds(const float odds) {
  std::vector<uint16> result;
  result.push_back(ProbabilityToValue(ProbabilityFromOdds(odds)) +
                   kUpdateMarker);
  for (int cell = 1; cell != 32768; ++cell) {
    result.push_back(ProbabilityToValue(ProbabilityFromOdds(
                         odds * Odds((*kValueToProbability)[cell]))) +
                     kUpdateMarker);
  }
  return result;
}

std::vector<uint16> ComputeLookupTableToApplyCorrespondenceCostOdds(
    float odds) {
  std::vector<uint16> result;
  result.push_back(CorrespondenceCostToValue(ProbabilityToCorrespondenceCost(
                       ProbabilityFromOdds(odds))) +
                   kUpdateMarker);
  for (int cell = 1; cell != 32768; ++cell) {
    result.push_back(
        CorrespondenceCostToValue(
            ProbabilityToCorrespondenceCost(ProbabilityFromOdds(
                odds * Odds(CorrespondenceCostToProbability(
                           (*kValueToCorrespondenceCost)[cell]))))) +
        kUpdateMarker);
  }
  return result;
}

}  // namespace mapping
}  // namespace cartographer
