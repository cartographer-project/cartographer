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

#include "cartographer/mapping/value_conversion_tables.h"

#include "cartographer/common/make_unique.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping {
namespace {

// 0 is unknown, [1, 32767] maps to [lower_bound, upper_bound].
float SlowValueToBoundedFloat(const uint16 value, const uint16 unknown_value,
                              const float unknown_result,
                              const float lower_bound,
                              const float upper_bound) {
  CHECK_GE(value, 0);
  CHECK_LE(value, 32767);
  if (value == unknown_value) return unknown_result;
  const float kScale = (upper_bound - lower_bound) / 32766.f;
  return value * kScale + (lower_bound - kScale);
}

std::unique_ptr<std::vector<float>> PrecomputeValueToBoundedFloat(
    const uint16 unknown_value, const float unknown_result,
    const float lower_bound, const float upper_bound) {
  auto result = common::make_unique<std::vector<float>>();
  // Repeat two times, so that both values with and without the update marker
  // can be converted to a probability.
  for (int repeat = 0; repeat != 2; ++repeat) {
    for (int value = 0; value != 32768; ++value) {
      result->push_back(SlowValueToBoundedFloat(
          value, unknown_value, unknown_result, lower_bound, upper_bound));
    }
  }
  return result;
}
}  // namespace

const std::vector<float>* ValueConversionTables::GetConversionTable(
    float unknown_result, float lower_bound, float upper_bound) {
  if (lower_bound == 0.f && upper_bound == 0.f) {
    LOG(WARNING) << "ValueConversionTables: upper_bound and upper_bound "
                    "are initialized with 0 indicating an older version of the "
                    "protobuf format. Loading default values.";
    lower_bound = 0.1f;
    upper_bound = 0.9f;
  }
  std::tuple<float, float, float> bounds =
      std::make_tuple(unknown_result, lower_bound, upper_bound);
  auto lookup_table_iterator = bounds_to_lookup_table_.find(bounds);
  if (lookup_table_iterator == bounds_to_lookup_table_.end()) {
    auto insertion_result = bounds_to_lookup_table_.emplace(
        bounds, PrecomputeValueToBoundedFloat(0, unknown_result, lower_bound,
                                              upper_bound));
    return insertion_result.first->second.get();
  } else {
    return lookup_table_iterator->second.get();
  }
}

}  // namespace mapping
}  // namespace cartographer
