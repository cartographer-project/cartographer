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

#include <random>

#include "gtest/gtest.h"

namespace cartographer {
namespace mapping {
namespace {

TEST(ValueConversionTablesTest, EqualTables) {
  ValueConversionTables value_conversion_tables;
  const std::vector<float>* reference_table =
      value_conversion_tables.GetConversionTable(0.1f, 0.1f, 0.5f);
  const std::vector<float>* test_table =
      value_conversion_tables.GetConversionTable(0.1f, 0.1f, 0.5f);
  EXPECT_EQ(reference_table, test_table);
}

TEST(ValueConversionTablesTest, InequalTables) {
  ValueConversionTables value_conversion_tables;
  const std::vector<float>* reference_table =
      value_conversion_tables.GetConversionTable(0.1f, 0.1f, 0.5f);
  const std::vector<float>* test_table =
      value_conversion_tables.GetConversionTable(0.1f, 0.4f, 0.5f);
  EXPECT_FALSE(reference_table == test_table);
}

TEST(ValueConversionTablesTest, ValueConversion) {
  ValueConversionTables value_conversion_tables;
  std::mt19937 rng(42);
  std::uniform_real_distribution<float> bound_distribution(-10.f, 10.f);
  for (size_t sample_index = 0; sample_index < 100; ++sample_index) {
    const float bound_sample_0 = bound_distribution(rng);
    const float bound_sample_1 = bound_distribution(rng);
    const float lower_bound = std::min(bound_sample_0, bound_sample_1);
    const float upper_bound = std::max(bound_sample_0, bound_sample_1);
    const float undefined_value = bound_distribution(rng);
    const std::vector<float>* conversion_table =
        value_conversion_tables.GetConversionTable(undefined_value, lower_bound,
                                                   upper_bound);
    EXPECT_EQ((*conversion_table)[0], undefined_value);
    const float scale = (upper_bound - lower_bound) / 32766.f;
    for (uint16 i = 1; i < 32768; ++i) {
      EXPECT_EQ((*conversion_table)[i], i * scale + (lower_bound - scale));
    }
  }
}

}  // namespace
}  // namespace mapping
}  // namespace cartographer
