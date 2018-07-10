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

}  // namespace
}  // namespace mapping
}  // namespace cartographer
