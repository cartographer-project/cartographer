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

#include "cartographer/mapping_2d/xy_index.h"

#include "gtest/gtest.h"

namespace cartographer {
namespace mapping_2d {
namespace {

TEST(XYIndexTest, XYIndexRangeIterator) {
  const CellLimits limits(5, 5);
  const Eigen::Array2i min(1, 2);
  const Eigen::Array2i max(3, 4);
  XYIndexRangeIterator it(limits, min, max);
  EXPECT_TRUE((min == *it.begin()).all()) << *it.begin();
  EXPECT_TRUE((Eigen::Array2i(1, 5) == *it.end()).all()) << *it.end();
  EXPECT_TRUE((min == *it).all()) << *it;
  int num_indices = 0;
  for (const Eigen::Array2i& xy_index :
       XYIndexRangeIterator(limits, min, max)) {
    LOG(INFO) << xy_index;
    EXPECT_TRUE((xy_index >= min).all());
    EXPECT_TRUE((xy_index <= max).all());
    ++num_indices;
  }
  EXPECT_EQ(9, num_indices);
}

}  // namespace
}  // namespace mapping_2d
}  // namespace cartographer
