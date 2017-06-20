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

#include "cartographer/mapping_2d/submaps.h"

#include <map>
#include <memory>
#include <set>
#include <string>

#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/common/lua_parameter_dictionary_test_helpers.h"
#include "cartographer/common/port.h"
#include "cartographer/transform/transform.h"
#include "gmock/gmock.h"

namespace cartographer {
namespace mapping_2d {
namespace {

TEST(SubmapsTest, TheRightNumberOfScansAreInserted) {
  constexpr int kNumRangeData = 10;
  auto parameter_dictionary = common::MakeDictionary(
      "return {"
      "resolution = 0.05, "
      "half_length = 10., "
      "num_range_data = " +
      std::to_string(kNumRangeData) +
      ", "
      "output_debug_images = false, "
      "range_data_inserter = {"
      "insert_free_space = true, "
      "hit_probability = 0.53, "
      "miss_probability = 0.495, "
      "},"
      "}");
  ActiveSubmaps submaps{CreateSubmapsOptions(parameter_dictionary.get())};
  std::set<std::shared_ptr<Submap>> all_submaps;
  for (int i = 0; i != 1000; ++i) {
    submaps.InsertRangeData({Eigen::Vector3f::Zero(), {}, {}});
    // Except for the first, maps should only be returned after enough scans.
    for (auto submap : submaps.submaps()) {
      all_submaps.insert(submap);
    }
    if (submaps.matching_index() != 0) {
      EXPECT_LE(kNumRangeData, submaps.submaps().front()->num_range_data());
    }
  }
  int correct_num_scans = 0;
  for (const auto& submap : all_submaps) {
    if (submap->num_range_data() == kNumRangeData * 2) {
      ++correct_num_scans;
    }
  }
  // Submaps should not be left without the right number of scans in them.
  EXPECT_EQ(correct_num_scans, all_submaps.size() - 2);
}

}  // namespace
}  // namespace mapping_2d
}  // namespace cartographer
