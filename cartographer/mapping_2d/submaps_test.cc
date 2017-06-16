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

// TODO (bradon-northcutt): Source these testing parameters from a common place.
const static double kPrecision_ = 1e-6;

}  // namespace
TEST(SubmapsTest, TheRightNumberOfScansAreInserted) {
  constexpr int kNumRangeData = 10;
  auto parameter_dictionary = common::MakeDictionary(
      "return {"
      "resolution = 0.05, "
      "half_length = 10., "
      "num_range_data = " +
      std::to_string(kNumRangeData) +
      ", "
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

TEST(SubmapsTest, ToFromProto) {
  Submap expected(MapLimits(1, Eigen::Vector2d(2, 3), CellLimits(100, 110)),
                  Eigen::Vector2f(4, 5));
  const auto actual = Submap(ToProto(expected));
  EXPECT_TRUE(expected.local_pose().translation().isApprox(
      actual.local_pose().translation(), kPrecision_));
  EXPECT_TRUE(expected.local_pose().rotation().isApprox(
      actual.local_pose().rotation(), kPrecision_));
  EXPECT_EQ(expected.num_range_data(), actual.num_range_data());
  EXPECT_EQ(expected.finished(), actual.finished());
  EXPECT_NEAR(expected.probability_grid().limits().resolution(),
              actual.probability_grid().limits().resolution(), kPrecision_);
  EXPECT_TRUE(expected.probability_grid().limits().max().isApprox(
      actual.probability_grid().limits().max(), kPrecision_));
  EXPECT_EQ(expected.probability_grid().limits().cell_limits().num_x_cells,
            actual.probability_grid().limits().cell_limits().num_x_cells);
  // TODO (brandon-northcutt) Verify SubmapID submap_index and trajectory_id
  // when serializing entire submap vectors.
}
}  // namespace mapping_2d
}  // namespace cartographer
