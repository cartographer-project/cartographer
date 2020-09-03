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

#include "cartographer/mapping/2d/submap_2d.h"

#include <map>
#include <memory>
#include <set>
#include <string>

#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/common/lua_parameter_dictionary_test_helpers.h"
#include "cartographer/common/port.h"
#include "cartographer/mapping/2d/probability_grid.h"
#include "cartographer/transform/transform.h"
#include "gmock/gmock.h"

namespace cartographer {
namespace mapping {
namespace {

TEST(Submap2DTest, TheRightNumberOfRangeDataAreInserted) {
  constexpr int kNumRangeData = 10;
  auto parameter_dictionary = common::MakeDictionary(
      "return {"
      "num_range_data = " +
      std::to_string(kNumRangeData) +
      ", "
      "grid_options_2d = {"
      "grid_type = \"PROBABILITY_GRID\","
      "resolution = 0.05, "
      "},"
      "range_data_inserter = {"
      "range_data_inserter_type = \"PROBABILITY_GRID_INSERTER_2D\","
      "probability_grid_range_data_inserter = {"
      "insert_free_space = true, "
      "hit_probability = 0.53, "
      "miss_probability = 0.495, "
      "},"
      "tsdf_range_data_inserter = { "
      "truncation_distance = 2.0,"
      "maximum_weight = 10.,"
      "update_free_space = false,"
      "normal_estimation_options = {"
      "num_normal_samples = 2,"
      "sample_radius = 10.,"
      "},"
      "project_sdf_distance_to_scan_normal = false,"
      "update_weight_range_exponent = 0,"
      "update_weight_angle_scan_normal_to_ray_kernel_bandwidth = 0,"
      "update_weight_distance_cell_to_hit_kernel_bandwidth = 0,"
      "},"
      "},"
      "}");
  ActiveSubmaps2D submaps{CreateSubmapsOptions2D(parameter_dictionary.get())};
  std::set<std::shared_ptr<const Submap2D>> all_submaps;
  for (int i = 0; i != 1000; ++i) {
    auto insertion_submaps =
        submaps.InsertRangeData({Eigen::Vector3f::Zero(), {}, {}});
    // Except for the first, maps should only be returned after enough range
    // data.
    for (const auto& submap : insertion_submaps) {
      all_submaps.insert(submap);
    }
    if (submaps.submaps().size() > 1) {
      EXPECT_LE(kNumRangeData, submaps.submaps().front()->num_range_data());
    }
  }
  EXPECT_EQ(2, submaps.submaps().size());
  int correct_num_finished_submaps = 0;
  int num_unfinished_submaps = 0;
  for (const auto& submap : all_submaps) {
    if (submap->num_range_data() == kNumRangeData * 2) {
      ++correct_num_finished_submaps;
    } else {
      EXPECT_EQ(kNumRangeData, submap->num_range_data());
      ++num_unfinished_submaps;
    }
  }
  // Submaps should not be left without the right number of range data in them.
  EXPECT_EQ(correct_num_finished_submaps, all_submaps.size() - 1);
  EXPECT_EQ(1, num_unfinished_submaps);
}

TEST(Submap2DTest, ToFromProto) {
  MapLimits expected_map_limits(1., Eigen::Vector2d(2., 3.),
                                CellLimits(100, 110));
  ValueConversionTables conversion_tables;
  Submap2D expected(Eigen::Vector2f(4.f, 5.f),
                    absl::make_unique<ProbabilityGrid>(expected_map_limits,
                                                       &conversion_tables),
                    &conversion_tables);
  const proto::Submap proto =
      expected.ToProto(true /* include_probability_grid_data */);
  EXPECT_TRUE(proto.has_submap_2d());
  EXPECT_FALSE(proto.has_submap_3d());
  const auto actual = Submap2D(proto.submap_2d(), &conversion_tables);
  EXPECT_TRUE(expected.local_pose().translation().isApprox(
      actual.local_pose().translation(), 1e-6));
  EXPECT_TRUE(expected.local_pose().rotation().isApprox(
      actual.local_pose().rotation(), 1e-6));
  EXPECT_EQ(expected.num_range_data(), actual.num_range_data());
  EXPECT_EQ(expected.insertion_finished(), actual.insertion_finished());
  EXPECT_NEAR(expected.grid()->limits().resolution(),
              actual.grid()->limits().resolution(), 1e-6);
  EXPECT_TRUE(expected.grid()->limits().max().isApprox(
      actual.grid()->limits().max(), 1e-6));
  EXPECT_EQ(expected.grid()->limits().cell_limits().num_x_cells,
            actual.grid()->limits().cell_limits().num_x_cells);
}

}  // namespace
}  // namespace mapping
}  // namespace cartographer
