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

#include "cartographer/mapping_3d/scan_matching/fast_correlative_scan_matcher.h"

#include <algorithm>
#include <cmath>
#include <random>
#include <string>

#include "cartographer/common/lua_parameter_dictionary_test_helpers.h"
#include "cartographer/mapping_3d/range_data_inserter.h"
#include "cartographer/transform/rigid_transform_test_helpers.h"
#include "cartographer/transform/transform.h"
#include "gtest/gtest.h"

namespace cartographer {
namespace mapping_3d {
namespace scan_matching {
namespace {

proto::FastCorrelativeScanMatcherOptions
CreateFastCorrelativeScanMatcherTestOptions(const int branch_and_bound_depth) {
  auto parameter_dictionary = common::MakeDictionary(
      "return {"
      "branch_and_bound_depth = " +
      std::to_string(branch_and_bound_depth) +
      ", "
      "full_resolution_depth = " +
      std::to_string(branch_and_bound_depth) +
      ", "
      "rotational_histogram_size = 30, "
      "min_rotational_score = 0.1, "
      "linear_xy_search_window = 0.8, "
      "linear_z_search_window = 0.8, "
      "angular_search_window = 0.3, "
      "}");
  return CreateFastCorrelativeScanMatcherOptions(parameter_dictionary.get());
}

mapping_3d::proto::RangeDataInserterOptions
CreateRangeDataInserterTestOptions() {
  auto parameter_dictionary = common::MakeDictionary(
      "return { "
      "hit_probability = 0.7, "
      "miss_probability = 0.4, "
      "num_free_space_voxels = 5, "
      "}");
  return CreateRangeDataInserterOptions(parameter_dictionary.get());
}

TEST(FastCorrelativeScanMatcherTest, CorrectPose) {
  std::mt19937 prng(42);
  std::uniform_real_distribution<float> distribution(-1.f, 1.f);
  RangeDataInserter range_data_inserter(CreateRangeDataInserterTestOptions());
  constexpr float kMinScore = 0.1f;
  const auto options = CreateFastCorrelativeScanMatcherTestOptions(5);

  sensor::PointCloud point_cloud{
      Eigen::Vector3f(4.f, 0.f, 0.f), Eigen::Vector3f(4.5f, 0.f, 0.f),
      Eigen::Vector3f(5.f, 0.f, 0.f), Eigen::Vector3f(5.5f, 0.f, 0.f),
      Eigen::Vector3f(0.f, 4.f, 0.f), Eigen::Vector3f(0.f, 4.5f, 0.f),
      Eigen::Vector3f(0.f, 5.f, 0.f), Eigen::Vector3f(0.f, 5.5f, 0.f),
      Eigen::Vector3f(0.f, 0.f, 4.f), Eigen::Vector3f(0.f, 0.f, 4.5f),
      Eigen::Vector3f(0.f, 0.f, 5.f), Eigen::Vector3f(0.f, 0.f, 5.5f)};

  for (int i = 0; i != 20; ++i) {
    const float x = 0.7f * distribution(prng);
    const float y = 0.7f * distribution(prng);
    const float z = 0.7f * distribution(prng);
    const float theta = 0.2f * distribution(prng);
    const auto expected_pose =
        transform::Rigid3f::Translation(Eigen::Vector3f(x, y, z)) *
        transform::Rigid3f::Rotation(
            Eigen::AngleAxisf(theta, Eigen::Vector3f::UnitZ()));

    HybridGrid hybrid_grid(0.05f);
    range_data_inserter.Insert(
        sensor::RangeData{
            expected_pose.translation(),
            sensor::TransformPointCloud(point_cloud, expected_pose),
            {}},
        &hybrid_grid);
    hybrid_grid.FinishUpdate();

    FastCorrelativeScanMatcher fast_correlative_scan_matcher(hybrid_grid, {},
                                                             options);
    float score = 0.f;
    transform::Rigid3d pose_estimate;
    float rotational_score = 0.f;
    EXPECT_TRUE(fast_correlative_scan_matcher.Match(
        transform::Rigid3d::Identity(), point_cloud, point_cloud, kMinScore,
        &score, &pose_estimate, &rotational_score));
    EXPECT_LT(kMinScore, score);
    EXPECT_LT(0.09f, rotational_score);
    EXPECT_THAT(expected_pose,
                transform::IsNearly(pose_estimate.cast<float>(), 0.05f))
        << "Actual: " << transform::ToProto(pose_estimate).DebugString()
        << "\nExpected: " << transform::ToProto(expected_pose).DebugString();
  }
}

}  // namespace
}  // namespace scan_matching
}  // namespace mapping_3d
}  // namespace cartographer
