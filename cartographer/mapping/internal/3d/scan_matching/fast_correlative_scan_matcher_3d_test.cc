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

#include "cartographer/mapping/internal/3d/scan_matching/fast_correlative_scan_matcher_3d.h"

#include <algorithm>
#include <cmath>
#include <random>
#include <string>

#include "absl/memory/memory.h"
#include "cartographer/common/internal/testing/lua_parameter_dictionary_test_helpers.h"
#include "cartographer/mapping/3d/range_data_inserter_3d.h"
#include "cartographer/transform/rigid_transform_test_helpers.h"
#include "cartographer/transform/transform.h"
#include "gtest/gtest.h"

namespace cartographer {
namespace mapping {
namespace scan_matching {
namespace {

class FastCorrelativeScanMatcher3DTest : public ::testing::Test {
 protected:
  FastCorrelativeScanMatcher3DTest()
      : range_data_inserter_(CreateRangeDataInserterTestOptions3D()),
        options_(CreateFastCorrelativeScanMatcher3DTestOptions3D(6)) {}

  void SetUp() override {
    point_cloud_ = sensor::PointCloud({{Eigen::Vector3f(4.f, 0.f, 0.f)},
                                       {Eigen::Vector3f(4.5f, 0.f, 0.f)},
                                       {Eigen::Vector3f(5.f, 0.f, 0.f)},
                                       {Eigen::Vector3f(5.5f, 0.f, 0.f)},
                                       {Eigen::Vector3f(0.f, 4.f, 0.f)},
                                       {Eigen::Vector3f(0.f, 4.5f, 0.f)},
                                       {Eigen::Vector3f(0.f, 5.f, 0.f)},
                                       {Eigen::Vector3f(0.f, 5.5f, 0.f)},
                                       {Eigen::Vector3f(0.f, 0.f, 4.f)},
                                       {Eigen::Vector3f(0.f, 0.f, 4.5f)},
                                       {Eigen::Vector3f(0.f, 0.f, 5.f)},
                                       {Eigen::Vector3f(0.f, 0.f, 5.5f)}});
  }

  transform::Rigid3f GetRandomPose() {
    const float x = 0.7f * distribution_(prng_);
    const float y = 0.7f * distribution_(prng_);
    const float z = 0.7f * distribution_(prng_);
    const float theta = 0.2f * distribution_(prng_);
    return transform::Rigid3f::Translation(Eigen::Vector3f(x, y, z)) *
           transform::Rigid3f::Rotation(
               Eigen::AngleAxisf(theta, Eigen::Vector3f::UnitZ()));
  }

  static proto::FastCorrelativeScanMatcherOptions3D
  CreateFastCorrelativeScanMatcher3DTestOptions3D(
      const int branch_and_bound_depth) {
    auto parameter_dictionary = common::MakeDictionary(
        "return {"
        "branch_and_bound_depth = " +
        std::to_string(branch_and_bound_depth) +
        ", "
        "full_resolution_depth = " +
        std::to_string(branch_and_bound_depth) +
        ", "
        "min_rotational_score = 0.1, "
        // Unknown space has kMinProbability = 0.1, so we need to make sure here
        // to pick a larger number otherwise we always find matches.
        "min_low_resolution_score = 0.15, "
        "linear_xy_search_window = 0.8, "
        "linear_z_search_window = 0.8, "
        "angular_search_window = 0.3, "
        "}");
    return CreateFastCorrelativeScanMatcherOptions3D(
        parameter_dictionary.get());
  }

  static mapping::proto::RangeDataInserterOptions3D
  CreateRangeDataInserterTestOptions3D() {
    auto parameter_dictionary = common::MakeDictionary(
        "return { "
        "hit_probability = 0.7, "
        "miss_probability = 0.4, "
        "num_free_space_voxels = 5, "
        "intensity_threshold = 100.0, "
        "}");
    return CreateRangeDataInserterOptions3D(parameter_dictionary.get());
  }

  std::unique_ptr<FastCorrelativeScanMatcher3D> GetFastCorrelativeScanMatcher(
      const proto::FastCorrelativeScanMatcherOptions3D& options,
      const transform::Rigid3f& pose) {
    hybrid_grid_ = absl::make_unique<HybridGrid>(0.05f);
    range_data_inserter_.Insert(
        sensor::RangeData{pose.translation(),
                          sensor::TransformPointCloud(point_cloud_, pose),
                          {}},
        hybrid_grid_.get(),
        /*intensity_hybrid_grid=*/nullptr);
    hybrid_grid_->FinishUpdate();

    return absl::make_unique<FastCorrelativeScanMatcher3D>(
        *hybrid_grid_, hybrid_grid_.get(), &GetRotationalScanMatcherHistogram(),
        options);
  }

  TrajectoryNode::Data CreateConstantData(
      const sensor::PointCloud& low_resolution_point_cloud) {
    return TrajectoryNode::Data{common::FromUniversal(0),
                                Eigen::Quaterniond::Identity(),
                                {},
                                point_cloud_,
                                low_resolution_point_cloud,
                                GetRotationalScanMatcherHistogram()};
  }

  const Eigen::VectorXf& GetRotationalScanMatcherHistogram() {
    return rotational_scan_matcher_histogram_;
  }

  std::mt19937 prng_ = std::mt19937(42);
  std::uniform_real_distribution<float> distribution_ =
      std::uniform_real_distribution<float>(-1.f, 1.f);
  RangeDataInserter3D range_data_inserter_;
  const proto::FastCorrelativeScanMatcherOptions3D options_;
  sensor::PointCloud point_cloud_;
  std::unique_ptr<HybridGrid> hybrid_grid_;
  const Eigen::VectorXf rotational_scan_matcher_histogram_ =
      Eigen::VectorXf::Zero(10);
};

constexpr float kMinScore = 0.1f;

TEST_F(FastCorrelativeScanMatcher3DTest, CorrectPoseForMatch) {
  for (int i = 0; i != 20; ++i) {
    const auto expected_pose = GetRandomPose();

    std::unique_ptr<FastCorrelativeScanMatcher3D> fast_correlative_scan_matcher(
        GetFastCorrelativeScanMatcher(options_, expected_pose));

    const std::unique_ptr<FastCorrelativeScanMatcher3D::Result> result =
        fast_correlative_scan_matcher->Match(
            transform::Rigid3d::Identity(), transform::Rigid3d::Identity(),
            CreateConstantData(point_cloud_), kMinScore);
    EXPECT_THAT(result, testing::NotNull());
    EXPECT_LT(kMinScore, result->score);
    EXPECT_LT(0.09f, result->rotational_score);
    EXPECT_LT(0.14f, result->low_resolution_score);
    EXPECT_THAT(expected_pose,
                transform::IsNearly(result->pose_estimate.cast<float>(), 0.05f))
        << "Actual: " << transform::ToProto(result->pose_estimate).DebugString()
        << "\nExpected: " << transform::ToProto(expected_pose).DebugString();

    const std::unique_ptr<FastCorrelativeScanMatcher3D::Result>
        low_resolution_result = fast_correlative_scan_matcher->Match(
            transform::Rigid3d::Identity(), transform::Rigid3d::Identity(),
            CreateConstantData(
                sensor::PointCloud({{Eigen::Vector3f(42.f, 42.f, 42.f)}})),
            kMinScore);
    EXPECT_THAT(low_resolution_result, testing::IsNull())
        << low_resolution_result->low_resolution_score;
  }
}

TEST_F(FastCorrelativeScanMatcher3DTest, CorrectPoseForMatchFullSubmap) {
  const auto expected_pose = GetRandomPose();

  std::unique_ptr<FastCorrelativeScanMatcher3D> fast_correlative_scan_matcher(
      GetFastCorrelativeScanMatcher(options_, expected_pose));

  const std::unique_ptr<FastCorrelativeScanMatcher3D::Result> result =
      fast_correlative_scan_matcher->MatchFullSubmap(
          Eigen::Quaterniond::Identity(), Eigen::Quaterniond::Identity(),
          CreateConstantData(point_cloud_), kMinScore);
  EXPECT_THAT(result, testing::NotNull());
  EXPECT_LT(kMinScore, result->score);
  EXPECT_LT(0.09f, result->rotational_score);
  EXPECT_LT(0.14f, result->low_resolution_score);
  EXPECT_THAT(expected_pose,
              transform::IsNearly(result->pose_estimate.cast<float>(), 0.05f))
      << "Actual: " << transform::ToProto(result->pose_estimate).DebugString()
      << "\nExpected: " << transform::ToProto(expected_pose).DebugString();

  const std::unique_ptr<FastCorrelativeScanMatcher3D::Result>
      low_resolution_result = fast_correlative_scan_matcher->MatchFullSubmap(
          Eigen::Quaterniond::Identity(), Eigen::Quaterniond::Identity(),
          CreateConstantData(
              sensor::PointCloud({{Eigen::Vector3f(42.f, 42.f, 42.f)}})),
          kMinScore);
  EXPECT_THAT(low_resolution_result, testing::IsNull())
      << low_resolution_result->low_resolution_score;
}

}  // namespace
}  // namespace scan_matching
}  // namespace mapping
}  // namespace cartographer
