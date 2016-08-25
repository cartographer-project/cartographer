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

#include "cartographer/mapping_2d/scan_matching/fast_correlative_scan_matcher.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <random>
#include <string>

#include "cartographer/common/lua_parameter_dictionary_test_helpers.h"
#include "cartographer/mapping_2d/laser_fan_inserter.h"
#include "cartographer/mapping_2d/probability_grid.h"
#include "cartographer/transform/rigid_transform_test_helpers.h"
#include "cartographer/transform/transform.h"
#include "gtest/gtest.h"

namespace cartographer {
namespace mapping_2d {
namespace scan_matching {
namespace {

TEST(PrecomputationGridTest, CorrectValues) {
  // Create a probability grid with random values that can be exactly
  // represented by uint8 values.
  std::mt19937 prng(42);
  std::uniform_int_distribution<int> distribution(0, 255);
  ProbabilityGrid probability_grid(
      MapLimits(0.05, Eigen::Vector2d(5., 5.), CellLimits(250, 250)));
  probability_grid.StartUpdate();
  for (const Eigen::Array2i& xy_index :
       XYIndexRangeIterator(Eigen::Array2i(50, 50), Eigen::Array2i(249, 249))) {
    probability_grid.SetProbability(
        xy_index, PrecomputationGrid::ToProbability(distribution(prng)));
  }

  std::vector<float> reusable_intermediate_grid;
  for (const int width : {1, 2, 3, 8}) {
    PrecomputationGrid precomputation_grid(
        probability_grid, probability_grid.limits().cell_limits(), width,
        &reusable_intermediate_grid);
    for (const Eigen::Array2i& xy_index :
         XYIndexRangeIterator(probability_grid.limits().cell_limits())) {
      float max_score = -std::numeric_limits<float>::infinity();
      for (int y = 0; y != width; ++y) {
        for (int x = 0; x != width; ++x) {
          max_score = std::max<float>(
              max_score,
              probability_grid.GetProbability(xy_index + Eigen::Array2i(x, y)));
        }
      }
      EXPECT_NEAR(max_score, PrecomputationGrid::ToProbability(
                                 precomputation_grid.GetValue(xy_index)),
                  1e-4);
    }
  }
}

TEST(PrecomputationGridTest, TinyProbabilityGrid) {
  std::mt19937 prng(42);
  std::uniform_int_distribution<int> distribution(0, 255);
  ProbabilityGrid probability_grid(
      MapLimits(0.05, Eigen::Vector2d(0.1, 0.1), CellLimits(4, 4)));
  probability_grid.StartUpdate();
  for (const Eigen::Array2i& xy_index :
       XYIndexRangeIterator(probability_grid.limits().cell_limits())) {
    probability_grid.SetProbability(
        xy_index, PrecomputationGrid::ToProbability(distribution(prng)));
  }

  std::vector<float> reusable_intermediate_grid;
  for (const int width : {1, 2, 3, 8, 200}) {
    PrecomputationGrid precomputation_grid(
        probability_grid, probability_grid.limits().cell_limits(), width,
        &reusable_intermediate_grid);
    for (const Eigen::Array2i& xy_index :
         XYIndexRangeIterator(probability_grid.limits().cell_limits())) {
      float max_score = -std::numeric_limits<float>::infinity();
      for (int y = 0; y != width; ++y) {
        for (int x = 0; x != width; ++x) {
          max_score = std::max<float>(
              max_score,
              probability_grid.GetProbability(xy_index + Eigen::Array2i(x, y)));
        }
      }
      EXPECT_NEAR(max_score, PrecomputationGrid::ToProbability(
                                 precomputation_grid.GetValue(xy_index)),
                  1e-4);
    }
  }
}

proto::FastCorrelativeScanMatcherOptions
CreateFastCorrelativeScanMatcherTestOptions(const int branch_and_bound_depth) {
  auto parameter_dictionary =
      common::MakeDictionary(R"text(
      return {
         linear_search_window = 3.,
         angular_search_window = 1.,
         covariance_scale = 1.,
         branch_and_bound_depth = )text" +
                             std::to_string(branch_and_bound_depth) + "}");
  return CreateFastCorrelativeScanMatcherOptions(parameter_dictionary.get());
}

mapping_2d::proto::LaserFanInserterOptions CreateLaserFanInserterTestOptions() {
  auto parameter_dictionary = common::MakeDictionary(R"text(
      return {
        insert_free_space = true,
        hit_probability = 0.7,
        miss_probability = 0.4,
      })text");
  return mapping_2d::CreateLaserFanInserterOptions(parameter_dictionary.get());
}

TEST(FastCorrelativeScanMatcherTest, CorrectPose) {
  std::mt19937 prng(42);
  std::uniform_real_distribution<float> distribution(-1.f, 1.f);
  LaserFanInserter laser_fan_inserter(CreateLaserFanInserterTestOptions());
  constexpr float kMinScore = 0.1f;
  const auto options = CreateFastCorrelativeScanMatcherTestOptions(3);

  sensor::PointCloud2D point_cloud;
  point_cloud.emplace_back(-2.5f, 0.5f);
  point_cloud.emplace_back(-2.f, 0.5f);
  point_cloud.emplace_back(0.f, -0.5f);
  point_cloud.emplace_back(0.5f, -1.6f);
  point_cloud.emplace_back(2.5f, 0.5f);
  point_cloud.emplace_back(2.5f, 1.7f);

  for (int i = 0; i != 50; ++i) {
    const transform::Rigid2f expected_pose(
        {2. * distribution(prng), 2. * distribution(prng)},
        0.5 * distribution(prng));

    ProbabilityGrid probability_grid(
        MapLimits(0.05, Eigen::Vector2d(5., 5.), CellLimits(200, 200)));
    probability_grid.StartUpdate();
    laser_fan_inserter.Insert(sensor::LaserFan{expected_pose.translation(),
                                               sensor::TransformPointCloud2D(
                                                   point_cloud, expected_pose),
                                               {}},
                              &probability_grid);

    FastCorrelativeScanMatcher fast_correlative_scan_matcher(probability_grid,
                                                             options);
    transform::Rigid2d pose_estimate;
    kalman_filter::Pose2DCovariance unused_covariance;
    float score;
    EXPECT_TRUE(fast_correlative_scan_matcher.Match(
        transform::Rigid2d::Identity(), point_cloud, kMinScore, &score,
        &pose_estimate, &unused_covariance));
    EXPECT_LT(kMinScore, score);
    EXPECT_THAT(expected_pose,
                transform::IsNearly(pose_estimate.cast<float>(), 0.03f))
        << "Actual: " << transform::ToProto(pose_estimate).DebugString()
        << "\nExpected: " << transform::ToProto(expected_pose).DebugString();
  }
}

TEST(FastCorrelativeScanMatcherTest, FullSubmapMatching) {
  std::mt19937 prng(42);
  std::uniform_real_distribution<float> distribution(-1.f, 1.f);
  LaserFanInserter laser_fan_inserter(CreateLaserFanInserterTestOptions());
  constexpr float kMinScore = 0.1f;
  const auto options = CreateFastCorrelativeScanMatcherTestOptions(6);

  sensor::PointCloud2D unperturbed_point_cloud;
  unperturbed_point_cloud.emplace_back(-2.5, 0.5);
  unperturbed_point_cloud.emplace_back(-2.25, 0.5);
  unperturbed_point_cloud.emplace_back(0., 0.5);
  unperturbed_point_cloud.emplace_back(0.25, 1.6);
  unperturbed_point_cloud.emplace_back(2.5, 0.5);
  unperturbed_point_cloud.emplace_back(2.0, 1.8);

  for (int i = 0; i != 20; ++i) {
    const transform::Rigid2f perturbation(
        {10. * distribution(prng), 10. * distribution(prng)},
        1.6 * distribution(prng));
    const sensor::PointCloud2D point_cloud =
        sensor::TransformPointCloud2D(unperturbed_point_cloud, perturbation);
    const transform::Rigid2f expected_pose =
        transform::Rigid2f({2. * distribution(prng), 2. * distribution(prng)},
                           0.5 * distribution(prng)) *
        perturbation.inverse();

    ProbabilityGrid probability_grid(
        MapLimits(0.05, Eigen::Vector2d(5., 5.), CellLimits(200, 200)));
    probability_grid.StartUpdate();
    laser_fan_inserter.Insert(
        sensor::LaserFan{
            (expected_pose * perturbation).translation(),
            sensor::TransformPointCloud2D(point_cloud, expected_pose),
            {}},
        &probability_grid);

    FastCorrelativeScanMatcher fast_correlative_scan_matcher(probability_grid,
                                                             options);
    transform::Rigid2d pose_estimate;
    kalman_filter::Pose2DCovariance unused_covariance;
    float score;
    EXPECT_TRUE(fast_correlative_scan_matcher.MatchFullSubmap(
        point_cloud, kMinScore, &score, &pose_estimate, &unused_covariance));
    EXPECT_LT(kMinScore, score);
    EXPECT_THAT(expected_pose,
                transform::IsNearly(pose_estimate.cast<float>(), 0.03f))
        << "Actual: " << transform::ToProto(pose_estimate).DebugString()
        << "\nExpected: " << transform::ToProto(expected_pose).DebugString();
  }
}

}  // namespace
}  // namespace scan_matching
}  // namespace mapping_2d
}  // namespace cartographer
