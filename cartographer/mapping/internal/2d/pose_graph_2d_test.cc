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

#include "cartographer/mapping/internal/2d/pose_graph_2d.h"

#include <cmath>
#include <memory>
#include <random>

#include "absl/memory/memory.h"
#include "cartographer/common/lua_parameter_dictionary_test_helpers.h"
#include "cartographer/common/thread_pool.h"
#include "cartographer/common/time.h"
#include "cartographer/mapping/2d/probability_grid_range_data_inserter_2d.h"
#include "cartographer/mapping/2d/submap_2d.h"
#include "cartographer/transform/rigid_transform.h"
#include "cartographer/transform/rigid_transform_test_helpers.h"
#include "cartographer/transform/transform.h"
#include "gmock/gmock.h"

namespace cartographer {
namespace mapping {
namespace {

class PoseGraph2DTest : public ::testing::Test {
 protected:
  PoseGraph2DTest() : thread_pool_(1) {
    // Builds a wavy, irregularly circular point cloud that is unique
    // rotationally. This gives us good rotational texture and avoids any
    // possibility of the CeresScanMatcher2D preferring free space (>
    // kMinProbability) to unknown space (== kMinProbability).
    for (float t = 0.f; t < 2.f * M_PI; t += 0.005f) {
      const float r = (std::sin(20.f * t) + 2.f) * std::sin(t + 2.f);
      point_cloud_.push_back(
          {Eigen::Vector3f{r * std::sin(t), r * std::cos(t), 0.f}});
    }

    {
      auto parameter_dictionary = common::MakeDictionary(R"text(
          return {
            num_range_data = 1,
            grid_options_2d = {
              grid_type = "PROBABILITY_GRID",
              resolution = 0.05,
            },
            range_data_inserter = {
              range_data_inserter_type = "PROBABILITY_GRID_INSERTER_2D",
              probability_grid_range_data_inserter = {
                insert_free_space = true,
                hit_probability = 0.53,
                miss_probability = 0.495,
              },
            tsdf_range_data_inserter = {
              truncation_distance = 0.3,
              maximum_weight = 10.,
              update_free_space = false,
              normal_estimation_options = {
                num_normal_samples = 4,
                sample_radius = 0.5,
              },
              project_sdf_distance_to_scan_normal = false,
              update_weight_range_exponent = 0,
              update_weight_angle_scan_normal_to_ray_kernel_bandwidth = 0,
              update_weight_distance_cell_to_hit_kernel_bandwidth = 0,
            },
          },
        })text");
      active_submaps_ = absl::make_unique<ActiveSubmaps2D>(
          mapping::CreateSubmapsOptions2D(parameter_dictionary.get()));
    }

    {
      auto parameter_dictionary = common::MakeDictionary(R"text(
          return {
            optimize_every_n_nodes = 1000,
            constraint_builder = {
              sampling_ratio = 1.,
              max_constraint_distance = 6.,
              min_score = 0.5,
              global_localization_min_score = 0.6,
              loop_closure_translation_weight = 1.,
              loop_closure_rotation_weight = 1.,
              log_matches = true,
              fast_correlative_scan_matcher = {
                linear_search_window = 3.,
                angular_search_window = 0.1,
                branch_and_bound_depth = 3,
              },
              ceres_scan_matcher = {
                occupied_space_weight = 20.,
                translation_weight = 10.,
                rotation_weight = 1.,
                ceres_solver_options = {
                  use_nonmonotonic_steps = true,
                  max_num_iterations = 50,
                  num_threads = 1,
                },
              },
              fast_correlative_scan_matcher_3d = {
                branch_and_bound_depth = 3,
                full_resolution_depth = 3,
                min_rotational_score = 0.1,
                min_low_resolution_score = 0.5,
                linear_xy_search_window = 4.,
                linear_z_search_window = 4.,
                angular_search_window = 0.1,
              },
              ceres_scan_matcher_3d = {
                occupied_space_weight_0 = 20.,
                translation_weight = 10.,
                rotation_weight = 1.,
                only_optimize_yaw = true,
                ceres_solver_options = {
                  use_nonmonotonic_steps = true,
                  max_num_iterations = 50,
                  num_threads = 1,
                },
              },
            },
            matcher_translation_weight = 1.,
            matcher_rotation_weight = 1.,
            optimization_problem = {
              acceleration_weight = 1.,
              rotation_weight = 1e2,
              huber_scale = 1.,
              local_slam_pose_translation_weight = 0.,
              local_slam_pose_rotation_weight = 0.,
              odometry_translation_weight = 0.,
              odometry_rotation_weight = 0.,
              fixed_frame_pose_translation_weight = 1e1,
              fixed_frame_pose_rotation_weight = 1e2,
              log_solver_summary = true,
              use_online_imu_extrinsics_in_3d = true,
              fix_z_in_3d = false,
              ceres_solver_options = {
                use_nonmonotonic_steps = false,
                max_num_iterations = 200,
                num_threads = 1,
              },
            },
            max_num_final_iterations = 200,
            global_sampling_ratio = 0.01,
            log_residual_histograms = true,
            global_constraint_search_after_n_seconds = 10.0,
          })text");
      auto options = CreatePoseGraphOptions(parameter_dictionary.get());
      pose_graph_ = absl::make_unique<PoseGraph2D>(
          options,
          absl::make_unique<optimization::OptimizationProblem2D>(
              options.optimization_problem_options()),
          &thread_pool_);
    }

    current_pose_ = transform::Rigid2d::Identity();
  }

  void MoveRelativeWithNoise(const transform::Rigid2d& movement,
                             const transform::Rigid2d& noise) {
    current_pose_ = current_pose_ * movement;
    const sensor::PointCloud new_point_cloud = sensor::TransformPointCloud(
        point_cloud_,
        transform::Embed3D(current_pose_.inverse().cast<float>()));
    const sensor::RangeData range_data{
        Eigen::Vector3f::Zero(), new_point_cloud, {}};
    const transform::Rigid2d pose_estimate = noise * current_pose_;
    constexpr int kTrajectoryId = 0;
    active_submaps_->InsertRangeData(TransformRangeData(
        range_data, transform::Embed3D(pose_estimate.cast<float>())));
    std::vector<std::shared_ptr<const Submap2D>> insertion_submaps;
    for (const auto& submap : active_submaps_->submaps()) {
      insertion_submaps.push_back(submap);
    }
    pose_graph_->AddNode(
        std::make_shared<const TrajectoryNode::Data>(
            TrajectoryNode::Data{common::FromUniversal(0),
                                 Eigen::Quaterniond::Identity(),
                                 range_data.returns,
                                 {},
                                 {},
                                 {},
                                 transform::Embed3D(pose_estimate)}),
        kTrajectoryId, insertion_submaps);
  }

  void MoveRelative(const transform::Rigid2d& movement) {
    MoveRelativeWithNoise(movement, transform::Rigid2d::Identity());
  }

  template <typename Range>
  std::vector<int> ToVectorInt(const Range& range) {
    return std::vector<int>(range.begin(), range.end());
  }

  sensor::PointCloud point_cloud_;
  std::unique_ptr<ActiveSubmaps2D> active_submaps_;
  common::ThreadPool thread_pool_;
  std::unique_ptr<PoseGraph2D> pose_graph_;
  transform::Rigid2d current_pose_;
};

TEST_F(PoseGraph2DTest, EmptyMap) {
  pose_graph_->RunFinalOptimization();
  const auto nodes = pose_graph_->GetTrajectoryNodes();
  EXPECT_TRUE(nodes.empty());
}

TEST_F(PoseGraph2DTest, NoMovement) {
  MoveRelative(transform::Rigid2d::Identity());
  MoveRelative(transform::Rigid2d::Identity());
  MoveRelative(transform::Rigid2d::Identity());
  pose_graph_->RunFinalOptimization();
  const auto nodes = pose_graph_->GetTrajectoryNodes();
  ASSERT_THAT(ToVectorInt(nodes.trajectory_ids()),
              ::testing::ContainerEq(std::vector<int>{0}));
  EXPECT_THAT(nodes.SizeOfTrajectoryOrZero(0), ::testing::Eq(3u));
  EXPECT_THAT(nodes.at(NodeId{0, 0}).global_pose,
              transform::IsNearly(transform::Rigid3d::Identity(), 1e-2));
  EXPECT_THAT(nodes.at(NodeId{0, 1}).global_pose,
              transform::IsNearly(transform::Rigid3d::Identity(), 1e-2));
  EXPECT_THAT(nodes.at(NodeId{0, 2}).global_pose,
              transform::IsNearly(transform::Rigid3d::Identity(), 1e-2));
}

TEST_F(PoseGraph2DTest, NoOverlappingNodes) {
  std::mt19937 rng(0);
  std::uniform_real_distribution<double> distribution(-1., 1.);
  std::vector<transform::Rigid2d> poses;
  for (int i = 0; i != 4; ++i) {
    MoveRelative(transform::Rigid2d({0.25 * distribution(rng), 5.}, 0.));
    poses.emplace_back(current_pose_);
  }
  pose_graph_->RunFinalOptimization();
  const auto nodes = pose_graph_->GetTrajectoryNodes();
  ASSERT_THAT(ToVectorInt(nodes.trajectory_ids()),
              ::testing::ContainerEq(std::vector<int>{0}));
  for (int i = 0; i != 4; ++i) {
    EXPECT_THAT(
        poses[i],
        IsNearly(transform::Project2D(nodes.at(NodeId{0, i}).global_pose),
                 1e-2))
        << i;
  }
}

TEST_F(PoseGraph2DTest, ConsecutivelyOverlappingNodes) {
  std::mt19937 rng(0);
  std::uniform_real_distribution<double> distribution(-1., 1.);
  std::vector<transform::Rigid2d> poses;
  for (int i = 0; i != 5; ++i) {
    MoveRelative(transform::Rigid2d({0.25 * distribution(rng), 2.}, 0.));
    poses.emplace_back(current_pose_);
  }
  pose_graph_->RunFinalOptimization();
  const auto nodes = pose_graph_->GetTrajectoryNodes();
  ASSERT_THAT(ToVectorInt(nodes.trajectory_ids()),
              ::testing::ContainerEq(std::vector<int>{0}));
  for (int i = 0; i != 5; ++i) {
    EXPECT_THAT(
        poses[i],
        IsNearly(transform::Project2D(nodes.at(NodeId{0, i}).global_pose),
                 1e-2))
        << i;
  }
}

TEST_F(PoseGraph2DTest, OverlappingNodes) {
  std::mt19937 rng(0);
  std::uniform_real_distribution<double> distribution(-1., 1.);
  std::vector<transform::Rigid2d> ground_truth;
  std::vector<transform::Rigid2d> poses;
  for (int i = 0; i != 5; ++i) {
    const double noise_x = 0.1 * distribution(rng);
    const double noise_y = 0.1 * distribution(rng);
    const double noise_orientation = 0.1 * distribution(rng);
    transform::Rigid2d noise({noise_x, noise_y}, noise_orientation);
    MoveRelativeWithNoise(
        transform::Rigid2d({0.15 * distribution(rng), 0.4}, 0.), noise);
    ground_truth.emplace_back(current_pose_);
    poses.emplace_back(noise * current_pose_);
  }
  pose_graph_->RunFinalOptimization();
  const auto nodes = pose_graph_->GetTrajectoryNodes();
  ASSERT_THAT(ToVectorInt(nodes.trajectory_ids()),
              ::testing::ContainerEq(std::vector<int>{0}));
  transform::Rigid2d true_movement =
      ground_truth.front().inverse() * ground_truth.back();
  transform::Rigid2d movement_before = poses.front().inverse() * poses.back();
  transform::Rigid2d error_before = movement_before.inverse() * true_movement;
  transform::Rigid3d optimized_movement =
      nodes.BeginOfTrajectory(0)->data.global_pose.inverse() *
      std::prev(nodes.EndOfTrajectory(0))->data.global_pose;
  transform::Rigid2d optimized_error =
      transform::Project2D(optimized_movement).inverse() * true_movement;
  EXPECT_THAT(std::abs(optimized_error.normalized_angle()),
              ::testing::Lt(std::abs(error_before.normalized_angle())));
  EXPECT_THAT(optimized_error.translation().norm(),
              ::testing::Lt(error_before.translation().norm()));
}

}  // namespace
}  // namespace mapping
}  // namespace cartographer
