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

#include "cartographer/mapping_2d/sparse_pose_graph.h"

#include <cmath>
#include <memory>
#include <random>

#include "cartographer/common/lua_parameter_dictionary_test_helpers.h"
#include "cartographer/common/make_unique.h"
#include "cartographer/common/thread_pool.h"
#include "cartographer/common/time.h"
#include "cartographer/mapping_2d/laser_fan_inserter.h"
#include "cartographer/mapping_2d/submaps.h"
#include "cartographer/transform/rigid_transform.h"
#include "cartographer/transform/rigid_transform_test_helpers.h"
#include "cartographer/transform/transform.h"
#include "gmock/gmock.h"

namespace cartographer {
namespace mapping_2d {
namespace {

class SparsePoseGraphTest : public ::testing::Test {
 protected:
  SparsePoseGraphTest() : thread_pool_(1) {
    // Builds a wavy, irregularly circular point cloud that is unique
    // rotationally. This gives us good rotational texture and avoids any
    // possibility of the CeresScanMatcher preferring free space (>
    // kMinProbability) to unknown space (== kMinProbability).
    for (float t = 0.f; t < 2.f * M_PI; t += 0.005f) {
      const float r = (std::sin(20.f * t) + 2.f) * std::sin(t + 2.f);
      point_cloud_.emplace_back(r * std::sin(t), r * std::cos(t));
    }

    {
      auto parameter_dictionary = common::MakeDictionary(R"text(
          return {
            resolution = 0.05,
            half_length = 21.,
            num_laser_fans = 1,
            output_debug_images = false,
            laser_fan_inserter = {
              insert_free_space = true,
              hit_probability = 0.53,
              miss_probability = 0.495,
            },
          })text");
      submaps_ = common::make_unique<Submaps>(
          CreateSubmapsOptions(parameter_dictionary.get()));
    }

    {
      auto parameter_dictionary = common::MakeDictionary(R"text(
          return {
            optimize_every_n_scans = 1000,
            constraint_builder = {
              sampling_ratio = 1.,
              max_constraint_distance = 6.,
              adaptive_voxel_filter = {
                max_length = 1e-5,
                min_num_points = 1,
                max_range = 50.,
              },
              min_score = 0.5,
              global_localization_min_score = 0.6,
              lower_covariance_eigenvalue_bound = 1e-6,
              log_matches = true,
              fast_correlative_scan_matcher = {
                linear_search_window = 3.,
                angular_search_window = 0.1,
                branch_and_bound_depth = 3,
              },
              ceres_scan_matcher = {
                occupied_space_cost_functor_weight = 20.,
                previous_pose_translation_delta_cost_functor_weight = 10.,
                initial_pose_estimate_rotation_delta_cost_functor_weight = 1.,
                covariance_scale = 1.,
                ceres_solver_options = {
                  use_nonmonotonic_steps = true,
                  max_num_iterations = 50,
                  num_threads = 1,
                },
              },
              fast_correlative_scan_matcher_3d = {
                branch_and_bound_depth = 3,
                full_resolution_depth = 3,
                rotational_histogram_size = 30,
                min_rotational_score = 0.1,
                linear_xy_search_window = 4.,
                linear_z_search_window = 4.,
                angular_search_window = 0.1,
              },
              ceres_scan_matcher_3d = {
                occupied_space_cost_functor_weight_0 = 20.,
                previous_pose_translation_delta_cost_functor_weight = 10.,
                initial_pose_estimate_rotation_delta_cost_functor_weight = 1.,
                covariance_scale = 1.,
                only_optimize_yaw = true,
                ceres_solver_options = {
                  use_nonmonotonic_steps = true,
                  max_num_iterations = 50,
                  num_threads = 1,
                },
              },
            },
            optimization_problem = {
              acceleration_scale = 1.,
              rotation_scale = 1e2,
              huber_scale = 1.,
              consecutive_scan_translation_penalty_factor = 0.,
              consecutive_scan_rotation_penalty_factor = 0.,
              log_solver_summary = true,
              log_residual_histograms = true,
              ceres_solver_options = {
                use_nonmonotonic_steps = false,
                max_num_iterations = 200,
                num_threads = 1,
              },
            },
            max_num_final_iterations = 200,
            global_sampling_ratio = 0.01,
          })text");
      sparse_pose_graph_ = common::make_unique<SparsePoseGraph>(
          mapping::CreateSparsePoseGraphOptions(parameter_dictionary.get()),
          &thread_pool_, &constant_node_data_);
    }

    current_pose_ = transform::Rigid2d::Identity();
  }

  void MoveRelativeWithNoise(const transform::Rigid2d& movement,
                             const transform::Rigid2d& noise) {
    current_pose_ = current_pose_ * movement;
    const sensor::PointCloud2D new_point_cloud = sensor::TransformPointCloud2D(
        point_cloud_, current_pose_.inverse().cast<float>());
    kalman_filter::Pose2DCovariance covariance =
        kalman_filter::Pose2DCovariance::Identity();
    const mapping::Submap* const matching_submap =
        submaps_->Get(submaps_->matching_index());
    std::vector<const mapping::Submap*> insertion_submaps;
    for (int insertion_index : submaps_->insertion_indices()) {
      insertion_submaps.push_back(submaps_->Get(insertion_index));
    }
    const sensor::LaserFan laser_fan{
        Eigen::Vector2f::Zero(), new_point_cloud, {}};
    const transform::Rigid2d pose_estimate = noise * current_pose_;
    submaps_->InsertLaserFan(
        TransformLaserFan(laser_fan, pose_estimate.cast<float>()));
    sparse_pose_graph_->AddScan(common::FromUniversal(0),
                                transform::Rigid3d::Identity(), laser_fan,
                                pose_estimate, covariance, submaps_.get(),
                                matching_submap, insertion_submaps);
  }

  void MoveRelative(const transform::Rigid2d& movement) {
    MoveRelativeWithNoise(movement, transform::Rigid2d::Identity());
  }

  sensor::PointCloud2D point_cloud_;
  std::unique_ptr<Submaps> submaps_;
  std::deque<mapping::TrajectoryNode::ConstantData> constant_node_data_;
  common::ThreadPool thread_pool_;
  std::unique_ptr<SparsePoseGraph> sparse_pose_graph_;
  transform::Rigid2d current_pose_;
};

TEST_F(SparsePoseGraphTest, EmptyMap) {
  sparse_pose_graph_->RunFinalOptimization();
  const auto nodes = sparse_pose_graph_->GetTrajectoryNodes();
  EXPECT_THAT(nodes.size(), ::testing::Eq(0));
}

TEST_F(SparsePoseGraphTest, NoMovement) {
  MoveRelative(transform::Rigid2d::Identity());
  MoveRelative(transform::Rigid2d::Identity());
  MoveRelative(transform::Rigid2d::Identity());
  sparse_pose_graph_->RunFinalOptimization();
  const auto nodes = sparse_pose_graph_->GetTrajectoryNodes();
  EXPECT_THAT(nodes.size(), ::testing::Eq(3));
  EXPECT_THAT(nodes[0].pose,
              transform::IsNearly(transform::Rigid3d::Identity(), 1e-2));
  EXPECT_THAT(nodes[1].pose,
              transform::IsNearly(transform::Rigid3d::Identity(), 1e-2));
  EXPECT_THAT(nodes[2].pose,
              transform::IsNearly(transform::Rigid3d::Identity(), 1e-2));
}

TEST_F(SparsePoseGraphTest, NoOverlappingScans) {
  std::mt19937 rng(0);
  std::uniform_real_distribution<double> distribution(-1., 1.);
  std::vector<transform::Rigid2d> poses;
  for (int i = 0; i != 4; ++i) {
    MoveRelative(transform::Rigid2d({0.25 * distribution(rng), 5.}, 0.));
    poses.emplace_back(current_pose_);
  }
  sparse_pose_graph_->RunFinalOptimization();
  const auto nodes = sparse_pose_graph_->GetTrajectoryNodes();
  for (int i = 0; i != 4; ++i) {
    EXPECT_THAT(poses[i], IsNearly(transform::Project2D(nodes[i].pose), 1e-2))
        << i;
  }
}

TEST_F(SparsePoseGraphTest, ConsecutivelyOverlappingScans) {
  std::mt19937 rng(0);
  std::uniform_real_distribution<double> distribution(-1., 1.);
  std::vector<transform::Rigid2d> poses;
  for (int i = 0; i != 5; ++i) {
    MoveRelative(transform::Rigid2d({0.25 * distribution(rng), 2.}, 0.));
    poses.emplace_back(current_pose_);
  }
  sparse_pose_graph_->RunFinalOptimization();
  const auto nodes = sparse_pose_graph_->GetTrajectoryNodes();
  for (int i = 0; i != 5; ++i) {
    EXPECT_THAT(poses[i], IsNearly(transform::Project2D(nodes[i].pose), 1e-2))
        << i;
  }
}

TEST_F(SparsePoseGraphTest, OverlappingScans) {
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
  sparse_pose_graph_->RunFinalOptimization();
  const auto nodes = sparse_pose_graph_->GetTrajectoryNodes();
  transform::Rigid2d true_movement =
      ground_truth.front().inverse() * ground_truth.back();
  transform::Rigid2d movement_before = poses.front().inverse() * poses.back();
  transform::Rigid2d error_before = movement_before.inverse() * true_movement;
  transform::Rigid3d optimized_movement =
      nodes.front().pose.inverse() * nodes.back().pose;
  transform::Rigid2d optimized_error =
      transform::Project2D(optimized_movement).inverse() * true_movement;
  EXPECT_THAT(std::abs(optimized_error.normalized_angle()),
              ::testing::Lt(std::abs(error_before.normalized_angle())));
  EXPECT_THAT(optimized_error.translation().norm(),
              ::testing::Lt(error_before.translation().norm()));
}

}  // namespace
}  // namespace mapping_2d
}  // namespace cartographer
