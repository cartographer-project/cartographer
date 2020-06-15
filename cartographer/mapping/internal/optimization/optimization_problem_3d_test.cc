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

#include "cartographer/mapping/internal/optimization/optimization_problem_3d.h"

#include <random>

#include "Eigen/Core"
#include "cartographer/common/lua_parameter_dictionary_test_helpers.h"
#include "cartographer/common/time.h"
#include "cartographer/mapping/internal/optimization/optimization_problem_options.h"
#include "cartographer/transform/transform.h"
#include "glog/logging.h"
#include "gmock/gmock.h"

namespace cartographer {
namespace mapping {
namespace optimization {
namespace {

class OptimizationProblem3DTest : public ::testing::Test {
 protected:
  OptimizationProblem3DTest()
      : optimization_problem_(CreateOptions()), rng_(45387) {}

  optimization::proto::OptimizationProblemOptions CreateOptions() {
    auto parameter_dictionary = common::MakeDictionary(R"text(
        return {
          acceleration_weight = 1e-4,
          rotation_weight = 1e-2,
          huber_scale = 1.,
          local_slam_pose_translation_weight = 1e-2,
          local_slam_pose_rotation_weight = 1e-2,
          odometry_translation_weight = 1e-2,
          odometry_rotation_weight = 1e-2,
          fixed_frame_pose_translation_weight = 1e1,
          fixed_frame_pose_rotation_weight = 1e2,
          fixed_frame_pose_use_tolerant_loss = false,
          fixed_frame_pose_tolerant_loss_param_a = 1,
          fixed_frame_pose_tolerant_loss_param_b = 1,
          log_solver_summary = true,
          use_online_imu_extrinsics_in_3d = true,
          fix_z_in_3d = false,
          ceres_solver_options = {
            use_nonmonotonic_steps = false,
            max_num_iterations = 200,
            num_threads = 4,
          },
        })text");
    return optimization::CreateOptimizationProblemOptions(
        parameter_dictionary.get());
  }

  transform::Rigid3d RandomTransform(double translation_size,
                                     double rotation_size) {
    std::uniform_real_distribution<double> translation_distribution(
        -translation_size, translation_size);
    const double x = translation_distribution(rng_);
    const double y = translation_distribution(rng_);
    const double z = translation_distribution(rng_);
    std::uniform_real_distribution<double> rotation_distribution(-rotation_size,
                                                                 rotation_size);
    const double rx = rotation_distribution(rng_);
    const double ry = rotation_distribution(rng_);
    const double rz = rotation_distribution(rng_);
    return transform::Rigid3d(Eigen::Vector3d(x, y, z),
                              transform::AngleAxisVectorToRotationQuaternion(
                                  Eigen::Vector3d(rx, ry, rz)));
  }

  transform::Rigid3d RandomYawOnlyTransform(double translation_size,
                                            double rotation_size) {
    std::uniform_real_distribution<double> translation_distribution(
        -translation_size, translation_size);
    const double x = translation_distribution(rng_);
    const double y = translation_distribution(rng_);
    const double z = translation_distribution(rng_);
    std::uniform_real_distribution<double> rotation_distribution(-rotation_size,
                                                                 rotation_size);
    const double rz = rotation_distribution(rng_);
    return transform::Rigid3d(Eigen::Vector3d(x, y, z),
                              transform::AngleAxisVectorToRotationQuaternion(
                                  Eigen::Vector3d(0., 0., rz)));
  }

  OptimizationProblem3D optimization_problem_;
  std::mt19937 rng_;
};

transform::Rigid3d AddNoise(const transform::Rigid3d& transform,
                            const transform::Rigid3d& noise) {
  const Eigen::Quaterniond noisy_rotation(noise.rotation() *
                                          transform.rotation());
  return transform::Rigid3d(transform.translation() + noise.translation(),
                            noisy_rotation);
}

TEST_F(OptimizationProblem3DTest, ReducesNoise) {
  constexpr int kNumNodes = 100;
  const transform::Rigid3d kSubmap0Transform = transform::Rigid3d::Identity();
  const transform::Rigid3d kSubmap2Transform = transform::Rigid3d::Rotation(
      Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ()));
  const int kTrajectoryId = 0;

  struct NoisyNode {
    transform::Rigid3d ground_truth_pose;
    transform::Rigid3d noise;
  };
  std::vector<NoisyNode> test_data;
  for (int j = 0; j != kNumNodes; ++j) {
    test_data.push_back(
        NoisyNode{RandomTransform(10., 3.), RandomYawOnlyTransform(0.2, 0.3)});
  }

  common::Time now = common::FromUniversal(0);
  for (const NoisyNode& node : test_data) {
    const transform::Rigid3d pose =
        AddNoise(node.ground_truth_pose, node.noise);
    optimization_problem_.AddImuData(
        kTrajectoryId, sensor::ImuData{now, Eigen::Vector3d::UnitZ() * 9.81,
                                       Eigen::Vector3d::Zero()});
    optimization_problem_.AddTrajectoryNode(kTrajectoryId,
                                            NodeSpec3D{now, pose, pose});
    now += common::FromSeconds(0.01);
  }

  std::vector<OptimizationProblem3D::Constraint> constraints;
  for (int j = 0; j != kNumNodes; ++j) {
    constraints.push_back(OptimizationProblem3D::Constraint{
        SubmapId{kTrajectoryId, 0}, NodeId{kTrajectoryId, j},
        OptimizationProblem3D::Constraint::Pose{
            AddNoise(test_data[j].ground_truth_pose, test_data[j].noise), 1.,
            1.}});
    // We add an additional independent, but equally noisy observation.
    constraints.push_back(OptimizationProblem3D::Constraint{
        SubmapId{kTrajectoryId, 1}, NodeId{kTrajectoryId, j},
        OptimizationProblem3D::Constraint::Pose{
            AddNoise(test_data[j].ground_truth_pose,
                     RandomYawOnlyTransform(0.2, 0.3)),
            1., 1.}});
    // We add very noisy data with a low weight to verify it is mostly ignored.
    constraints.push_back(OptimizationProblem3D::Constraint{
        SubmapId{kTrajectoryId, 2}, NodeId{kTrajectoryId, j},
        OptimizationProblem3D::Constraint::Pose{
            kSubmap2Transform.inverse() * test_data[j].ground_truth_pose *
                RandomTransform(1e3, 3.),
            1e-9, 1e-9}});
  }

  double translation_error_before = 0.;
  double rotation_error_before = 0.;
  const auto& node_data = optimization_problem_.node_data();
  for (int j = 0; j != kNumNodes; ++j) {
    translation_error_before +=
        (test_data[j].ground_truth_pose.translation() -
         node_data.at(NodeId{kTrajectoryId, j}).global_pose.translation())
            .norm();
    rotation_error_before +=
        transform::GetAngle(test_data[j].ground_truth_pose.inverse() *
                            node_data.at(NodeId{kTrajectoryId, j}).global_pose);
  }

  optimization_problem_.AddSubmap(kTrajectoryId, kSubmap0Transform);
  optimization_problem_.AddSubmap(kTrajectoryId, kSubmap0Transform);
  optimization_problem_.AddSubmap(kTrajectoryId, kSubmap2Transform);
  const std::map<int, PoseGraphInterface::TrajectoryState> kTrajectoriesState =
      {{kTrajectoryId, PoseGraphInterface::TrajectoryState::ACTIVE}};
  optimization_problem_.Solve(constraints, kTrajectoriesState, {});

  double translation_error_after = 0.;
  double rotation_error_after = 0.;
  for (int j = 0; j != kNumNodes; ++j) {
    translation_error_after +=
        (test_data[j].ground_truth_pose.translation() -
         node_data.at(NodeId{kTrajectoryId, j}).global_pose.translation())
            .norm();
    rotation_error_after +=
        transform::GetAngle(test_data[j].ground_truth_pose.inverse() *
                            node_data.at(NodeId{kTrajectoryId, j}).global_pose);
  }

  EXPECT_GT(0.8 * translation_error_before, translation_error_after);
  EXPECT_GT(0.8 * rotation_error_before, rotation_error_after);
}

}  // namespace
}  // namespace optimization
}  // namespace mapping
}  // namespace cartographer
