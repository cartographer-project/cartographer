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

#include "cartographer/mapping/internal/3d/local_trajectory_builder_3d.h"

#include <memory>
#include <random>

#include "Eigen/Core"
#include "cartographer/common/internal/testing/lua_parameter_dictionary_test_helpers.h"
#include "cartographer/common/time.h"
#include "cartographer/mapping/3d/hybrid_grid.h"
#include "cartographer/mapping/internal/3d/local_trajectory_builder_options_3d.h"
#include "cartographer/sensor/range_data.h"
#include "cartographer/transform/rigid_transform.h"
#include "cartographer/transform/rigid_transform_test_helpers.h"
#include "cartographer/transform/transform.h"
#include "glog/logging.h"
#include "gmock/gmock.h"

namespace cartographer {
namespace mapping {
namespace {

constexpr char kSensorId[] = "sensor_id";

class LocalTrajectoryBuilderTest : public ::testing::Test {
 protected:
  struct TrajectoryNode {
    common::Time time;
    transform::Rigid3d pose;
  };

  void SetUp() override { GenerateBubbles(); }

  mapping::proto::LocalTrajectoryBuilderOptions3D
  CreateTrajectoryBuilderOptions3D() {
    auto parameter_dictionary = common::MakeDictionary(R"text(
        return {
          min_range = 0.5,
          max_range = 50.,
          num_accumulated_range_data = 1,
          voxel_filter_size = 0.2,

          high_resolution_adaptive_voxel_filter = {
            max_length = 0.7,
            min_num_points = 200,
            max_range = 50.,
          },

          low_resolution_adaptive_voxel_filter = {
            max_length = 0.7,
            min_num_points = 200,
            max_range = 50.,
          },

          use_online_correlative_scan_matching = false,
          real_time_correlative_scan_matcher = {
            linear_search_window = 0.2,
            angular_search_window = math.rad(1.),
            translation_delta_cost_weight = 1e-1,
            rotation_delta_cost_weight = 1.,
          },

          ceres_scan_matcher = {
            occupied_space_weight_0 = 5.,
            occupied_space_weight_1 = 20.,
            translation_weight = 0.1,
            rotation_weight = 0.3,
            only_optimize_yaw = false,
            ceres_solver_options = {
              use_nonmonotonic_steps = true,
              max_num_iterations = 20,
              num_threads = 1,
            },
          },

          motion_filter = {
            max_time_seconds = 0.2,
            max_distance_meters = 0.02,
            max_angle_radians = 0.001,
          },

          imu_gravity_time_constant = 1.,
          rotational_histogram_size = 120,
          
          pose_extrapolator = {
            use_imu_based = false,
            constant_velocity = {
              imu_gravity_time_constant = 10.,
              pose_queue_duration = 0.001,
            },
            imu_based = {
              pose_queue_duration = 5.,
              gravity_constant = 9.806,
              pose_translation_weight = 1.,
              pose_rotation_weight = 1.,
              imu_acceleration_weight = 1.,
              imu_rotation_weight = 1.,
              odometry_translation_weight = 1.,
              odometry_rotation_weight = 1.,
              solver_options = {
                use_nonmonotonic_steps = false;
                max_num_iterations = 10;
                num_threads = 1;
              },
            },
          },
          
          submaps = {
            high_resolution = 0.2,
            high_resolution_max_range = 50.,
            low_resolution = 0.5,
            num_range_data = 45000,
            range_data_inserter = {
              hit_probability = 0.7,
              miss_probability = 0.4,
              num_free_space_voxels = 0,
              intensity_threshold = 100.0,
            },
          },

          use_intensities = false,
        }
        )text");
    return mapping::CreateLocalTrajectoryBuilderOptions3D(
        parameter_dictionary.get());
  }

  void GenerateBubbles() {
    std::mt19937 prng(42);
    std::uniform_real_distribution<float> distribution(-1.f, 1.f);

    for (int i = 0; i != 100; ++i) {
      const float x = distribution(prng);
      const float y = distribution(prng);
      const float z = distribution(prng);
      bubbles_.push_back(10. * Eigen::Vector3f(x, y, z).normalized());
    }
  }

  // Computes the earliest intersection of the ray 'from' to 'to' with the
  // axis-aligned cube with edge length 30 and center at the origin. Assumes
  // that 'from' is inside the cube.
  Eigen::Vector3f CollideWithBox(const Eigen::Vector3f& from,
                                 const Eigen::Vector3f& to) {
    float first = 100.f;
    if (to.x() > from.x()) {
      first = std::min(first, (15.f - from.x()) / (to.x() - from.x()));
    } else if (to.x() < from.x()) {
      first = std::min(first, (-15.f - from.x()) / (to.x() - from.x()));
    }
    if (to.y() > from.y()) {
      first = std::min(first, (15.f - from.y()) / (to.y() - from.y()));
    } else if (to.y() < from.y()) {
      first = std::min(first, (-15.f - from.y()) / (to.y() - from.y()));
    }
    if (to.z() > from.z()) {
      first = std::min(first, (15.f - from.z()) / (to.z() - from.z()));
    } else if (to.z() < from.z()) {
      first = std::min(first, (-15.f - from.z()) / (to.z() - from.z()));
    }
    return first * (to - from) + from;
  }

  // Computes the earliest intersection of the ray 'from' to 'to' with all
  // bubbles. Returns 'to' if no intersection exists.
  Eigen::Vector3f CollideWithBubbles(const Eigen::Vector3f& from,
                                     const Eigen::Vector3f& to) {
    float first = 1.f;
    constexpr float kBubbleRadius = 0.5f;
    for (const Eigen::Vector3f& center : bubbles_) {
      const float a = (to - from).squaredNorm();
      const float beta = (to - from).dot(from - center);
      const float c =
          (from - center).squaredNorm() - kBubbleRadius * kBubbleRadius;
      const float discriminant = beta * beta - a * c;
      if (discriminant < 0.f) {
        continue;
      }
      const float solution = (-beta - std::sqrt(discriminant)) / a;
      if (solution < 0.f) {
        continue;
      }
      first = std::min(first, solution);
    }
    return first * (to - from) + from;
  }

  sensor::TimedPointCloudData GeneratePointCloudData(
      const transform::Rigid3d& pose, const common::Time time) {
    // 360 degree rays at 16 angles.
    sensor::TimedPointCloud directions_in_rangefinder_frame;
    for (int r = -8; r != 8; ++r) {
      for (int s = -250; s != 250; ++s) {
        const sensor::TimedRangefinderPoint first_point{
            Eigen::Vector3f{
                Eigen::AngleAxisf(M_PI * s / 250., Eigen::Vector3f::UnitZ()) *
                Eigen::AngleAxisf(M_PI / 12. * r / 8.,
                                  Eigen::Vector3f::UnitY()) *
                Eigen::Vector3f::UnitX()},
            0.};
        directions_in_rangefinder_frame.push_back(first_point);
        // Second orthogonal rangefinder.
        const sensor::TimedRangefinderPoint second_point{
            Eigen::Vector3f{
                Eigen::AngleAxisf(M_PI / 2., Eigen::Vector3f::UnitX()) *
                Eigen::AngleAxisf(M_PI * s / 250., Eigen::Vector3f::UnitZ()) *
                Eigen::AngleAxisf(M_PI / 12. * r / 8.,
                                  Eigen::Vector3f::UnitY()) *
                Eigen::Vector3f::UnitX()},
            0.};
        directions_in_rangefinder_frame.push_back(second_point);
      }
    }
    // We simulate a 30 m edge length box around the origin, also containing
    // 50 cm radius spheres.
    sensor::TimedPointCloud returns_in_world_frame;
    for (const auto& direction_in_world_frame :
         sensor::TransformTimedPointCloud(directions_in_rangefinder_frame,
                                          pose.cast<float>())) {
      const Eigen::Vector3f origin =
          pose.cast<float>() * Eigen::Vector3f::Zero();
      const sensor::TimedRangefinderPoint return_point{
          CollideWithBubbles(
              origin,
              CollideWithBox(origin, direction_in_world_frame.position)),
          0.};
      returns_in_world_frame.push_back(return_point);
    }
    return {time, Eigen::Vector3f::Zero(),
            sensor::TransformTimedPointCloud(returns_in_world_frame,
                                             pose.inverse().cast<float>())};
  }

  void AddLinearOnlyImuObservation(const common::Time time,
                                   const transform::Rigid3d& expected_pose) {
    const Eigen::Vector3d gravity =
        expected_pose.rotation().inverse() * Eigen::Vector3d(0., 0., 9.81);
    local_trajectory_builder_->AddImuData(
        sensor::ImuData{time, gravity, Eigen::Vector3d::Zero()});
  }

  std::vector<TrajectoryNode> GenerateCorkscrewTrajectory() {
    std::vector<TrajectoryNode> trajectory;
    common::Time current_time = common::FromUniversal(12345678);
    // One second at zero velocity.
    for (int i = 0; i != 5; ++i) {
      current_time += common::FromSeconds(0.3);
      trajectory.push_back(
          TrajectoryNode{current_time, transform::Rigid3d::Identity()});
    }
    // Corkscrew translation and constant velocity rotation.
    for (double t = 0.; t <= 0.6; t += 0.05) {
      current_time += common::FromSeconds(0.3);
      trajectory.push_back(TrajectoryNode{
          current_time,
          transform::Rigid3d::Translation(Eigen::Vector3d(
              std::sin(4. * t), 1. - std::cos(4. * t), 1. * t)) *
              transform::Rigid3d::Rotation(Eigen::AngleAxisd(
                  0.3 * t, Eigen::Vector3d(1., -1., 2.).normalized()))});
    }
    return trajectory;
  }

  void VerifyAccuracy(const std::vector<TrajectoryNode>& expected_trajectory,
                      double expected_accuracy) {
    int num_poses = 0;
    for (const TrajectoryNode& node : expected_trajectory) {
      AddLinearOnlyImuObservation(node.time, node.pose);
      const auto point_cloud = GeneratePointCloudData(node.pose, node.time);
      const std::unique_ptr<LocalTrajectoryBuilder3D::MatchingResult>
          matching_result =
              local_trajectory_builder_->AddRangeData(kSensorId, point_cloud);
      if (matching_result != nullptr) {
        EXPECT_THAT(matching_result->local_pose,
                    transform::IsNearly(node.pose, 1e-1));
        ++num_poses;
        LOG(INFO) << "num_poses: " << num_poses;
      }
    }
  }

  std::unique_ptr<LocalTrajectoryBuilder3D> local_trajectory_builder_;
  std::vector<Eigen::Vector3f> bubbles_;
};

TEST_F(LocalTrajectoryBuilderTest, MoveInsideCubeUsingOnlyCeresScanMatcher) {
  local_trajectory_builder_.reset(new LocalTrajectoryBuilder3D(
      CreateTrajectoryBuilderOptions3D(), {kSensorId}));
  VerifyAccuracy(GenerateCorkscrewTrajectory(), 1e-1);
}

}  // namespace
}  // namespace mapping
}  // namespace cartographer
