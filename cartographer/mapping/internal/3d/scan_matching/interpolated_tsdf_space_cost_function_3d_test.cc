/*
 * Copyright 2018 The Cartographer Authors
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

#include "cartographer/mapping/internal/3d/scan_matching/interpolated_tsdf_space_cost_function_3d.h"

#include "cartographer/common/ceres_solver_options.h"
#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/common/lua_parameter_dictionary_test_helpers.h"
#include "cartographer/evaluation/scan_cloud_generator.h"
#include "cartographer/mapping/3d/hybrid_grid_tsdf.h"
#include "cartographer/mapping/3d/range_data_inserter_3d.h"
#include "cartographer/mapping/3d/tsdf_range_data_inserter_3d.h"
#include "ceres/ceres.h"
#include "ceres_scan_matcher_3d.h"
#include "gmock/gmock.h"
#include "gtest/gtest.h"

//
// mapping::proto::LocalTrajectoryBuilderOptions3D
// CreateTrajectoryBuilderOptions3D() {
//  auto parameter_dictionary = common::MakeDictionary(R"text(
//        return {
//          min_range = 0.5,
//          max_range = 50.,
//          num_accumulated_range_data = 1,
//          voxel_filter_size = 0.2,
//
//          high_resolution_adaptive_voxel_filter = {
//            max_length = 0.7,
//            min_num_points = 200,
//            max_range = 50.,
//          },
//
//          low_resolution_adaptive_voxel_filter = {
//            max_length = 0.7,
//            min_num_points = 200,
//            max_range = 50.,
//          },
//
//          use_online_correlative_scan_matching = false,
//          real_time_correlative_scan_matcher = {
//            linear_search_window = 0.2,
//            angular_search_window = math.rad(1.),
//            translation_delta_cost_weight = 1e-1,
//            rotation_delta_cost_weight = 1.,
//          },
//
//          ceres_scan_matcher = {
//            occupied_space_weight_0 = 5.,
//            occupied_space_weight_1 = 20.,
//            translation_weight = 0.1,
//            rotation_weight = 0.3,
//            only_optimize_yaw = false,
//            ceres_solver_options = {
//              use_nonmonotonic_steps = true,
//              max_num_iterations = 20,
//              num_threads = 1,
//            },
//          },
//
//          motion_filter = {
//            max_time_seconds = 0.2,
//            max_distance_meters = 0.02,
//            max_angle_radians = 0.001,
//          },
//
//          imu_gravity_time_constant = 1.,
//          rotational_histogram_size = 120,
//
//  optimizing_local_trajectory_builder = {
//      high_resolution_grid_weight = 1,
//      low_resolution_grid_weight = 1,
//      velocity_weight = 1,
//      translation_weight = 1,
//      rotation_weight = 1,
//      odometry_translation_weight = 1,
//      odometry_rotation_weight = 1,
//      scans_per_optimization_update = 1,
//      initialize_map_orientation_with_imu = true,
//      calibrate_imu = false,
//  },
//
//          submaps = {
//            high_resolution = 0.2,
//            high_resolution_max_range = 50.,
//            low_resolution = 0.5,
//            num_range_data = 45000,
//            grid_type = "TSDF",
//            range_data_inserter = {
//              range_data_inserter_type = "TSDF_INSERTER_3D",
//      probability_grid_range_data_inserter = {
//        hit_probability = 0.55,
//        miss_probability = 0.49,
//        num_free_space_voxels = 2,
//      },
//      tsdf_range_data_inserter = {
//        relative_truncation_distance = 4,
//        maximum_weight = 1000.,
//        num_free_space_voxels = 0,
//        project_sdf_distance_to_scan_normal = false,
//      },
//            },
//          },
//        }
//        )text");
//  return mapping::CreateLocalTrajectoryBuilderOptions3D(
//      parameter_dictionary.get());
//}

namespace cartographer {
namespace mapping {
namespace scan_matching {
namespace {

using ::testing::DoubleNear;
using ::testing::ElementsAre;

class TSDFSpaceCostFunction3DTest : public ::testing::Test {
 protected:
  TSDFSpaceCostFunction3DTest() {
    auto parameter_dictionary = common::MakeDictionary(R"text(
        return {
      range_data_inserter_type = "TSDF_INSERTER_3D",
      probability_grid_range_data_inserter = {
        hit_probability = 0.55,
        miss_probability = 0.49,
        num_free_space_voxels = 2,
      },
      tsdf_range_data_inserter = {
        relative_truncation_distance = 4,
        maximum_weight = 1000.,
        num_free_space_voxels = 0,
        project_sdf_distance_to_scan_normal = false,
      },})text");
    options_ = CreateRangeDataInserterOptions3D(parameter_dictionary.get());
    range_data_inserter_ = absl::make_unique<TSDFRangeDataInserter3D>(options_);
    tsdf_ = absl::make_unique<HybridGridTSDF>(
        0.05,
        options_.tsdf_range_data_inserter_options_3d()
            .relative_truncation_distance(),
        options_.tsdf_range_data_inserter_options_3d().maximum_weight(),
        &conversion_tables_);
  }

  void InsertPointcloud() {
    auto range_data = sensor::RangeData();
    evaluation::ScanCloudGenerator generator =
        evaluation::ScanCloudGenerator(0.025);
    generator.generateCube(range_data.returns, 1.f, 0.f);
    range_data_inserter_->Insert(range_data, tsdf_.get());
    tsdf_->FinishUpdate();
  }

  ValueConversionTables conversion_tables_;
  ::cartographer::mapping::proto::RangeDataInserterOptions3D options_;
  std::unique_ptr<HybridGridTSDF> tsdf_;
  std::unique_ptr<TSDFRangeDataInserter3D> range_data_inserter_;
};

struct State {
  std::array<double, 3> translation;
  std::array<double, 4> rotation;  // Rotation quaternion as (w, x, y, z).
  std::array<double, 3> velocity;

  State(const Eigen::Vector3d& translation, const Eigen::Quaterniond& rotation,
        const Eigen::Vector3d& velocity)
      : translation{{translation.x(), translation.y(), translation.z()}},
        rotation{{rotation.w(), rotation.x(), rotation.y(), rotation.z()}},
        velocity{{velocity.x(), velocity.y(), velocity.z()}} {}

  Eigen::Quaterniond ToQuaternion() const {
    return Eigen::Quaterniond(rotation[0], rotation[1], rotation[2],
                              rotation[3]);
  }

  transform::Rigid3d ToRigid() const {
    return transform::Rigid3d(
        Eigen::Vector3d(translation[0], translation[1], translation[2]),
        ToQuaternion());
  }
};

TEST_F(TSDFSpaceCostFunction3DTest, MatchEmptyTSDF) {
  sensor::PointCloud matching_cloud = sensor::PointCloud();
  evaluation::ScanCloudGenerator generator =
      evaluation::ScanCloudGenerator(0.025);
  generator.generateCube(matching_cloud, 1.f, 0.f);
  std::unique_ptr<ceres::CostFunction> cost_function(
      InterpolatedTSDFSpaceCostFunction3D::CreateAutoDiffCostFunction(
          1.0, matching_cloud, *tsdf_, 0.5));
  State state_t0 =
      State(Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity(),
            Eigen::Vector3d::Zero());
  State state_t1 = State({0.0, 0.0, 0.025}, Eigen::Quaterniond::Identity(),
                         Eigen::Vector3d::Zero());
  State state_t0_res = state_t0;
  State state_t1_res = state_t1;

  ceres::Problem problem;
  problem.AddResidualBlock(
      scan_matching::InterpolatedTSDFSpaceCostFunction3D::
          CreateAutoDiffCostFunction(1.0, matching_cloud, *tsdf_, 0.5),
      nullptr, state_t0.translation.data(), state_t0.rotation.data(),
      state_t1.translation.data(), state_t1.rotation.data());

  auto parameter_dictionary_scan_matcher = common::MakeDictionary(R"text(
    return {
      occupied_space_weight_0 = 1.,
      translation_weight = 0.,
      rotation_weight = 0.,
      only_optimize_yaw = false,
      ceres_solver_options = {
        use_nonmonotonic_steps = false,
        max_num_iterations = 120,
        num_threads = 1,
      },
    })text");
  auto scan_matcher_options =
      cartographer::mapping::scan_matching::CreateCeresScanMatcherOptions3D(
          parameter_dictionary_scan_matcher.get());
  const ceres::Solver::Options ceres_solver_options_(
      common::CreateCeresSolverOptions(
          scan_matcher_options.ceres_solver_options()));
  ceres::Solver::Summary summary;
  ceres::Solve(ceres_solver_options_, &problem, &summary);

  EXPECT_EQ(summary.iterations.size(), 1);
  EXPECT_EQ(state_t0.translation, state_t0_res.translation);
  EXPECT_EQ(state_t0.rotation, state_t0_res.rotation);
  EXPECT_EQ(state_t1.translation, state_t1_res.translation);
  EXPECT_EQ(state_t1.rotation, state_t1_res.rotation);
}

TEST_F(TSDFSpaceCostFunction3DTest, SmallPertubation) {
  InsertPointcloud();
  auto matching_cloud = sensor::PointCloud();
  evaluation::ScanCloudGenerator generator =
      evaluation::ScanCloudGenerator(0.025);
  generator.generateCube(matching_cloud, 1.f, 0.f);

  State state_t0 = State({0.0, 0.0, 0.0}, Eigen::Quaterniond::Identity(),
                         Eigen::Vector3d::Zero());

  State state_t0_expected = state_t0;
  State state_t1 = State({0.02, 0.02, 0.02}, Eigen::Quaterniond::Identity(),
                         Eigen::Vector3d::Zero());
  State state_t1_expected = state_t0;

  ceres::Problem problem;
  problem.AddResidualBlock(
      scan_matching::InterpolatedTSDFSpaceCostFunction3D::
          CreateAutoDiffCostFunction(1.0, matching_cloud, *tsdf_, 0.5),
      nullptr, state_t0.translation.data(), state_t0.rotation.data(),
      state_t1.translation.data(), state_t1.rotation.data());

  problem.SetParameterBlockConstant(state_t0.rotation.data());
  problem.SetParameterBlockConstant(state_t0.translation.data());
  problem.SetParameterization(state_t1.rotation.data(),
                              new ceres::QuaternionParameterization());

  auto parameter_dictionary_scan_matcher = common::MakeDictionary(R"text(
    return {
      occupied_space_weight_0 = 1.,
      translation_weight = 0.,
      rotation_weight = 0.,
      only_optimize_yaw = false,
      ceres_solver_options = {
        use_nonmonotonic_steps = false,
        max_num_iterations = 120,
        num_threads = 1,
      },
    })text");
  auto scan_matcher_options =
      cartographer::mapping::scan_matching::CreateCeresScanMatcherOptions3D(
          parameter_dictionary_scan_matcher.get());
  const ceres::Solver::Options ceres_solver_options_(
      common::CreateCeresSolverOptions(
          scan_matcher_options.ceres_solver_options()));
  ceres::Solver::Summary summary;
  ceres::Solve(ceres_solver_options_, &problem, &summary);
  EXPECT_GT(summary.iterations.size(), 1);
  EXPECT_EQ(state_t0.translation, state_t0_expected.translation);
  EXPECT_EQ(state_t0.rotation, state_t0_expected.rotation);
  for (int i = 0; i < 3; ++i) {
    EXPECT_NEAR(state_t1.translation[i], state_t1_expected.translation[i],
                1E-4);
  }
  for (int i = 0; i < 4; ++i) {
    EXPECT_NEAR(state_t1.rotation[i], state_t1_expected.rotation[i], 5E-3);
  }
}

}  // namespace
}  // namespace scan_matching
}  // namespace mapping
}  // namespace cartographer
