#include "cartographer/mapping/internal/3d/local_trajectory_builder_3d.h"

#include <memory>
#include <random>

#include "matplotlibcpp.h"

#include "Eigen/Core"
#include "cartographer/common/lua_parameter_dictionary_test_helpers.h"
#include "cartographer/common/time.h"
#include "cartographer/evaluation/grid_drawer.h"
#include "cartographer/evaluation/simulation/range_sensor.h"
#include "cartographer/evaluation/simulation/scene.h"
#include "cartographer/mapping/3d/hybrid_grid.h"
#include "cartographer/mapping/internal/3d/local_trajectory_builder_options_3d.h"
#include "cartographer/mapping/internal/3d/optimizing_local_trajectory_builder.h"
#include "cartographer/sensor/range_data.h"
#include "cartographer/transform/rigid_transform.h"
#include "cartographer/transform/rigid_transform_test_helpers.h"
#include "cartographer/transform/transform.h"

namespace cartographer {
namespace mapping {

namespace {

constexpr char kSensorId[] = "sensor_id";

class LocalTrajectoryBuilderEval {
 public:
  mapping::proto::LocalTrajectoryBuilderOptions3D
  CreateTrajectoryBuilderEvalOptions3D() {
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

          submaps = {
            high_resolution = 0.2,
            high_resolution_max_range = 50.,
            low_resolution = 0.5,
            num_range_data = 45000,
            grid_type = "TSDF",
            range_data_inserter = {
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
                project_sdf_distance_to_scan_normal = true,
                weight_function_epsilon = 1,
                weight_function_sigma = 4.,
                normal_estimate_max_nn = 20.,
                normal_estimate_radius = 0.3,
              },
            },
          },
        optimizing_local_trajectory_builder = {
              high_resolution_grid_weight = 100,
              low_resolution_grid_weight = 10,
              velocity_weight = 1,
              translation_weight = 1,
              rotation_weight = 1,
              odometry_translation_weight = 1,
              odometry_rotation_weight = 1,
              initialize_map_orientation_with_imu = true,
              calibrate_imu = false,
              optimization_rate = 0.05,
              ct_window_horizon = 0.6,
              ct_window_rate = 0.3,
              imu_integrator = "RK4",
              imu_cost_term = "DIRECT",
              sync_control_points_with_range_data = true,
          },
        }
        )text");
    return mapping::CreateLocalTrajectoryBuilderOptions3D(
        parameter_dictionary.get());
  }

  LocalTrajectoryBuilderEval() {
    local_trajectory_builder_.reset(new OptimizingLocalTrajectoryBuilder(
        CreateTrajectoryBuilderEvalOptions3D(), {kSensorId}));
    SetUp();
    //    VerifyAccuracy(GenerateCorkscrewTrajectory(), 1e-1);
    Evaluate(0.01);
  }

 public:
  struct TrajectoryNode {
    common::Time time;
    transform::Rigid3d pose;
  };

  void SetUp() {
    scene_.AddBox({-5.f, -5.f, -1.f}, {10.f, 20.f, 10.f});
    scene_.AddSphere({5.f, 5.f, 0.f}, 3.f);
    scene_.AddSphere({-3.f, 2.f, -1.f}, 1.f);
    scene_.AddSphere({-2.f, -2.f, 2.f}, 2.f);
  }

  void AddLinearOnlyImuObservation(const common::Time time,
                                   const transform::Rigid3d& expected_pose) {
    const Eigen::Vector3d gravity =
        expected_pose.rotation().inverse() * Eigen::Vector3d(0., 0., 9.81);
    local_trajectory_builder_->AddImuData(
        sensor::ImuData{time, gravity, Eigen::Vector3d::Zero()});
  }

  void GetLinearTrajectory(double t, Eigen::Vector3f& p, Eigen::Vector3f& v,
                           Eigen::Vector3f& a) {
    double t_init = 12.0;
    double velocity = 0.5;
    if (t <= t_init) {
      p = {0.f, 0.f, 0.f};
      v = {0.f, 0.f, 0.f};
      a = {0.f, 0.f, 0.f};
    } else {
      p = {0.f, 0.f, float(t - t_init) * float(velocity)};
      v = {0.f, 0.f, float(velocity)};
      a = {0.f, 0.f, 0.f};
    }
  }

  std::vector<TrajectoryNode> GenerateCorkscrewTrajectory() {
    std::vector<TrajectoryNode> trajectory;
    common::Time current_time = common::FromUniversal(12345678);
    // One second at zero velocity.
    for (int i = 0; i != 15; ++i) {
      current_time += common::FromSeconds(0.3);
      trajectory.push_back(
          TrajectoryNode{current_time, transform::Rigid3d::Identity()});
    }
    // Corkscrew translation and constant velocity rotation.
    for (double t = 0.; t <= 1.6; t += 0.05) {
      current_time += common::FromSeconds(0.3);
      trajectory.push_back(TrajectoryNode{
          current_time,
          transform::Rigid3d::Translation(Eigen::Vector3d(0, 0, 1. * t))});

      //      trajectory.push_back(
      //          TrajectoryNode{current_time, transform::Rigid3d::Identity()});
    }
    return trajectory;
  }

  //  void VerifyAccuracy(const std::vector<TrajectoryNode>&
  //  expected_trajectory,
  //                      double expected_accuracy) {
  //    int num_poses = 0;
  //    for (const TrajectoryNode& node : expected_trajectory) {
  //      AddLinearOnlyImuObservation(node.time, node.pose);
  //      const auto range_data =
  //          range_sensor_.GenerateRangeData(node.pose, scene_);
  //      const
  //      std::unique_ptr<OptimizingLocalTrajectoryBuilder::MatchingResult>
  //          matching_result = local_trajectory_builder_->AddRangeData(
  //              kSensorId, sensor::TimedPointCloudData{
  //                             node.time, range_data.origin,
  //                             range_data.returns});
  //
  //      sensor::WriteToPCD(
  //          sensor::TransformTimedRangeData(range_data,
  //          node.pose.cast<float>()), "range_data_" +
  //          std::to_string(num_poses) + ".pcd");
  //      if (matching_result != nullptr) {
  //        //          cartographer::evaluation::GridDrawer grid_drawer =
  //        //          cartographer::evaluation::GridDrawer(static_cast<const
  //        //
  //        HybridGridTSDF&>(matching_result.get()->insertion_result.get()->insertion_submaps[0]->high_resolution_hybrid_grid()));
  //        //        cartographer::evaluation::GridDrawer grid_drawer =
  //        //            cartographer::evaluation::GridDrawer();
  //        //        grid_drawer.DrawInterpolatedTSD(static_cast<const
  //        //        HybridGridTSDF&>(
  //        //                                            matching_result.get()
  //        // ->insertion_result.get()
  //        // ->insertion_submaps[0]
  //        // ->low_resolution_hybrid_grid()),
  //        //                                        0.2);
  //        //          grid_drawer.DrawPointcloud(sample.range_data.returns,
  //        // transform::Rigid3d::Identity());
  //        //        grid_drawer.ToFile("interpolated_grid_with_cloud" +
  //        //                           std::to_string(num_poses) + ".png");
  //        LOG(INFO) << "res " << matching_result->local_pose;
  //        LOG(INFO) << "res " << matching_result->time;
  //        LOG(INFO) << "exp " << node.pose;
  //        LOG(INFO) << "exp " << node.time;
  //        //        EXPECT_THAT(matching_result->local_pose,
  //        //                    transform::IsNearly(node.pose, 1e-1));
  //
  //        ++num_poses;
  //        LOG(INFO) << "num_poses: " << num_poses;
  //      }
  //    }
  //  }

  void Evaluate(double expected_accuracy) {
    double imu_frequency = 100;
    double lidar_frequency = 20;
    double sim_step_duration = 0.01;
    double t_end = 4.0;
    int num_poses = 0;
    std::vector<float> positions_x_measured = {};
    std::vector<float> positions_y_measured = {};
    std::vector<float> positions_z_measured = {};
    std::vector<float> positions_x_gt = {};
    std::vector<float> positions_y_gt = {};
    std::vector<float> positions_z_gt = {};
    std::vector<float> timestamps_measured = {};
    std::vector<float> timestamps_gt = {};
    common::Time start_time = common::FromUniversal(12345678);
    common::Time last_imu_measurement_time = common::FromUniversal(0);
    common::Time last_lidar_measurement_time = common::FromUniversal(0);
    for (common::Time current_time = start_time;
         common::ToSeconds(current_time - start_time) < t_end;
         current_time += common::FromSeconds(sim_step_duration)) {
      Eigen::Vector3f p, v, a;
      GetLinearTrajectory(common::ToSeconds(current_time - start_time), p, v,
                          a);
      transform::Rigid3d expected_pose = {p.cast<double>(),
                                          Eigen::Quaterniond::Identity()};
      if (common::ToSeconds(current_time - last_imu_measurement_time) >=
          1.0 / imu_frequency) {
        AddLinearOnlyImuObservation(current_time, expected_pose);
        last_imu_measurement_time = current_time;
      }
      if (common::ToSeconds(current_time - last_lidar_measurement_time) <
          1.0 / lidar_frequency) {
        continue;
      }
      const auto range_data = range_sensor_.GenerateRangeData(
          common::ToSeconds(current_time - start_time), expected_pose, scene_);
      const std::unique_ptr<OptimizingLocalTrajectoryBuilder::MatchingResult>
          matching_result = local_trajectory_builder_->AddRangeData(
              kSensorId,
              sensor::TimedPointCloudData{current_time, range_data.origin,
                                          range_data.returns});

      //      sensor::WriteToPCD(
      //          sensor::TransformTimedRangeData(range_data,
      //          expected_pose.cast<float>()), "range_data_" +
      //          std::to_string(num_poses) + ".pcd");
      if (matching_result != nullptr) {
        //          cartographer::evaluation::GridDrawer grid_drawer =
        //          cartographer::evaluation::GridDrawer(static_cast<const
        //          HybridGridTSDF&>(matching_result.get()->insertion_result.get()->insertion_submaps[0]->high_resolution_hybrid_grid()));
        //        cartographer::evaluation::GridDrawer grid_drawer =
        //            cartographer::evaluation::GridDrawer();
        //        grid_drawer.DrawInterpolatedTSD(static_cast<const
        //        HybridGridTSDF&>(
        //                                            matching_result.get()
        //                                                ->insertion_result.get()
        //                                                ->insertion_submaps[0]
        //                                                ->low_resolution_hybrid_grid()),
        //                                        0.2);
        //          grid_drawer.DrawPointcloud(sample.range_data.returns,
        //                                     transform::Rigid3d::Identity());
        //        grid_drawer.ToFile("interpolated_grid_with_cloud" +
        //                           std::to_string(num_poses) + ".png");
        LOG(INFO) << "res " << matching_result->local_pose;
        LOG(INFO) << "res "
                  << common::ToSeconds(matching_result->time - start_time);
        LOG(INFO) << "exp " << expected_pose;
        LOG(INFO) << "exp " << common::ToSeconds(current_time - start_time);
        //        EXPECT_THAT(matching_result->local_pose,
        //                    transform::IsNearly(node.pose, 1e-1));

        positions_x_measured.push_back(
            matching_result->local_pose.translation().x());
        positions_y_measured.push_back(
            matching_result->local_pose.translation().y());
        positions_z_measured.push_back(
            matching_result->local_pose.translation().z());
        timestamps_measured.push_back(
            common::ToSeconds(matching_result->time - start_time));

        ++num_poses;
        LOG(INFO) << "num_poses: " << num_poses;
      }
    }
    matplotlibcpp::plot();
    matplotlibcpp::named_plot("x", timestamps_measured, positions_x_measured);
    matplotlibcpp::named_plot("y", timestamps_measured, positions_y_measured);
    matplotlibcpp::named_plot("z", timestamps_measured, positions_z_measured);
    matplotlibcpp::legend();
    matplotlibcpp::show();
  }

  std::unique_ptr<OptimizingLocalTrajectoryBuilder> local_trajectory_builder_;
  evaluation::Scene scene_;
  evaluation::RangeSensor range_sensor_;
};

}  // namespace
}  // namespace mapping
}  // namespace cartographer

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  FLAGS_logtostderr = true;
  google::ParseCommandLineFlags(&argc, &argv, true);

  cartographer::mapping::LocalTrajectoryBuilderEval();
  //    LocalTrajectoryBuilderEval
  //    cartographer::evaluation::RunScanMatchingEvaluationRotatingScan();
}