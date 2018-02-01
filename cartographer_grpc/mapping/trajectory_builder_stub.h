/*
 * Copyright 2017 The Cartographer Authors
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

#ifndef CARTOGRAPHER_GRPC_MAPPING_TRAJECTORY_BUILDER_STUB_H_
#define CARTOGRAPHER_GRPC_MAPPING_TRAJECTORY_BUILDER_STUB_H_

#include <thread>

#include "cartographer/mapping/local_slam_result_data.h"
#include "cartographer/mapping/trajectory_builder_interface.h"
#include "cartographer_grpc/framework/client.h"
#include "cartographer_grpc/handlers/add_fixed_frame_pose_data_handler.h"
#include "cartographer_grpc/handlers/add_imu_data_handler.h"
#include "cartographer_grpc/handlers/add_landmark_data_handler.h"
#include "cartographer_grpc/handlers/add_local_slam_result_data_handler.h"
#include "cartographer_grpc/handlers/add_odometry_data_handler.h"
#include "cartographer_grpc/handlers/add_rangefinder_data_handler.h"
#include "cartographer_grpc/handlers/receive_local_slam_results_handler.h"
#include "grpc++/grpc++.h"

namespace cartographer_grpc {
namespace mapping {

class TrajectoryBuilderStub
    : public cartographer::mapping::TrajectoryBuilderInterface {
 public:
  TrajectoryBuilderStub(std::shared_ptr<grpc::Channel> client_channel,
                        const int trajectory_id,
                        LocalSlamResultCallback local_slam_result_callback);
  ~TrajectoryBuilderStub() override;
  TrajectoryBuilderStub(const TrajectoryBuilderStub&) = delete;
  TrajectoryBuilderStub& operator=(const TrajectoryBuilderStub&) = delete;

  void AddSensorData(const std::string& sensor_id,
                     const cartographer::sensor::TimedPointCloudData&
                         timed_point_cloud_data) override;
  void AddSensorData(const std::string& sensor_id,
                     const cartographer::sensor::ImuData& imu_data) override;
  void AddSensorData(
      const std::string& sensor_id,
      const cartographer::sensor::OdometryData& odometry_data) override;
  void AddSensorData(const std::string& sensor_id,
                     const cartographer::sensor::FixedFramePoseData&
                         fixed_frame_pose) override;
  void AddSensorData(
      const std::string& sensor_id,
      const cartographer::sensor::LandmarkData& landmark_data) override;
  void AddLocalSlamResultData(
      std::unique_ptr<cartographer::mapping::LocalSlamResultData>
          local_slam_result_data) override;

 private:
  static void RunLocalSlamResultsReader(
      framework::Client<handlers::ReceiveLocalSlamResultsHandler>*
          client_reader,
      LocalSlamResultCallback local_slam_result_callback);

  std::shared_ptr<grpc::Channel> client_channel_;
  const int trajectory_id_;
  std::unique_ptr<framework::Client<handlers::AddRangefinderDataHandler>>
      add_rangefinder_client_;
  std::unique_ptr<framework::Client<handlers::AddImuDataHandler>>
      add_imu_client_;
  std::unique_ptr<framework::Client<handlers::AddOdometryDataHandler>>
      add_odometry_client_;
  std::unique_ptr<framework::Client<handlers::AddFixedFramePoseDataHandler>>
      add_fixed_frame_pose_client_;
  std::unique_ptr<framework::Client<handlers::AddLandmarkDataHandler>>
      add_landmark_client_;
  framework::Client<handlers::ReceiveLocalSlamResultsHandler>
      receive_local_slam_results_client_;
  std::unique_ptr<std::thread> receive_local_slam_results_thread_;
};

}  // namespace mapping
}  // namespace cartographer_grpc

#endif  // CARTOGRAPHER_GRPC_MAPPING_TRAJECTORY_BUILDER_STUB_H_
