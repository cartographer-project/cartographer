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

#ifndef CARTOGRAPHER_CLOUD_INTERNAL_CLIENT_TRAJECTORY_BUILDER_STUB_H_
#define CARTOGRAPHER_CLOUD_INTERNAL_CLIENT_TRAJECTORY_BUILDER_STUB_H_

#include <thread>

#include "async_grpc/client.h"
#include "cartographer/cloud/internal/handlers/add_fixed_frame_pose_data_handler.h"
#include "cartographer/cloud/internal/handlers/add_imu_data_handler.h"
#include "cartographer/cloud/internal/handlers/add_landmark_data_handler.h"
#include "cartographer/cloud/internal/handlers/add_odometry_data_handler.h"
#include "cartographer/cloud/internal/handlers/add_rangefinder_data_handler.h"
#include "cartographer/cloud/internal/handlers/receive_local_slam_results_handler.h"
#include "cartographer/mapping/local_slam_result_data.h"
#include "cartographer/mapping/trajectory_builder_interface.h"
#include "grpc++/grpc++.h"
#include "pose_graph_stub.h"
#include "trajectory_builder_stub.h"

namespace cartographer {
namespace cloud {

class TrajectoryBuilderStub : public mapping::TrajectoryBuilderInterface {
 public:
  TrajectoryBuilderStub(std::shared_ptr<::grpc::Channel> client_channel,
                        const int trajectory_id,
                        LocalSlamResultCallback local_slam_result_callback);
  ~TrajectoryBuilderStub() override;
  TrajectoryBuilderStub(const TrajectoryBuilderStub&) = delete;
  TrajectoryBuilderStub& operator=(const TrajectoryBuilderStub&) = delete;

  void AddSensorData(
      const std::string& sensor_id,
      const sensor::TimedPointCloudData& timed_point_cloud_data) override;
  void AddSensorData(const std::string& sensor_id,
                     const sensor::ImuData& imu_data) override;
  void AddSensorData(const std::string& sensor_id,
                     const sensor::OdometryData& odometry_data) override;
  void AddSensorData(
      const std::string& sensor_id,
      const sensor::FixedFramePoseData& fixed_frame_pose) override;
  void AddSensorData(const std::string& sensor_id,
                     const sensor::LandmarkData& landmark_data) override;
  void AddLocalSlamResultData(std::unique_ptr<mapping::LocalSlamResultData>
                                  local_slam_result_data) override;

 private:
  static void RunLocalSlamResultsReader(
      async_grpc::Client<handlers::ReceiveLocalSlamResultsSignature>*
          client_reader,
      LocalSlamResultCallback local_slam_result_callback);

  std::shared_ptr<::grpc::Channel> client_channel_;
  const int trajectory_id_;
  std::unique_ptr<async_grpc::Client<handlers::AddRangefinderDataSignature>>
      add_rangefinder_client_;
  std::unique_ptr<async_grpc::Client<handlers::AddImuDataSignature>>
      add_imu_client_;
  std::unique_ptr<async_grpc::Client<handlers::AddOdometryDataSignature>>
      add_odometry_client_;
  std::unique_ptr<async_grpc::Client<handlers::AddFixedFramePoseDataSignature>>
      add_fixed_frame_pose_client_;
  std::unique_ptr<async_grpc::Client<handlers::AddLandmarkDataSignature>>
      add_landmark_client_;
  async_grpc::Client<handlers::ReceiveLocalSlamResultsSignature>
      receive_local_slam_results_client_;
  std::unique_ptr<std::thread> receive_local_slam_results_thread_;
};

}  // namespace cloud
}  // namespace cartographer

#endif  // CARTOGRAPHER_CLOUD_INTERNAL_CLIENT_TRAJECTORY_BUILDER_STUB_H_
