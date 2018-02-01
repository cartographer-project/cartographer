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

#ifndef CARTOGRAPHER_GRPC_LOCAL_TRAJECTORY_UPLOADER_H
#define CARTOGRAPHER_GRPC_LOCAL_TRAJECTORY_UPLOADER_H

#include <map>
#include <memory>
#include <set>
#include <string>
#include <thread>

#include "cartographer/common/blocking_queue.h"
#include "cartographer/mapping/proto/trajectory_builder_options.pb.h"
#include "cartographer/mapping/trajectory_builder_interface.h"
#include "cartographer_grpc/framework/client.h"
#include "cartographer_grpc/handlers/add_fixed_frame_pose_data_handler.h"
#include "cartographer_grpc/handlers/add_imu_data_handler.h"
#include "cartographer_grpc/handlers/add_landmark_data_handler.h"
#include "cartographer_grpc/handlers/add_local_slam_result_data_handler.h"
#include "cartographer_grpc/handlers/add_odometry_data_handler.h"
#include "cartographer_grpc/proto/map_builder_service.pb.h"
#include "grpc++/grpc++.h"

namespace cartographer_grpc {

class LocalTrajectoryUploaderInterface {
 public:
  using SensorId = cartographer::mapping::TrajectoryBuilderInterface::SensorId;

  virtual ~LocalTrajectoryUploaderInterface() = default;

  // Enqueue an Add*DataRequest message to be uploaded.
  virtual void EnqueueDataRequest(
      std::unique_ptr<google::protobuf::Message> data_request) = 0;
  virtual void AddTrajectory(
      int local_trajectory_id, const std::set<SensorId>& expected_sensor_ids,
      const cartographer::mapping::proto::TrajectoryBuilderOptions&
          trajectory_options) = 0;
  virtual void FinishTrajectory(int local_trajectory_id) = 0;

  virtual SensorId GetLocalSlamResultSensorId(
      int local_trajectory_id) const = 0;
};

class LocalTrajectoryUploader : public LocalTrajectoryUploaderInterface {
 public:
  LocalTrajectoryUploader(const std::string& uplink_server_address);
  ~LocalTrajectoryUploader();

  // Starts the upload thread.
  void Start();

  // Shuts down the upload thread. This method blocks until the shutdown is
  // complete.
  void Shutdown();

  void AddTrajectory(
      int local_trajectory_id, const std::set<SensorId>& expected_sensor_ids,
      const cartographer::mapping::proto::TrajectoryBuilderOptions&
          trajectory_options) override;
  void FinishTrajectory(int local_trajectory_id) override;
  void EnqueueDataRequest(
      std::unique_ptr<google::protobuf::Message> data_request) override;

  SensorId GetLocalSlamResultSensorId(int local_trajectory_id) const override {
    return SensorId{SensorId::SensorType::LOCAL_SLAM_RESULT,
                    "local_slam_result_" + std::to_string(local_trajectory_id)};
  }

 private:
  void ProcessSendQueue();
  void TranslateTrajectoryId(proto::SensorMetadata* sensor_metadata);
  void ProcessFixedFramePoseDataMessage(
      proto::AddFixedFramePoseDataRequest* data_request);
  void ProcessImuDataMessage(proto::AddImuDataRequest* data_request);
  void ProcessLocalSlamResultDataMessage(
      proto::AddLocalSlamResultDataRequest* data_request);
  void ProcessOdometryDataMessage(proto::AddOdometryDataRequest* data_request);
  void ProcessLandmarkDataMessage(proto::AddLandmarkDataRequest* data_request);

  std::shared_ptr<grpc::Channel> client_channel_;
  std::map<int, int> local_to_cloud_trajectory_id_map_;
  cartographer::common::BlockingQueue<
      std::unique_ptr<google::protobuf::Message>>
      send_queue_;
  bool shutting_down_ = false;
  std::unique_ptr<std::thread> upload_thread_;
  std::unique_ptr<framework::Client<handlers::AddFixedFramePoseDataHandler>>
      add_fixed_frame_pose_client_;
  std::unique_ptr<framework::Client<handlers::AddImuDataHandler>>
      add_imu_client_;
  std::unique_ptr<framework::Client<handlers::AddLocalSlamResultDataHandler>>
      add_local_slam_result_client_;
  std::unique_ptr<framework::Client<handlers::AddOdometryDataHandler>>
      add_odometry_client_;
  std::unique_ptr<framework::Client<handlers::AddLandmarkDataHandler>>
      add_landmark_client_;
};

}  // namespace cartographer_grpc

#endif  // CARTOGRAPHER_GRPC_LOCAL_TRAJECTORY_UPLOADER_H
