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
#include "cartographer_grpc/proto/map_builder_service.grpc.pb.h"
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
  void AddLocalSlamResultData(
      std::unique_ptr<cartographer::mapping::LocalSlamResultData>
          local_slam_result_data) override;

 private:
  template <typename RequestType>
  struct SensorClientWriter {
    grpc::ClientContext client_context;
    std::unique_ptr<grpc::ClientWriter<RequestType>> client_writer;
    google::protobuf::Empty response;
  };
  struct LocalSlamResultReader {
    grpc::ClientContext client_context;
    std::unique_ptr<grpc::ClientReader<proto::ReceiveLocalSlamResultsResponse>>
        client_reader;
    std::unique_ptr<std::thread> thread;
  };

  static void RunLocalSlamResultReader(
      grpc::ClientReader<proto::ReceiveLocalSlamResultsResponse>* client_reader,
      LocalSlamResultCallback local_slam_result_callback);

  std::shared_ptr<grpc::Channel> client_channel_;
  const int trajectory_id_;
  std::unique_ptr<proto::MapBuilderService::Stub> stub_;
  SensorClientWriter<proto::AddRangefinderDataRequest> rangefinder_writer_;
  SensorClientWriter<proto::AddImuDataRequest> imu_writer_;
  SensorClientWriter<proto::AddOdometryDataRequest> odometry_writer_;
  SensorClientWriter<proto::AddFixedFramePoseDataRequest> fixed_frame_writer_;
  LocalSlamResultReader local_slam_result_reader_;
};

}  // namespace mapping
}  // namespace cartographer_grpc

#endif  // CARTOGRAPHER_GRPC_MAPPING_TRAJECTORY_BUILDER_STUB_H_
