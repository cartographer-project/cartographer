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

#include "cartographer/mapping/trajectory_builder_interface.h"
#include "cartographer_grpc/proto/map_builder_service.grpc.pb.h"
#include "grpc++/grpc++.h"

namespace cartographer_grpc {
namespace mapping {

class TrajectoryBuilderStub
    : public cartographer::mapping::TrajectoryBuilderInterface {
 public:
  TrajectoryBuilderStub(std::shared_ptr<grpc::Channel> client_channel,
                        proto::MapBuilderService::Stub* stub);

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

 private:
  std::shared_ptr<grpc::Channel> client_channel_;
  proto::MapBuilderService::Stub* stub_;
};

}  // namespace mapping
}  // namespace cartographer_grpc

#endif  // CARTOGRAPHER_GRPC_MAPPING_TRAJECTORY_BUILDER_STUB_H_
