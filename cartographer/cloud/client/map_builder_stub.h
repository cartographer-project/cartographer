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

#ifndef CARTOGRAPHER_CLOUD_CLIENT_MAP_BUILDER_STUB_H_
#define CARTOGRAPHER_CLOUD_CLIENT_MAP_BUILDER_STUB_H_

#include <memory>

#include "cartographer/mapping/map_builder_interface.h"
#include "cartographer/mapping/pose_graph_interface.h"
#include "cartographer/mapping/trajectory_builder_interface.h"
#include "grpc++/grpc++.h"

namespace cartographer {
namespace cloud {

class MapBuilderStub : public mapping::MapBuilderInterface {
 public:
  MapBuilderStub(const std::string& server_address);

  MapBuilderStub(const MapBuilderStub&) = delete;
  MapBuilderStub& operator=(const MapBuilderStub&) = delete;

  int AddTrajectoryBuilder(
      const std::set<SensorId>& expected_sensor_ids,
      const mapping::proto::TrajectoryBuilderOptions& trajectory_options,
      LocalSlamResultCallback local_slam_result_callback) override;
  int AddTrajectoryForDeserialization(
      const mapping::proto::TrajectoryBuilderOptionsWithSensorIds&
          options_with_sensor_ids_proto) override;
  mapping::TrajectoryBuilderInterface* GetTrajectoryBuilder(
      int trajectory_id) const override;
  void FinishTrajectory(int trajectory_id) override;
  std::string SubmapToProto(
      const mapping::SubmapId& submap_id,
      mapping::proto::SubmapQuery::Response* response) override;
  void SerializeState(io::ProtoStreamWriterInterface* writer) override;
  void LoadState(io::ProtoStreamReaderInterface* reader,
                 bool load_frozen_state) override;
  int num_trajectory_builders() const override;
  mapping::PoseGraphInterface* pose_graph() override;
  const std::vector<mapping::proto::TrajectoryBuilderOptionsWithSensorIds>&
  GetAllTrajectoryBuilderOptions() const override;

 private:
  std::shared_ptr<::grpc::Channel> client_channel_;
  std::unique_ptr<mapping::PoseGraphInterface> pose_graph_stub_;
  std::map<int, std::unique_ptr<mapping::TrajectoryBuilderInterface>>
      trajectory_builder_stubs_;
};

}  // namespace cloud
}  // namespace cartographer

#endif  // CARTOGRAPHER_CLOUD_CLIENT_MAP_BUILDER_STUB_H_
