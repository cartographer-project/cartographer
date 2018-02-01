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

#ifndef CARTOGRAPHER_GRPC_MAPPING_MAP_BUILDER_STUB_H_
#define CARTOGRAPHER_GRPC_MAPPING_MAP_BUILDER_STUB_H_

#include <memory>

#include "cartographer/mapping/map_builder_interface.h"
#include "cartographer_grpc/mapping/pose_graph_stub.h"
#include "cartographer_grpc/mapping/trajectory_builder_stub.h"
#include "grpc++/grpc++.h"

namespace cartographer_grpc {
namespace mapping {

class MapBuilderStub : public cartographer::mapping::MapBuilderInterface {
 public:
  MapBuilderStub(const std::string& server_address);

  MapBuilderStub(const MapBuilderStub&) = delete;
  MapBuilderStub& operator=(const MapBuilderStub&) = delete;

  int AddTrajectoryBuilder(
      const std::set<SensorId>& expected_sensor_ids,
      const cartographer::mapping::proto::TrajectoryBuilderOptions&
          trajectory_options,
      LocalSlamResultCallback local_slam_result_callback) override;
  int AddTrajectoryForDeserialization() override;
  cartographer::mapping::TrajectoryBuilderInterface* GetTrajectoryBuilder(
      int trajectory_id) const override;
  void FinishTrajectory(int trajectory_id) override;
  std::string SubmapToProto(
      const cartographer::mapping::SubmapId& submap_id,
      cartographer::mapping::proto::SubmapQuery::Response* response) override;
  void SerializeState(
      cartographer::io::ProtoStreamWriterInterface* writer) override;
  void LoadMap(cartographer::io::ProtoStreamReaderInterface* reader) override;
  int num_trajectory_builders() const override;
  cartographer::mapping::PoseGraphInterface* pose_graph() override;

 private:
  std::shared_ptr<grpc::Channel> client_channel_;
  PoseGraphStub pose_graph_stub_;
  std::map<int, std::unique_ptr<TrajectoryBuilderStub>>
      trajectory_builder_stubs_;
};

}  // namespace mapping
}  // namespace cartographer_grpc

#endif  // CARTOGRAPHER_GRPC_MAPPING_MAP_BUILDER_STUB_H_
