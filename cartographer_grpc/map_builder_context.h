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

#ifndef CARTOGRAPHER_GRPC_MAP_BUILDER_CONTEXT_H
#define CARTOGRAPHER_GRPC_MAP_BUILDER_CONTEXT_H

#include "cartographer/mapping_2d/submaps.h"
#include "cartographer/mapping_3d/submaps.h"
#include "cartographer_grpc/map_builder_context_interface.h"

namespace cartographer_grpc {

class MapBuilderContext : public MapBuilderContextInterface {
 public:
  MapBuilderContext(MapBuilderServer* map_builder_server);
  cartographer::mapping::MapBuilderInterface& map_builder() override;
  cartographer::common::BlockingQueue<std::unique_ptr<Data>>&
  sensor_data_queue() override;
  cartographer::mapping::TrajectoryBuilderInterface::LocalSlamResultCallback
  GetLocalSlamResultCallbackForSubscriptions() override;
  void AddSensorDataToTrajectory(const Data& sensor_data) override;
  SubscriptionId SubscribeLocalSlamResults(
      int trajectory_id, LocalSlamSubscriptionCallback callback) override;
  void UnsubscribeLocalSlamResults(
      const SubscriptionId& subscription_id) override;
  void NotifyFinishTrajectory(int trajectory_id) override;
  std::unique_ptr<cartographer::mapping::LocalSlamResultData>
  ProcessLocalSlamResultData(
      const std::string& sensor_id, cartographer::common::Time time,
      const cartographer::mapping::proto::LocalSlamResultData& proto) override;
  LocalTrajectoryUploaderInterface* local_trajectory_uploader() override;
  void EnqueueSensorData(
      int trajectory_id,
      std::unique_ptr<cartographer::sensor::Data> data) override;
  void EnqueueLocalSlamResultData(
      int trajectory_id, const std::string& sensor_id,
      std::unique_ptr<cartographer::mapping::LocalSlamResultData>
          local_slam_result_data) override;

 private:
  std::shared_ptr<cartographer::mapping_2d::Submap> UpdateSubmap2D(
      const cartographer::mapping::proto::Submap& proto);
  std::shared_ptr<cartographer::mapping_3d::Submap> UpdateSubmap3D(
      const cartographer::mapping::proto::Submap& proto);

  MapBuilderServer* map_builder_server_;
  cartographer::mapping::MapById<cartographer::mapping::SubmapId,
                                 std::shared_ptr<cartographer::mapping::Submap>>
      unfinished_submaps_;
};

}  // namespace cartographer_grpc

#endif  // CARTOGRAPHER_GRPC_MAP_BUILDER_CONTEXT_H
