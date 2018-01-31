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

#include "cartographer_grpc/map_builder_context.h"
#include "cartographer_grpc/map_builder_server.h"

namespace cartographer_grpc {

MapBuilderContext::MapBuilderContext(MapBuilderServer* map_builder_server)
    : map_builder_server_(map_builder_server) {}

cartographer::mapping::MapBuilderInterface& MapBuilderContext::map_builder() {
  return *map_builder_server_->map_builder_;
}

cartographer::common::BlockingQueue<
    std::unique_ptr<MapBuilderContextInterface::Data>>&
MapBuilderContext::sensor_data_queue() {
  return map_builder_server_->incoming_data_queue_;
}

cartographer::mapping::TrajectoryBuilderInterface::LocalSlamResultCallback
MapBuilderContext::GetLocalSlamResultCallbackForSubscriptions() {
  MapBuilderServer* map_builder_server = map_builder_server_;
  return [map_builder_server](
             int trajectory_id, cartographer::common::Time time,
             cartographer::transform::Rigid3d local_pose,
             cartographer::sensor::RangeData range_data,
             std::unique_ptr<const cartographer::mapping::
                                 TrajectoryBuilderInterface::InsertionResult>
                 insertion_result) {
    map_builder_server->OnLocalSlamResult(trajectory_id, time, local_pose,
                                          std::move(range_data),
                                          std::move(insertion_result));
  };
}

void MapBuilderContext::AddSensorDataToTrajectory(const Data& sensor_data) {
  sensor_data.data->AddToTrajectoryBuilder(
      map_builder_server_->map_builder_->GetTrajectoryBuilder(
          sensor_data.trajectory_id));
}

MapBuilderContextInterface::SubscriptionId
MapBuilderContext::SubscribeLocalSlamResults(
    int trajectory_id, LocalSlamSubscriptionCallback callback) {
  return map_builder_server_->SubscribeLocalSlamResults(trajectory_id,
                                                        callback);
}

void MapBuilderContext::UnsubscribeLocalSlamResults(
    const SubscriptionId& subscription_id) {
  map_builder_server_->UnsubscribeLocalSlamResults(subscription_id);
}

void MapBuilderContext::NotifyFinishTrajectory(int trajectory_id) {
  map_builder_server_->NotifyFinishTrajectory(trajectory_id);
}

std::shared_ptr<cartographer::mapping_2d::Submap>
MapBuilderContext::UpdateSubmap2D(
    const cartographer::mapping::proto::Submap& proto) {
  CHECK(proto.has_submap_2d());
  cartographer::mapping::SubmapId submap_id{proto.submap_id().trajectory_id(),
                                            proto.submap_id().submap_index()};
  std::shared_ptr<cartographer::mapping_2d::Submap> submap_2d_ptr;
  auto submap_it = unfinished_submaps_.find(submap_id);
  if (submap_it == unfinished_submaps_.end()) {
    // Seeing a submap for the first time it should never be finished.
    CHECK(!proto.submap_2d().finished());
    submap_2d_ptr =
        std::make_shared<cartographer::mapping_2d::Submap>(proto.submap_2d());
    unfinished_submaps_.Insert(submap_id, submap_2d_ptr);
  } else {
    submap_2d_ptr = std::dynamic_pointer_cast<cartographer::mapping_2d::Submap>(
        submap_it->data);
    CHECK(submap_2d_ptr);
    submap_2d_ptr->UpdateFromProto(proto);

    // If the submap was just finished by the recent update, remove it from the
    // list of unfinished submaps.
    if (submap_2d_ptr->finished()) {
      unfinished_submaps_.Trim(submap_id);
    } else {
      // If the submap is unfinished set the 'num_range_data' to 0 since we
      // haven't changed the HybridGrid.
      submap_2d_ptr->SetNumRangeData(0);
    }
  }
  return submap_2d_ptr;
}

std::shared_ptr<cartographer::mapping_3d::Submap>
MapBuilderContext::UpdateSubmap3D(
    const cartographer::mapping::proto::Submap& proto) {
  CHECK(proto.has_submap_3d());
  cartographer::mapping::SubmapId submap_id{proto.submap_id().trajectory_id(),
                                            proto.submap_id().submap_index()};
  std::shared_ptr<cartographer::mapping_3d::Submap> submap_3d_ptr;
  auto submap_it = unfinished_submaps_.find(submap_id);
  if (submap_it == unfinished_submaps_.end()) {
    // Seeing a submap for the first time it should never be finished.
    CHECK(!proto.submap_3d().finished());
    submap_3d_ptr =
        std::make_shared<cartographer::mapping_3d::Submap>(proto.submap_3d());
    unfinished_submaps_.Insert(submap_id, submap_3d_ptr);
    submap_it = unfinished_submaps_.find(submap_id);
  } else {
    submap_3d_ptr = std::dynamic_pointer_cast<cartographer::mapping_3d::Submap>(
        submap_it->data);
    CHECK(submap_3d_ptr);

    // Update submap with information in incoming request.
    submap_3d_ptr->UpdateFromProto(proto);

    // If the submap was just finished by the recent update, remove it from the
    // list of unfinished submaps.
    if (submap_3d_ptr->finished()) {
      unfinished_submaps_.Trim(submap_id);
    } else {
      // If the submap is unfinished set the 'num_range_data' to 0 since we
      // haven't changed the HybridGrid.
      submap_3d_ptr->SetNumRangeData(0);
    }
  }
  return submap_3d_ptr;
}

std::unique_ptr<cartographer::mapping::LocalSlamResultData>
MapBuilderContext::ProcessLocalSlamResultData(
    const std::string& sensor_id, cartographer::common::Time time,
    const cartographer::mapping::proto::LocalSlamResultData& proto) {
  CHECK_GE(proto.submaps().size(), 1);
  CHECK(proto.submaps(0).has_submap_2d() || proto.submaps(0).has_submap_3d());
  if (proto.submaps(0).has_submap_2d()) {
    std::vector<std::shared_ptr<const cartographer::mapping_2d::Submap>>
        submaps;
    for (const auto& submap_proto : proto.submaps()) {
      submaps.push_back(UpdateSubmap2D(submap_proto));
    }
    return cartographer::common::make_unique<
        cartographer::mapping::LocalSlamResult2D>(
        sensor_id, time,
        std::make_shared<const cartographer::mapping::TrajectoryNode::Data>(
            cartographer::mapping::FromProto(proto.node_data())),
        submaps);
  } else {
    std::vector<std::shared_ptr<const cartographer::mapping_3d::Submap>>
        submaps;
    for (const auto& submap_proto : proto.submaps()) {
      submaps.push_back(UpdateSubmap3D(submap_proto));
    }
    return cartographer::common::make_unique<
        cartographer::mapping::LocalSlamResult3D>(
        sensor_id, time,
        std::make_shared<const cartographer::mapping::TrajectoryNode::Data>(
            cartographer::mapping::FromProto(proto.node_data())),
        std::move(submaps));
  }
}

LocalTrajectoryUploader* MapBuilderContext::local_trajectory_uploader() {
  return map_builder_server_->local_trajectory_uploader_.get();
}

void MapBuilderContext::EnqueueSensorData(
    int trajectory_id, std::unique_ptr<cartographer::sensor::Data> data) {
  map_builder_server_->incoming_data_queue_.Push(
      cartographer::common::make_unique<Data>(
          Data{trajectory_id, std::move(data)}));
}

void MapBuilderContext::EnqueueLocalSlamResultData(
    int trajectory_id, const std::string& sensor_id,
    std::unique_ptr<cartographer::mapping::LocalSlamResultData>
        local_slam_result_data) {
  map_builder_server_->incoming_data_queue_.Push(
      cartographer::common::make_unique<Data>(
          Data{trajectory_id, std::move(local_slam_result_data)}));
}

}  // namespace cartographer_grpc