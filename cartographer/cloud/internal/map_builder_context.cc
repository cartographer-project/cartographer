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

#include "cartographer/cloud/internal/map_builder_context.h"

#include "cartographer/cloud/internal/map_builder_server.h"

namespace cartographer {
namespace cloud {

MapBuilderContext::MapBuilderContext(MapBuilderServer* map_builder_server)
    : map_builder_server_(map_builder_server) {}

mapping::MapBuilderInterface& MapBuilderContext::map_builder() {
  return *map_builder_server_->map_builder_;
}

common::BlockingQueue<std::unique_ptr<MapBuilderContextInterface::Data>>&
MapBuilderContext::sensor_data_queue() {
  return map_builder_server_->incoming_data_queue_;
}

mapping::TrajectoryBuilderInterface::LocalSlamResultCallback
MapBuilderContext::GetLocalSlamResultCallbackForSubscriptions() {
  MapBuilderServer* map_builder_server = map_builder_server_;
  return [map_builder_server](
             int trajectory_id, common::Time time,
             transform::Rigid3d local_pose, sensor::RangeData range_data,
             std::unique_ptr<
                 const mapping::TrajectoryBuilderInterface::InsertionResult>
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

std::shared_ptr<mapping::Submap2D> MapBuilderContext::UpdateSubmap2D(
    const mapping::proto::Submap& proto) {
  CHECK(proto.has_submap_2d());
  mapping::SubmapId submap_id{proto.submap_id().trajectory_id(),
                              proto.submap_id().submap_index()};
  std::shared_ptr<mapping::Submap2D> submap_2d_ptr;
  auto submap_it = unfinished_submaps_.find(submap_id);
  if (submap_it == unfinished_submaps_.end()) {
    // Seeing a submap for the first time it should never be finished.
    CHECK(!proto.submap_2d().finished());
    submap_2d_ptr = std::make_shared<mapping::Submap2D>(proto.submap_2d());
    unfinished_submaps_.Insert(submap_id, submap_2d_ptr);
  } else {
    submap_2d_ptr =
        std::dynamic_pointer_cast<mapping::Submap2D>(submap_it->data);
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

std::shared_ptr<mapping::Submap3D> MapBuilderContext::UpdateSubmap3D(
    const mapping::proto::Submap& proto) {
  CHECK(proto.has_submap_3d());
  mapping::SubmapId submap_id{proto.submap_id().trajectory_id(),
                              proto.submap_id().submap_index()};
  std::shared_ptr<mapping::Submap3D> submap_3d_ptr;
  auto submap_it = unfinished_submaps_.find(submap_id);
  if (submap_it == unfinished_submaps_.end()) {
    // Seeing a submap for the first time it should never be finished.
    CHECK(!proto.submap_3d().finished());
    submap_3d_ptr = std::make_shared<mapping::Submap3D>(proto.submap_3d());
    unfinished_submaps_.Insert(submap_id, submap_3d_ptr);
    submap_it = unfinished_submaps_.find(submap_id);
  } else {
    submap_3d_ptr =
        std::dynamic_pointer_cast<mapping::Submap3D>(submap_it->data);
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

std::unique_ptr<mapping::LocalSlamResultData>
MapBuilderContext::ProcessLocalSlamResultData(
    const std::string& sensor_id, common::Time time,
    const mapping::proto::LocalSlamResultData& proto) {
  CHECK_GE(proto.submaps().size(), 1);
  CHECK(proto.submaps(0).has_submap_2d() || proto.submaps(0).has_submap_3d());
  if (proto.submaps(0).has_submap_2d()) {
    std::vector<std::shared_ptr<const mapping::Submap2D>> submaps;
    for (const auto& submap_proto : proto.submaps()) {
      submaps.push_back(UpdateSubmap2D(submap_proto));
    }
    return common::make_unique<mapping::LocalSlamResult2D>(
        sensor_id, time,
        std::make_shared<const mapping::TrajectoryNode::Data>(
            mapping::FromProto(proto.node_data())),
        submaps);
  } else {
    std::vector<std::shared_ptr<const mapping::Submap3D>> submaps;
    for (const auto& submap_proto : proto.submaps()) {
      submaps.push_back(UpdateSubmap3D(submap_proto));
    }
    return common::make_unique<mapping::LocalSlamResult3D>(
        sensor_id, time,
        std::make_shared<const mapping::TrajectoryNode::Data>(
            mapping::FromProto(proto.node_data())),
        std::move(submaps));
  }
}

LocalTrajectoryUploaderInterface*
MapBuilderContext::local_trajectory_uploader() {
  return map_builder_server_->local_trajectory_uploader_.get();
}

void MapBuilderContext::EnqueueSensorData(int trajectory_id,
                                          std::unique_ptr<sensor::Data> data) {
  map_builder_server_->incoming_data_queue_.Push(
      common::make_unique<Data>(Data{trajectory_id, std::move(data)}));
}

void MapBuilderContext::EnqueueLocalSlamResultData(
    int trajectory_id, const std::string& sensor_id,
    std::unique_ptr<mapping::LocalSlamResultData> local_slam_result_data) {
  map_builder_server_->incoming_data_queue_.Push(common::make_unique<Data>(
      Data{trajectory_id, std::move(local_slam_result_data)}));
}

}  // namespace cloud
}  // namespace cartographer
