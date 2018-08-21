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

#ifndef CARTOGRAPHER_INTERNAL_CLOUD_MAP_BUILDER_CONTEXT_IMPL_H
#define CARTOGRAPHER_INTERNAL_CLOUD_MAP_BUILDER_CONTEXT_IMPL_H

namespace cartographer {
namespace cloud {

template <class SubmapType>
MapBuilderContext<SubmapType>::MapBuilderContext(
    MapBuilderServer* map_builder_server)
    : map_builder_server_(map_builder_server) {}

template <class SubmapType>
mapping::MapBuilderInterface& MapBuilderContext<SubmapType>::map_builder() {
  return *map_builder_server_->map_builder_;
}

template <class SubmapType>
common::BlockingQueue<std::unique_ptr<MapBuilderContextInterface::Data>>&
MapBuilderContext<SubmapType>::sensor_data_queue() {
  return map_builder_server_->incoming_data_queue_;
}

template <class SubmapType>
mapping::TrajectoryBuilderInterface::LocalSlamResultCallback
MapBuilderContext<SubmapType>::GetLocalSlamResultCallbackForSubscriptions() {
  return [this](int trajectory_id, common::Time time,
                transform::Rigid3d local_pose, sensor::RangeData range_data,
                std::unique_ptr<
                    const mapping::TrajectoryBuilderInterface::InsertionResult>
                    insertion_result) {
    auto it = client_ids_.find(trajectory_id);
    if (it == client_ids_.end()) {
      LOG(ERROR) << "Unknown trajectory_id " << trajectory_id << ". Ignoring.";
      return;
    }
    map_builder_server_->OnLocalSlamResult(trajectory_id, it->second, time,
                                           local_pose, std::move(range_data),
                                           std::move(insertion_result));
  };
}

template <class SubmapType>
void MapBuilderContext<SubmapType>::AddSensorDataToTrajectory(
    const Data& sensor_data) {
  sensor_data.data->AddToTrajectoryBuilder(
      map_builder_server_->map_builder_->GetTrajectoryBuilder(
          sensor_data.trajectory_id));
}

template <class SubmapType>
MapBuilderContextInterface::LocalSlamSubscriptionId
MapBuilderContext<SubmapType>::SubscribeLocalSlamResults(
    int trajectory_id, LocalSlamSubscriptionCallback callback) {
  return map_builder_server_->SubscribeLocalSlamResults(trajectory_id,
                                                        callback);
}

template <class SubmapType>
void MapBuilderContext<SubmapType>::UnsubscribeLocalSlamResults(
    const LocalSlamSubscriptionId& subscription_id) {
  map_builder_server_->UnsubscribeLocalSlamResults(subscription_id);
}

template <class SubmapType>
int MapBuilderContext<SubmapType>::SubscribeGlobalSlamOptimizations(
    GlobalSlamOptimizationCallback callback) {
  return map_builder_server_->SubscribeGlobalSlamOptimizations(callback);
}

template <class SubmapType>
void MapBuilderContext<SubmapType>::UnsubscribeGlobalSlamOptimizations(
    int subscription_index) {
  map_builder_server_->UnsubscribeGlobalSlamOptimizations(subscription_index);
}

template <class SubmapType>
void MapBuilderContext<SubmapType>::NotifyFinishTrajectory(int trajectory_id) {
  map_builder_server_->NotifyFinishTrajectory(trajectory_id);
}

template <class SubmapType>
LocalTrajectoryUploaderInterface*
MapBuilderContext<SubmapType>::local_trajectory_uploader() {
  return map_builder_server_->local_trajectory_uploader_.get();
}

template <class SubmapType>
void MapBuilderContext<SubmapType>::EnqueueSensorData(
    int trajectory_id, std::unique_ptr<sensor::Data> data) {
  map_builder_server_->incoming_data_queue_.Push(
      absl::make_unique<Data>(Data{trajectory_id, std::move(data)}));
}

template <class SubmapType>
void MapBuilderContext<SubmapType>::RegisterClientIdForTrajectory(
    const std::string& client_id, int trajectory_id) {
  CHECK_EQ(client_ids_.count(trajectory_id), 0u);
  LOG(INFO) << "Registering trajectory_id " << trajectory_id << " to client_id "
            << client_id;
  client_ids_[trajectory_id] = client_id;
}

template <class SubmapType>
bool MapBuilderContext<SubmapType>::CheckClientIdForTrajectory(
    const std::string& client_id, int trajectory_id) {
  return (client_ids_.count(trajectory_id) > 0 &&
          client_ids_[trajectory_id] == client_id);
}

template <>
void MapBuilderContext<mapping::Submap2D>::EnqueueLocalSlamResultData(
    int trajectory_id, const std::string& sensor_id,
    const mapping::proto::LocalSlamResultData& local_slam_result_data);
template <>
void MapBuilderContext<mapping::Submap3D>::EnqueueLocalSlamResultData(
    int trajectory_id, const std::string& sensor_id,
    const mapping::proto::LocalSlamResultData& local_slam_result_data);

}  // namespace cloud
}  // namespace cartographer

#endif  // CARTOGRAPHER_INTERNAL_CLOUD_MAP_BUILDER_CONTEXT_IMPL_H
