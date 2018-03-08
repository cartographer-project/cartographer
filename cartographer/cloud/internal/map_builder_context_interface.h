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

#ifndef CARTOGRAPHER_CLOUD_MAP_BUILDER_CONTEXT_INTERFACE_H
#define CARTOGRAPHER_CLOUD_MAP_BUILDER_CONTEXT_INTERFACE_H

#include "cartographer/cloud/internal/framework/execution_context.h"
#include "cartographer/cloud/internal/local_trajectory_uploader.h"
#include "cartographer/common/blocking_queue.h"
#include "cartographer/mapping/map_builder_interface.h"
#include "cartographer/sensor/data.h"
#include "cartographer/sensor/range_data.h"
#include "cartographer/transform/rigid_transform.h"

namespace cartographer {
namespace cloud {

class MapBuilderServer;
class MapBuilderContextInterface : public framework::ExecutionContext {
 public:
  struct LocalSlamResult {
    int trajectory_id;
    common::Time time;
    transform::Rigid3d local_pose;
    std::shared_ptr<const sensor::RangeData> range_data;
    std::unique_ptr<const mapping::TrajectoryBuilderInterface::InsertionResult>
        insertion_result;
  };
  // Calling with 'nullptr' signals subscribers that the subscription has ended.
  using LocalSlamSubscriptionCallback =
      std::function<void(std::unique_ptr<LocalSlamResult>)>;
  struct Data {
    int trajectory_id;
    std::unique_ptr<sensor::Data> data;
  };
  struct SubscriptionId {
    const int trajectory_id;
    const int subscription_index;
  };

  MapBuilderContextInterface() = default;
  ~MapBuilderContextInterface() = default;

  MapBuilderContextInterface(const MapBuilderContextInterface&) = delete;
  MapBuilderContextInterface& operator=(const MapBuilderContextInterface&) =
      delete;

  virtual mapping::MapBuilderInterface& map_builder() = 0;
  virtual common::BlockingQueue<std::unique_ptr<Data>>& sensor_data_queue() = 0;
  virtual mapping::TrajectoryBuilderInterface::LocalSlamResultCallback
  GetLocalSlamResultCallbackForSubscriptions() = 0;
  virtual void AddSensorDataToTrajectory(const Data& sensor_data) = 0;
  virtual SubscriptionId SubscribeLocalSlamResults(
      int trajectory_id, LocalSlamSubscriptionCallback callback) = 0;
  virtual void UnsubscribeLocalSlamResults(
      const SubscriptionId& subscription_id) = 0;
  virtual void NotifyFinishTrajectory(int trajectory_id) = 0;
  virtual std::unique_ptr<mapping::LocalSlamResultData>
  ProcessLocalSlamResultData(
      const std::string& sensor_id, common::Time time,
      const mapping::proto::LocalSlamResultData& proto) = 0;
  virtual LocalTrajectoryUploaderInterface* local_trajectory_uploader() = 0;
  virtual void EnqueueSensorData(int trajectory_id,
                                 std::unique_ptr<sensor::Data> data) = 0;
  virtual void EnqueueLocalSlamResultData(
      int trajectory_id, const std::string& sensor_id,
      std::unique_ptr<mapping::LocalSlamResultData> local_slam_result_data) = 0;
};

}  // namespace cloud
}  // namespace cartographer

#endif  // CARTOGRAPHER_CLOUD_MAP_BUILDER_CONTEXT_INTERFACE_H
