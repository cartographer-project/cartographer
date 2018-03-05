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

#include "cartographer/cloud/internal/handlers/add_local_slam_result_data_handler.h"

#include "cartographer/cloud/internal/framework/rpc_handler.h"
#include "cartographer/cloud/internal/map_builder_context_interface.h"
#include "cartographer/cloud/proto/map_builder_service.pb.h"
#include "cartographer/common/make_unique.h"
#include "cartographer/mapping/local_slam_result_data.h"
#include "cartographer/mapping/trajectory_node.h"
#include "cartographer/sensor/dispatchable.h"
#include "google/protobuf/empty.pb.h"

namespace cartographer {
namespace cloud {
namespace handlers {

void AddLocalSlamResultDataHandler::OnRequest(
    const proto::AddLocalSlamResultDataRequest& request) {
  auto local_slam_result_data =
      GetContext<MapBuilderContextInterface>()->ProcessLocalSlamResultData(
          request.sensor_metadata().sensor_id(),
          common::FromUniversal(request.local_slam_result_data().timestamp()),
          request.local_slam_result_data());
  GetContext<MapBuilderContextInterface>()->EnqueueLocalSlamResultData(
      request.sensor_metadata().trajectory_id(),
      request.sensor_metadata().sensor_id(), std::move(local_slam_result_data));
}

void AddLocalSlamResultDataHandler::OnReadsDone() {
  Send(common::make_unique<google::protobuf::Empty>());
}

}  // namespace handlers
}  // namespace cloud
}  // namespace cartographer
