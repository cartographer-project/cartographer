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

#include "cartographer_grpc/internal/handlers/add_fixed_frame_pose_data_handler.h"

#include "cartographer/common/make_unique.h"
#include "cartographer/sensor/dispatchable.h"
#include "cartographer/sensor/fixed_frame_pose_data.h"
#include "cartographer_grpc/internal/framework/rpc_handler.h"
#include "cartographer_grpc/internal/map_builder_context_interface.h"
#include "cartographer_grpc/internal/sensor/serialization.h"
#include "cartographer_grpc/proto/map_builder_service.pb.h"
#include "google/protobuf/empty.pb.h"

namespace cartographer_grpc {
namespace handlers {

void AddFixedFramePoseDataHandler::OnRequest(
    const proto::AddFixedFramePoseDataRequest &request) {
  // The 'BlockingQueue' returned by 'sensor_data_queue()' is already
  // thread-safe. Therefore it suffices to get an unsynchronized reference to
  // the 'MapBuilderContext'.
  GetUnsynchronizedContext<MapBuilderContextInterface>()->EnqueueSensorData(
      request.sensor_metadata().trajectory_id(),
      cartographer::sensor::MakeDispatchable(
          request.sensor_metadata().sensor_id(),
          cartographer::sensor::FromProto(request.fixed_frame_pose_data())));

  // The 'BlockingQueue' in 'LocalTrajectoryUploader' is thread-safe.
  // Therefore it suffices to get an unsynchronized reference to the
  // 'MapBuilderContext'.
  if (GetUnsynchronizedContext<MapBuilderContextInterface>()
          ->local_trajectory_uploader()) {
    auto data_request = cartographer::common::make_unique<
        proto::AddFixedFramePoseDataRequest>();
    sensor::CreateAddFixedFramePoseDataRequest(
        request.sensor_metadata().sensor_id(),
        request.sensor_metadata().trajectory_id(),
        request.fixed_frame_pose_data(), data_request.get());
    GetUnsynchronizedContext<MapBuilderContextInterface>()
        ->local_trajectory_uploader()
        ->EnqueueDataRequest(std::move(data_request));
  }
}

void AddFixedFramePoseDataHandler::OnReadsDone() {
  Send(cartographer::common::make_unique<google::protobuf::Empty>());
}

}  // namespace handlers
}  // namespace cartographer_grpc
