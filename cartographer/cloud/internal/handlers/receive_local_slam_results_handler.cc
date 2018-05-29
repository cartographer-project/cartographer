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

#include "cartographer/cloud/internal/handlers/receive_local_slam_results_handler.h"

#include "async_grpc/rpc_handler.h"
#include "cartographer/cloud/internal/map_builder_context_interface.h"
#include "cartographer/cloud/proto/map_builder_service.pb.h"
#include "cartographer/common/make_unique.h"
#include "cartographer/transform/transform.h"

namespace cartographer {
namespace cloud {
namespace handlers {
namespace {

std::unique_ptr<proto::ReceiveLocalSlamResultsResponse> GenerateResponse(
    std::unique_ptr<MapBuilderContextInterface::LocalSlamResult>
        local_slam_result) {
  auto response = common::make_unique<proto::ReceiveLocalSlamResultsResponse>();
  response->set_trajectory_id(local_slam_result->trajectory_id);
  response->set_timestamp(common::ToUniversal(local_slam_result->time));
  *response->mutable_local_pose() =
      transform::ToProto(local_slam_result->local_pose);
  if (local_slam_result->range_data) {
    *response->mutable_range_data() =
        sensor::ToProto(*local_slam_result->range_data);
  }
  if (local_slam_result->insertion_result) {
    local_slam_result->insertion_result->node_id.ToProto(
        response->mutable_insertion_result()->mutable_node_id());
  }
  return response;
}

}  // namespace

void ReceiveLocalSlamResultsHandler::OnRequest(
    const proto::ReceiveLocalSlamResultsRequest& request) {
  auto writer = GetWriter();
  MapBuilderContextInterface::LocalSlamSubscriptionId subscription_id =
      GetUnsynchronizedContext<MapBuilderContextInterface>()
          ->SubscribeLocalSlamResults(
              request.trajectory_id(),
              [writer](
                  std::unique_ptr<MapBuilderContextInterface::LocalSlamResult>
                      local_slam_result) {
                if (local_slam_result) {
                  if (!writer.Write(
                          GenerateResponse(std::move(local_slam_result)))) {
                    // Client closed connection.
                    LOG(INFO) << "Client closed connection.";
                    return false;
                  }
                } else {
                  // Callback with 'nullptr' signals that the trajectory
                  // finished.
                  writer.WritesDone();
                }
                return true;
              });

  subscription_id_ =
      common::make_unique<MapBuilderContextInterface::LocalSlamSubscriptionId>(
          subscription_id);
}

void ReceiveLocalSlamResultsHandler::OnFinish() {
  if (subscription_id_) {
    GetUnsynchronizedContext<MapBuilderContextInterface>()
        ->UnsubscribeLocalSlamResults(*subscription_id_);
  }
}

}  // namespace handlers
}  // namespace cloud
}  // namespace cartographer
