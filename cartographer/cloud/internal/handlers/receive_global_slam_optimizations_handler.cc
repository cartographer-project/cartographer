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

#include "cartographer/cloud/internal/handlers/receive_global_slam_optimizations_handler.h"

#include "absl/memory/memory.h"
#include "async_grpc/rpc_handler.h"
#include "cartographer/cloud/internal/map_builder_context_interface.h"
#include "cartographer/cloud/proto/map_builder_service.pb.h"
#include "cartographer/transform/transform.h"

namespace cartographer {
namespace cloud {
namespace handlers {
namespace {

std::unique_ptr<proto::ReceiveGlobalSlamOptimizationsResponse> GenerateResponse(
    const std::map<int, mapping::SubmapId> &last_optimized_submap_ids,
    const std::map<int, mapping::NodeId> &last_optimized_node_ids) {
  auto response =
      absl::make_unique<proto::ReceiveGlobalSlamOptimizationsResponse>();
  for (const auto &entry : last_optimized_submap_ids) {
    entry.second.ToProto(
        &(*response->mutable_last_optimized_submap_ids())[entry.first]);
  }
  for (const auto &entry : last_optimized_node_ids) {
    entry.second.ToProto(
        &(*response->mutable_last_optimized_node_ids())[entry.first]);
  }
  return response;
}

}  // namespace

void ReceiveGlobalSlamOptimizationsHandler::OnRequest(
    const google::protobuf::Empty &request) {
  auto writer = GetWriter();
  const int subscription_index =
      GetUnsynchronizedContext<MapBuilderContextInterface>()
          ->SubscribeGlobalSlamOptimizations(
              [writer](const std::map<int, mapping::SubmapId>
                           &last_optimized_submap_ids,
                       const std::map<int, mapping::NodeId>
                           &last_optimized_node_ids) {
                if (!writer.Write(GenerateResponse(last_optimized_submap_ids,
                                                   last_optimized_node_ids))) {
                  // Client closed connection.
                  LOG(INFO) << "Client closed connection.";
                  return false;
                }
                return true;
              });

  LOG(INFO) << "Added subscription: " << subscription_index;
  subscription_index_ = subscription_index;
}

void ReceiveGlobalSlamOptimizationsHandler::OnFinish() {
  if (subscription_index_.has_value()) {
    LOG(INFO) << "Removing subscription " << subscription_index_.value();
    GetUnsynchronizedContext<MapBuilderContextInterface>()
        ->UnsubscribeGlobalSlamOptimizations(subscription_index_.value());
  }
}

}  // namespace handlers
}  // namespace cloud
}  // namespace cartographer
