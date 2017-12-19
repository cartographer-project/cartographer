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

#ifndef CARTOGRAPHER_GRPC_HANDLERS_RECEIVE_LOCAL_SLAM_RESULTS_HANDLER_H
#define CARTOGRAPHER_GRPC_HANDLERS_RECEIVE_LOCAL_SLAM_RESULTS_HANDLER_H

#include "cartographer/common/make_unique.h"
#include "cartographer_grpc/framework/rpc_handler.h"
#include "cartographer_grpc/map_builder_server.h"
#include "cartographer_grpc/proto/map_builder_service.pb.h"

namespace cartographer_grpc {
namespace handlers {

class ReceiveLocalSlamResultsHandler
    : public framework::RpcHandler<
          proto::ReceiveLocalSlamResultsRequest,
          framework::Stream<proto::ReceiveLocalSlamResultsResponse>> {
 public:
  void OnRequest(
      const proto::ReceiveLocalSlamResultsRequest& request) override {
    auto writer = GetWriter();
    MapBuilderServer::SubscriptionId subscription_id =
        GetUnsynchronizedContext<MapBuilderServer::MapBuilderContext>()
            ->SubscribeLocalSlamResults(
                request.trajectory_id(),
                [writer](int trajectory_id, cartographer::common::Time time,
                         cartographer::transform::Rigid3d local_pose,
                         std::shared_ptr<const cartographer::sensor::RangeData>
                             range_data,
                         std::unique_ptr<const cartographer::mapping::NodeId>
                             node_id) {
                  writer(GenerateResponse(trajectory_id, time, local_pose,
                                          range_data, std::move(node_id)));
                });

    subscription_id_ =
        cartographer::common::make_unique<MapBuilderServer::SubscriptionId>(
            subscription_id);
  }

  static std::unique_ptr<proto::ReceiveLocalSlamResultsResponse>
  GenerateResponse(
      int trajectory_id, cartographer::common::Time time,
      const cartographer::transform::Rigid3d& local_pose,
      std::shared_ptr<const cartographer::sensor::RangeData> range_data,
      std::unique_ptr<const cartographer::mapping::NodeId> node_id) {
    auto response = cartographer::common::make_unique<
        proto::ReceiveLocalSlamResultsResponse>();
    response->set_trajectory_id(trajectory_id);
    response->set_timestamp(cartographer::common::ToUniversal(time));
    *response->mutable_local_pose() =
        cartographer::transform::ToProto(local_pose);
    if (range_data) {
      *response->mutable_range_data() =
          cartographer::sensor::ToProto(*range_data);
    }
    if (node_id) {
      response->mutable_node_id()->set_trajectory_id(node_id->trajectory_id);
      response->mutable_node_id()->set_node_index(node_id->node_index);
    }
    return response;
  }

  void OnFinish() override {
    if (subscription_id_) {
      GetUnsynchronizedContext<MapBuilderServer::MapBuilderContext>()
          ->UnsubscribeLocalSlamResults(*subscription_id_);
    }
  }

 private:
  std::unique_ptr<MapBuilderServer::SubscriptionId> subscription_id_;
};

}  // namespace handlers
}  // namespace cartographer_grpc

#endif  // CARTOGRAPHER_GRPC_HANDLERS_RECEIVE_LOCAL_SLAM_RESULTS_HANDLER_H
