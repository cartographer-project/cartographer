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
  std::string method_name() const override {
    return "/cartographer_grpc.proto.MapBuilderService/ReceiveLocalSlamResults";
  }
  void OnRequest(
      const proto::ReceiveLocalSlamResultsRequest& request) override {
    auto writer = GetWriter();
    MapBuilderContextInterface::SubscriptionId subscription_id =
        GetUnsynchronizedContext<MapBuilderContext>()
            ->SubscribeLocalSlamResults(
                request.trajectory_id(),
                [writer](
                    std::unique_ptr<MapBuilderContextInterface::LocalSlamResult>
                        local_slam_result) {
                  if (local_slam_result) {
                    writer.Write(
                        GenerateResponse(std::move(local_slam_result)));
                  } else {
                    // Callback with 'nullptr' signals that the trajectory
                    // finished.
                    writer.WritesDone();
                  }
                });

    subscription_id_ = cartographer::common::make_unique<
        MapBuilderContextInterface::SubscriptionId>(subscription_id);
  }

  static std::unique_ptr<proto::ReceiveLocalSlamResultsResponse>
  GenerateResponse(std::unique_ptr<MapBuilderContextInterface::LocalSlamResult>
                       local_slam_result) {
    auto response = cartographer::common::make_unique<
        proto::ReceiveLocalSlamResultsResponse>();
    response->set_trajectory_id(local_slam_result->trajectory_id);
    response->set_timestamp(
        cartographer::common::ToUniversal(local_slam_result->time));
    *response->mutable_local_pose() =
        cartographer::transform::ToProto(local_slam_result->local_pose);
    if (local_slam_result->range_data) {
      *response->mutable_range_data() =
          cartographer::sensor::ToProto(*local_slam_result->range_data);
    }
    if (local_slam_result->insertion_result) {
      local_slam_result->insertion_result->node_id.ToProto(
          response->mutable_insertion_result()->mutable_node_id());
    }
    return response;
  }

  void OnFinish() override {
    if (subscription_id_) {
      GetUnsynchronizedContext<MapBuilderContext>()
          ->UnsubscribeLocalSlamResults(*subscription_id_);
    }
  }

 private:
  std::unique_ptr<MapBuilderContextInterface::SubscriptionId> subscription_id_;
};

}  // namespace handlers
}  // namespace cartographer_grpc

#endif  // CARTOGRAPHER_GRPC_HANDLERS_RECEIVE_LOCAL_SLAM_RESULTS_HANDLER_H
