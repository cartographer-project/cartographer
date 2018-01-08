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

#include "cartographer_grpc/mapping/pose_graph_stub.h"

#include "glog/logging.h"

namespace cartographer_grpc {
namespace mapping {

PoseGraphStub::PoseGraphStub(std::shared_ptr<grpc::Channel> client_channel,
                             proto::MapBuilderService::Stub* stub)
    : client_channel_(client_channel), stub_(stub) {}

void PoseGraphStub::RunFinalOptimization() { LOG(FATAL) << "Not implemented"; }

cartographer::mapping::MapById<
    cartographer::mapping::SubmapId,
    cartographer::mapping::PoseGraphInterface::SubmapData>
PoseGraphStub::GetAllSubmapData() {
  LOG(FATAL) << "Not implemented";
}

cartographer::mapping::MapById<
    cartographer::mapping::SubmapId,
    cartographer::mapping::PoseGraphInterface::SubmapPose>
PoseGraphStub::GetAllSubmapPoses() {
  grpc::ClientContext client_context;
  google::protobuf::Empty request;
  proto::GetAllSubmapPosesResponse response;
  stub_->GetAllSubmapPoses(&client_context, request, &response);
  cartographer::mapping::MapById<
      cartographer::mapping::SubmapId,
      cartographer::mapping::PoseGraphInterface::SubmapPose>
      submap_poses;
  for (const auto& submap_pose : response.submap_poses()) {
    submap_poses.Insert(
        cartographer::mapping::SubmapId{submap_pose.submap_id().trajectory_id(),
                                        submap_pose.submap_id().submap_index()},
        cartographer::mapping::PoseGraphInterface::SubmapPose{
            submap_pose.submap_version(),
            cartographer::transform::ToRigid3(submap_pose.global_pose())});
  }
  return submap_poses;
}

cartographer::transform::Rigid3d PoseGraphStub::GetLocalToGlobalTransform(
    int trajectory_id) {
  LOG(FATAL) << "Not implemented";
}

cartographer::mapping::MapById<cartographer::mapping::NodeId,
                               cartographer::mapping::TrajectoryNode>
PoseGraphStub::GetTrajectoryNodes() {
  LOG(FATAL) << "Not implemented";
}

cartographer::mapping::MapById<cartographer::mapping::NodeId,
                               cartographer::mapping::TrajectoryNodePose>
PoseGraphStub::GetTrajectoryNodePoses() {
  grpc::ClientContext client_context;
  google::protobuf::Empty request;
  proto::GetTrajectoryNodePosesResponse response;
  stub_->GetTrajectoryNodePoses(&client_context, request, &response);
  cartographer::mapping::MapById<cartographer::mapping::NodeId,
                                 cartographer::mapping::TrajectoryNodePose>
      node_poses;
  for (const auto& node_pose : response.node_poses()) {
    node_poses.Insert(
        cartographer::mapping::NodeId{node_pose.node_id().trajectory_id(),
                                      node_pose.node_id().node_index()},
        cartographer::mapping::TrajectoryNodePose{
            node_pose.has_constant_data(),
            cartographer::transform::ToRigid3(node_pose.global_pose())});
  }
  return node_poses;
}

bool PoseGraphStub::IsTrajectoryFinished(int trajectory_id) {
  LOG(FATAL) << "Not implemented";
}

std::vector<cartographer::mapping::PoseGraphInterface::Constraint>
PoseGraphStub::constraints() {
  LOG(FATAL) << "Not implemented";
}

}  // namespace mapping
}  // namespace cartographer_grpc
