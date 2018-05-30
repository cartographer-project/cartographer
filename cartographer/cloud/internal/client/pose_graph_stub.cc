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

#include "cartographer/cloud/internal/client/pose_graph_stub.h"
#include "async_grpc/client.h"
#include "cartographer/cloud/internal/handlers/get_all_submap_poses.h"
#include "cartographer/cloud/internal/handlers/get_constraints_handler.h"
#include "cartographer/cloud/internal/handlers/get_landmark_poses_handler.h"
#include "cartographer/cloud/internal/handlers/get_local_to_global_transform_handler.h"
#include "cartographer/cloud/internal/handlers/get_trajectory_node_poses_handler.h"
#include "cartographer/cloud/internal/handlers/is_trajectory_finished_handler.h"
#include "cartographer/cloud/internal/handlers/is_trajectory_frozen_handler.h"
#include "cartographer/cloud/internal/handlers/run_final_optimization_handler.h"
#include "cartographer/cloud/internal/handlers/set_landmark_pose_handler.h"
#include "cartographer/mapping/pose_graph.h"
#include "cartographer/transform/transform.h"
#include "glog/logging.h"

namespace cartographer {
namespace cloud {

PoseGraphStub::PoseGraphStub(std::shared_ptr<::grpc::Channel> client_channel)
    : client_channel_(client_channel) {}

void PoseGraphStub::RunFinalOptimization() {
  google::protobuf::Empty request;
  async_grpc::Client<handlers::RunFinalOptimizationSignature> client(
      client_channel_);
  CHECK(client.Write(request));
}

mapping::MapById<mapping::SubmapId, mapping::PoseGraphInterface::SubmapData>
PoseGraphStub::GetAllSubmapData() const {
  LOG(FATAL) << "Not implemented";
}

mapping::MapById<mapping::SubmapId, mapping::PoseGraphInterface::SubmapPose>
PoseGraphStub::GetAllSubmapPoses() const {
  google::protobuf::Empty request;
  async_grpc::Client<handlers::GetAllSubmapPosesSignature> client(
      client_channel_);
  CHECK(client.Write(request));
  mapping::MapById<mapping::SubmapId, mapping::PoseGraphInterface::SubmapPose>
      submap_poses;
  for (const auto& submap_pose : client.response().submap_poses()) {
    submap_poses.Insert(
        mapping::SubmapId{submap_pose.submap_id().trajectory_id(),
                          submap_pose.submap_id().submap_index()},
        mapping::PoseGraphInterface::SubmapPose{
            submap_pose.submap_version(),
            transform::ToRigid3(submap_pose.global_pose())});
  }
  return submap_poses;
}

transform::Rigid3d PoseGraphStub::GetLocalToGlobalTransform(
    int trajectory_id) const {
  proto::GetLocalToGlobalTransformRequest request;
  request.set_trajectory_id(trajectory_id);
  async_grpc::Client<handlers::GetLocalToGlobalTransformSignature> client(
      client_channel_);
  CHECK(client.Write(request));
  return transform::ToRigid3(client.response().local_to_global());
}

mapping::MapById<mapping::NodeId, mapping::TrajectoryNode>
PoseGraphStub::GetTrajectoryNodes() const {
  LOG(FATAL) << "Not implemented";
}

mapping::MapById<mapping::NodeId, mapping::TrajectoryNodePose>
PoseGraphStub::GetTrajectoryNodePoses() const {
  google::protobuf::Empty request;
  async_grpc::Client<handlers::GetTrajectoryNodePosesSignature> client(
      client_channel_);
  CHECK(client.Write(request));
  mapping::MapById<mapping::NodeId, mapping::TrajectoryNodePose> node_poses;
  for (const auto& node_pose : client.response().node_poses()) {
    common::optional<mapping::TrajectoryNodePose::ConstantPoseData>
        constant_pose_data;
    if (node_pose.has_constant_pose_data()) {
      constant_pose_data = mapping::TrajectoryNodePose::ConstantPoseData{
          common::FromUniversal(node_pose.constant_pose_data().timestamp()),
          transform::ToRigid3(node_pose.constant_pose_data().local_pose())};
    }
    node_poses.Insert(
        mapping::NodeId{node_pose.node_id().trajectory_id(),
                        node_pose.node_id().node_index()},
        mapping::TrajectoryNodePose{
            transform::ToRigid3(node_pose.global_pose()), constant_pose_data});
  }
  return node_poses;
}

std::map<std::string, transform::Rigid3d> PoseGraphStub::GetLandmarkPoses()
    const {
  google::protobuf::Empty request;
  async_grpc::Client<handlers::GetLandmarkPosesSignature> client(
      client_channel_);
  CHECK(client.Write(request));
  std::map<std::string, transform::Rigid3d> landmark_poses;
  for (const auto& landmark_pose : client.response().landmark_poses()) {
    landmark_poses[landmark_pose.landmark_id()] =
        transform::ToRigid3(landmark_pose.global_pose());
  }
  return landmark_poses;
}

void PoseGraphStub::SetLandmarkPose(const std::string& landmark_id,
                                    const transform::Rigid3d& global_pose) {
  proto::SetLandmarkPoseRequest request;
  request.mutable_landmark_pose()->set_landmark_id(landmark_id);
  *request.mutable_landmark_pose()->mutable_global_pose() =
      transform::ToProto(global_pose);
  async_grpc::Client<handlers::SetLandmarkPoseSignature> client(
      client_channel_);
  CHECK(client.Write(request));
}

bool PoseGraphStub::IsTrajectoryFinished(int trajectory_id) const {
  proto::IsTrajectoryFinishedRequest request;
  request.set_trajectory_id(trajectory_id);
  async_grpc::Client<handlers::IsTrajectoryFinishedSignature> client(
      client_channel_);
  CHECK(client.Write(request));
  return client.response().is_finished();
}

bool PoseGraphStub::IsTrajectoryFrozen(int trajectory_id) const {
  proto::IsTrajectoryFrozenRequest request;
  request.set_trajectory_id(trajectory_id);
  async_grpc::Client<handlers::IsTrajectoryFrozenSignature> client(
      client_channel_);
  CHECK(client.Write(request));
  return client.response().is_frozen();
}

std::map<int, mapping::PoseGraphInterface::TrajectoryData>
PoseGraphStub::GetTrajectoryData() const {
  LOG(FATAL) << "Not implemented";
}

std::vector<mapping::PoseGraphInterface::Constraint>
PoseGraphStub::constraints() const {
  google::protobuf::Empty request;
  async_grpc::Client<handlers::GetConstraintsSignature> client(client_channel_);
  CHECK(client.Write(request));
  return mapping::FromProto(client.response().constraints());
}

mapping::proto::PoseGraph PoseGraphStub::ToProto() const {
  LOG(FATAL) << "Not implemented";
}

void PoseGraphStub::SetGlobalSlamOptimizationCallback(
    GlobalSlamOptimizationCallback callback) {
  LOG(FATAL) << "Not implemented";
}

}  // namespace cloud
}  // namespace cartographer
