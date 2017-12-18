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

cartographer::transform::Rigid3d PoseGraphStub::GetLocalToGlobalTransform(
    int trajectory_id) {
  LOG(FATAL) << "Not implemented";
}

cartographer::mapping::MapById<cartographer::mapping::NodeId,
                               cartographer::mapping::TrajectoryNode>
PoseGraphStub::GetTrajectoryNodes() {
  LOG(FATAL) << "Not implemented";
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
