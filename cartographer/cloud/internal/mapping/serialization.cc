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

#include "cartographer/cloud/internal/mapping/serialization.h"

#include "cartographer/common/port.h"

namespace cartographer {
namespace cloud {

proto::TrajectoryRemapping ToProto(
    const std::map<int, int>& trajectory_remapping) {
  proto::TrajectoryRemapping proto;
  *proto.mutable_serialized_trajectories_to_trajectories() =
      google::protobuf::Map<int32, int32>(trajectory_remapping.begin(),
                                          trajectory_remapping.end());
  return proto;
}

std::map<int, int> FromProto(const proto::TrajectoryRemapping& proto) {
  return std::map<int, int>(
      proto.serialized_trajectories_to_trajectories().begin(),
      proto.serialized_trajectories_to_trajectories().end());
}

}  // namespace cloud
}  // namespace cartographer
