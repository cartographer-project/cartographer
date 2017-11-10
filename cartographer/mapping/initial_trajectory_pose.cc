/*
 * Copyright 2016 The Cartographer Authors
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

#include "cartographer/mapping/initial_trajectory_pose.h"

#include "cartographer/transform/transform.h"

namespace cartographer {
namespace mapping {

proto::InitialTrajectoryPose CreateInitialTrajectoryPose(
    common::LuaParameterDictionary* const parameter_dictionary) {
  proto::InitialTrajectoryPose options;
  options.set_to_trajectory_id(
      parameter_dictionary->GetInt("to_trajectory_id"));
  *options.mutable_relative_pose() =
      transform::ToProto(transform::FromDictionary(
          +parameter_dictionary->GetDictionary("relative_pose").get()));
  options.set_timestamp(parameter_dictionary->GetInt("timestamp"));
  return options;
}

InitialTrajectoryPose ToInitialTrajectoryPose(
    const proto::InitialTrajectoryPose& initial_pose) {
  return {initial_pose.to_trajectory_id(),
          transform::ToRigid3(initial_pose.relative_pose()),
          common::FromUniversal(initial_pose.timestamp())};
}

proto::InitialTrajectoryPose ToProto(
    const InitialTrajectoryPose& initial_pose) {
  proto::InitialTrajectoryPose proto;
  *proto.mutable_relative_pose() =
      transform::ToProto(initial_pose.relative_pose);
  proto.set_to_trajectory_id(initial_pose.to_trajectory_id);
  proto.set_timestamp(common::ToUniversal(initial_pose.time));
  return proto;
}

}  // namespace mapping
}  // namespace cartographer
