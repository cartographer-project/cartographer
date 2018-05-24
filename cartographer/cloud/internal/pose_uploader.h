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

#ifndef CARTOGRAPHER_CLOUD_INTERNAL_POSE_UPLOADER_H
#define CARTOGRAPHER_CLOUD_INTERNAL_POSE_UPLOADER_H

#include "cartographer/cloud/proto/pose_service.pb.h"
#include "cartographer/common/optional.h"
#include "cartographer/mapping/id.h"
#include "cartographer/transform/rigid_transform.h"

namespace cartographer {
namespace cloud {

class PoseUploaderInterface {
 public:
  virtual void EnqueueLocalPose(common::Time time, int trajectory_id,
                                const transform::Rigid3d &local_pose) = 0;
  virtual void EnqueueGlobalPose(common::Time time,
                                 const mapping::NodeId &node_id,
                                 const transform::Rigid3d &local_pose,
                                 const transform::Rigid3d &global_pose) = 0;
  virtual void SetActiveTrajectory(int trajectory_id) = 0;
  virtual int active_trajectory_id() = 0;
  virtual mapping::NodeId last_global_node_id() = 0;
};

// Returns PoseUploader with the actual implementation.
std::unique_ptr<PoseUploaderInterface> CreatePoseUploader(
    const std::string &pose_server_address, int batch_size,
    bool enable_ssl_encryption);

}  // namespace cloud
}  // namespace cartographer

#endif  // CARTOGRAPHER_CLOUD_INTERNAL_POSE_UPLOADER_H
