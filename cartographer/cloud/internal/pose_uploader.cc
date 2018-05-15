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

#include "cartographer/cloud/internal/pose_uploader.h"
#include "async_grpc/client.h"
#include "cartographer/common/make_unique.h"

namespace cartographer {
namespace cloud {
namespace {

using common::make_unique;

constexpr int kConnectionTimeoutInSecond = 10;

DEFINE_HANDLER_SIGNATURE(AddPoseBatchSignature, proto::AddPoseBatchRequest,
                         google::protobuf::Empty,
                         "/cartographer.cloud.proto.PoseService/AddPoseBatch")

class PoseUploader : public PoseUploaderInterface {
 public:
  PoseUploader(const std::string &pose_server_address, int batch_size,
               bool enable_ssl_encryption);
  void EnqueueLocalPose(common::Time time, int trajectory_id,
                        const transform::Rigid3d &local_pose) override;
  void EnqueueGlobalPose(common::Time time, const mapping::NodeId &node_id,
                         const transform::Rigid3d &local_pose,
                         const transform::Rigid3d &global_pose) override;
  void SetActiveTrajectory(int trajectory_id) override;
  int active_trajectory_id() override { return active_trajectory_id_; }
  mapping::NodeId last_global_node_id() override {
    return last_global_node_id_;
  }

 private:
  std::shared_ptr<::grpc::Channel> client_channel_;
  int batch_size_;
  int active_trajectory_id_ = -1;
  mapping::NodeId last_global_node_id_{-1, -1};
};

PoseUploader::PoseUploader(const std::string &pose_server_address,
                           int batch_size, bool enable_ssl_encryption)
    : client_channel_(::grpc::CreateChannel(
          pose_server_address,
          enable_ssl_encryption
              ? ::grpc::SslCredentials(::grpc::SslCredentialsOptions())
              : ::grpc::InsecureChannelCredentials())),
      batch_size_(batch_size) {}

void PoseUploader::EnqueueLocalPose(common::Time time, int trajectory_id,
                                    const transform::Rigid3d &local_pose) {
  CHECK_EQ(trajectory_id, active_trajectory_id_);
  // TODO(cschuet): Enqueue pose for uploading.
}

void PoseUploader::EnqueueGlobalPose(common::Time time,
                                     const mapping::NodeId &node_id,
                                     const transform::Rigid3d &local_pose,
                                     const transform::Rigid3d &global_pose) {
  CHECK_EQ(node_id.trajectory_id, active_trajectory_id_);
  last_global_node_id_.node_index = node_id.node_index;
  // TODO(cschuet): Enqueue pose for uploading.
}

void PoseUploader::SetActiveTrajectory(int trajectory_id) {
  active_trajectory_id_ = trajectory_id;
  last_global_node_id_ = mapping::NodeId{trajectory_id, -1};
}

}  // namespace

std::unique_ptr<PoseUploaderInterface> CreatePoseUploader(
    const std::string &pose_server_address, int batch_size,
    bool enable_ssl_encryption) {
  return common::make_unique<PoseUploader>(pose_server_address, batch_size,
                                           enable_ssl_encryption);
}

}  // namespace cloud
}  // namespace cartographer
