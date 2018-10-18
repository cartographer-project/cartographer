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

#include "cartographer/cloud/internal/local_trajectory_uploader.h"

#include <map>
#include <thread>

#include "absl/memory/memory.h"
#include "async_grpc/client.h"
#include "cartographer/cloud/internal/handlers/add_sensor_data_batch_handler.h"
#include "cartographer/cloud/internal/handlers/add_trajectory_handler.h"
#include "cartographer/cloud/internal/handlers/finish_trajectory_handler.h"
#include "cartographer/cloud/internal/sensor/serialization.h"
#include "cartographer/common/blocking_queue.h"
#include "glog/logging.h"
#include "grpc++/grpc++.h"

namespace cartographer {
namespace cloud {
namespace {

using absl::make_unique;

constexpr int kConnectionTimeoutInSeconds = 10;
constexpr int kConnectionRecoveryTimeoutInSeconds = 60;
constexpr int kTokenRefreshIntervalInSeconds = 60;
const common::Duration kPopTimeout = common::FromMilliseconds(100);

// This defines the '::grpc::StatusCode's that are considered unrecoverable
// errors and hence no retries will be attempted by the client.
const std::set<::grpc::StatusCode> kUnrecoverableStatusCodes = {
    ::grpc::DEADLINE_EXCEEDED,
    ::grpc::NOT_FOUND,
    ::grpc::UNAVAILABLE,
    ::grpc::UNKNOWN,
};

bool IsNewSubmap(const mapping::proto::Submap& submap) {
  return (submap.has_submap_2d() && submap.submap_2d().num_range_data() == 1) ||
         (submap.has_submap_3d() && submap.submap_3d().num_range_data() == 1);
}

class LocalTrajectoryUploader : public LocalTrajectoryUploaderInterface {
 public:
  struct TrajectoryInfo {
    // nullopt if uplink has not yet responded to AddTrajectoryRequest.
    absl::optional<int> uplink_trajectory_id;
    const std::set<SensorId> expected_sensor_ids;
    const mapping::proto::TrajectoryBuilderOptions trajectory_options;
    const std::string client_id;
  };

 public:
  LocalTrajectoryUploader(const std::string& uplink_server_address,
                          int batch_size, bool enable_ssl_encryption,
                          bool enable_google_auth);
  ~LocalTrajectoryUploader();

  // Starts the upload thread.
  void Start() final;

  // Shuts down the upload thread. This method blocks until the shutdown is
  // complete.
  void Shutdown() final;

  grpc::Status AddTrajectory(
      const std::string& client_id, int local_trajectory_id,
      const std::set<SensorId>& expected_sensor_ids,
      const mapping::proto::TrajectoryBuilderOptions& trajectory_options) final;
  grpc::Status FinishTrajectory(const std::string& client_id,
                                int local_trajectory_id) final;
  void EnqueueSensorData(std::unique_ptr<proto::SensorData> sensor_data) final;
  void TryRecovery();

  SensorId GetLocalSlamResultSensorId(int local_trajectory_id) const final {
    return SensorId{SensorId::SensorType::LOCAL_SLAM_RESULT,
                    "local_slam_result_" + std::to_string(local_trajectory_id)};
  }

 private:
  void ProcessSendQueue();
  // Returns 'false' for failure.
  bool TranslateTrajectoryId(proto::SensorMetadata* sensor_metadata);
  grpc::Status RegisterTrajectory(int local_trajectory_id);

  std::shared_ptr<::grpc::Channel> client_channel_;
  int batch_size_;
  std::map<int, TrajectoryInfo> local_trajectory_id_to_trajectory_info_;
  common::BlockingQueue<std::unique_ptr<proto::SensorData>> send_queue_;
  bool shutting_down_ = false;
  std::unique_ptr<std::thread> upload_thread_;
};

LocalTrajectoryUploader::LocalTrajectoryUploader(
    const std::string& uplink_server_address, int batch_size,
    bool enable_ssl_encryption, bool enable_google_auth)
    : batch_size_(batch_size) {
  auto channel_creds =
      enable_google_auth
          ? grpc::GoogleDefaultCredentials()
          : (enable_ssl_encryption
                 ? ::grpc::SslCredentials(::grpc::SslCredentialsOptions())
                 : ::grpc::InsecureChannelCredentials());

  client_channel_ = ::grpc::CreateChannel(uplink_server_address, channel_creds);
  std::chrono::system_clock::time_point deadline =
      std::chrono::system_clock::now() +
      std::chrono::seconds(kConnectionTimeoutInSeconds);
  LOG(INFO) << "Connecting to uplink " << uplink_server_address;
  if (!client_channel_->WaitForConnected(deadline)) {
    LOG(ERROR) << "Failed to connect to " << uplink_server_address;
  }
}

LocalTrajectoryUploader::~LocalTrajectoryUploader() {}

void LocalTrajectoryUploader::Start() {
  CHECK(!shutting_down_);
  CHECK(!upload_thread_);
  upload_thread_ =
      make_unique<std::thread>([this]() { this->ProcessSendQueue(); });
}

void LocalTrajectoryUploader::Shutdown() {
  CHECK(!shutting_down_);
  CHECK(upload_thread_);
  shutting_down_ = true;
  upload_thread_->join();
}

void LocalTrajectoryUploader::TryRecovery() {
  if (client_channel_->GetState(false /* try_to_connect */) !=
      grpc_connectivity_state::GRPC_CHANNEL_READY) {
    LOG(INFO) << "Trying to re-connect to uplink...";
    std::chrono::system_clock::time_point deadline =
        std::chrono::system_clock::now() +
        std::chrono::seconds(kConnectionRecoveryTimeoutInSeconds);
    if (!client_channel_->WaitForConnected(deadline)) {
      LOG(ERROR) << "Failed to re-connect to uplink prior to recovery attempt.";
      return;
    }
  }
  LOG(INFO) << "Uplink channel ready, trying recovery.";

  // Wind the sensor_data_queue forward to the next new submap.
  LOG(INFO) << "LocalTrajectoryUploader tries to recover with next submap.";
  while (true) {
    if (shutting_down_) {
      return;
    }
    proto::SensorData* sensor_data =
        send_queue_.PeekWithTimeout<proto::SensorData>(kPopTimeout);
    if (sensor_data) {
      CHECK_GE(sensor_data->local_slam_result_data().submaps_size(), 0);
      if (sensor_data->sensor_data_case() ==
              proto::SensorData::kLocalSlamResultData &&
          sensor_data->local_slam_result_data().submaps_size() > 0 &&
          IsNewSubmap(sensor_data->local_slam_result_data().submaps(
              sensor_data->local_slam_result_data().submaps_size() - 1))) {
        break;
      } else {
        send_queue_.Pop();
      }
    }
  }

  // Because the trajectories may be interrupted on the uplink side, we can no
  // longer upload to those.
  for (auto& entry : local_trajectory_id_to_trajectory_info_) {
    entry.second.uplink_trajectory_id.reset();
  }
  // TODO(gaschler): If the uplink did not restart but only the connection was
  // interrupted, this leaks trajectories in the uplink.

  // Attempt to recreate the trajectories.
  for (const auto& entry : local_trajectory_id_to_trajectory_info_) {
    grpc::Status status = RegisterTrajectory(entry.first);
    if (!status.ok()) {
      LOG(ERROR) << "Failed to create trajectory. Aborting recovery attempt. "
                 << status.error_message();
      return;
    }
  }
  LOG(INFO) << "LocalTrajectoryUploader recovered.";
}

void LocalTrajectoryUploader::ProcessSendQueue() {
  LOG(INFO) << "Starting uploader thread.";
  proto::AddSensorDataBatchRequest batch_request;
  while (!shutting_down_) {
    auto sensor_data = send_queue_.PopWithTimeout(kPopTimeout);
    if (sensor_data) {
      if (!TranslateTrajectoryId(sensor_data->mutable_sensor_metadata())) {
        batch_request.clear_sensor_data();
        TryRecovery();
        continue;
      }
      proto::SensorData* added_sensor_data = batch_request.add_sensor_data();
      *added_sensor_data = *sensor_data;

      // A submap also holds a trajectory id that must be translated to uplink's
      // trajectory id.
      if (added_sensor_data->has_local_slam_result_data()) {
        for (mapping::proto::Submap& mutable_submap :
             *added_sensor_data->mutable_local_slam_result_data()
                  ->mutable_submaps()) {
          mutable_submap.mutable_submap_id()->set_trajectory_id(
              added_sensor_data->sensor_metadata().trajectory_id());
        }
      }

      if (batch_request.sensor_data_size() == batch_size_) {
        async_grpc::Client<handlers::AddSensorDataBatchSignature> client(
            client_channel_, common::FromSeconds(kConnectionTimeoutInSeconds),
            async_grpc::CreateUnlimitedConstantDelayStrategy(
                common::FromSeconds(1), kUnrecoverableStatusCodes));
        if (client.Write(batch_request)) {
          LOG(INFO) << "Uploaded " << batch_request.ByteSize()
                    << " bytes of sensor data.";
          batch_request.clear_sensor_data();
          continue;
        }
        // Unrecoverable error occurred. Attempt recovery.
        batch_request.clear_sensor_data();
        TryRecovery();
      }
    }
  }
}

bool LocalTrajectoryUploader::TranslateTrajectoryId(
    proto::SensorMetadata* sensor_metadata) {
  auto it = local_trajectory_id_to_trajectory_info_.find(
      sensor_metadata->trajectory_id());
  if (it == local_trajectory_id_to_trajectory_info_.end()) {
    return false;
  }
  if (!it->second.uplink_trajectory_id.has_value()) {
    // Could not yet register trajectory with uplink server.
    return false;
  }
  int cloud_trajectory_id = it->second.uplink_trajectory_id.value();
  sensor_metadata->set_trajectory_id(cloud_trajectory_id);
  return true;
}

grpc::Status LocalTrajectoryUploader::AddTrajectory(
    const std::string& client_id, int local_trajectory_id,
    const std::set<SensorId>& expected_sensor_ids,
    const mapping::proto::TrajectoryBuilderOptions& trajectory_options) {
  CHECK_EQ(local_trajectory_id_to_trajectory_info_.count(local_trajectory_id),
           0);
  local_trajectory_id_to_trajectory_info_.emplace(
      local_trajectory_id,
      TrajectoryInfo{{}, expected_sensor_ids, trajectory_options, client_id});
  return RegisterTrajectory(local_trajectory_id);
}

grpc::Status LocalTrajectoryUploader::RegisterTrajectory(
    int local_trajectory_id) {
  TrajectoryInfo& trajectory_info =
      local_trajectory_id_to_trajectory_info_.at(local_trajectory_id);
  proto::AddTrajectoryRequest request;
  request.set_client_id(trajectory_info.client_id);
  *request.mutable_trajectory_builder_options() =
      trajectory_info.trajectory_options;
  for (const SensorId& sensor_id : trajectory_info.expected_sensor_ids) {
    // Range sensors are not forwarded, but combined into a LocalSlamResult.
    if (sensor_id.type != SensorId::SensorType::RANGE) {
      *request.add_expected_sensor_ids() = cloud::ToProto(sensor_id);
    }
  }
  *request.add_expected_sensor_ids() =
      cloud::ToProto(GetLocalSlamResultSensorId(local_trajectory_id));
  async_grpc::Client<handlers::AddTrajectorySignature> client(
      client_channel_, common::FromSeconds(kConnectionTimeoutInSeconds));
  ::grpc::Status status;
  if (!client.Write(request, &status)) {
    LOG(ERROR) << "Failed to register local_trajectory_id "
               << local_trajectory_id << " with uplink server. "
               << status.error_message();
    return status;
  }
  LOG(INFO) << "Created trajectory for client_id: " << trajectory_info.client_id
            << " local trajectory_id: " << local_trajectory_id
            << " uplink trajectory_id: " << client.response().trajectory_id();
  trajectory_info.uplink_trajectory_id = client.response().trajectory_id();
  return status;
}

grpc::Status LocalTrajectoryUploader::FinishTrajectory(
    const std::string& client_id, int local_trajectory_id) {
  auto it = local_trajectory_id_to_trajectory_info_.find(local_trajectory_id);
  if (it == local_trajectory_id_to_trajectory_info_.end()) {
    return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT,
                        "local_trajectory_id has not been"
                        " registered with AddTrajectory.");
  }
  auto cloud_trajectory_id = it->second.uplink_trajectory_id;
  if (!cloud_trajectory_id.has_value()) {
    return grpc::Status(
        grpc::StatusCode::UNAVAILABLE,
        "trajectory_id has not been created in uplink, ignoring.");
  }
  proto::FinishTrajectoryRequest request;
  request.set_client_id(client_id);
  request.set_trajectory_id(cloud_trajectory_id.value());
  async_grpc::Client<handlers::FinishTrajectorySignature> client(
      client_channel_, common::FromSeconds(kConnectionTimeoutInSeconds));
  grpc::Status status;
  client.Write(request, &status);
  return status;
}

void LocalTrajectoryUploader::EnqueueSensorData(
    std::unique_ptr<proto::SensorData> sensor_data) {
  send_queue_.Push(std::move(sensor_data));
}

}  // namespace

std::unique_ptr<LocalTrajectoryUploaderInterface> CreateLocalTrajectoryUploader(
    const std::string& uplink_server_address, int batch_size,
    bool enable_ssl_encryption, bool enable_google_auth) {
  return make_unique<LocalTrajectoryUploader>(uplink_server_address, batch_size,
                                              enable_ssl_encryption,
                                              enable_google_auth);
}

}  // namespace cloud
}  // namespace cartographer
