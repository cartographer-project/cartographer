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
#include "async_grpc/token_file_credentials.h"
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
constexpr int kTokenRefreshIntervalInSeconds = 60;
const common::Duration kPopTimeout = common::FromMilliseconds(100);

// This defines the '::grpc::StatusCode's that are considered unrecoverable
// errors and hence no retries will be attempted by the client.
const std::set<::grpc::StatusCode> kUnrecoverableStatusCodes = {
    ::grpc::NOT_FOUND};

bool IsNewSubmap(const mapping::proto::Submap& submap) {
  return (submap.has_submap_2d() && submap.submap_2d().num_range_data() == 1) ||
         (submap.has_submap_3d() && submap.submap_3d().num_range_data() == 1);
}

class LocalTrajectoryUploader : public LocalTrajectoryUploaderInterface {
 public:
  struct TrajectoryInfo {
    TrajectoryInfo() = default;
    TrajectoryInfo(
        const int uplink_trajectory_id,
        const std::set<SensorId>& expected_sensor_ids,
        const mapping::proto::TrajectoryBuilderOptions& trajectory_options,
        const std::string& client_id)
        : uplink_trajectory_id(uplink_trajectory_id),
          expected_sensor_ids(expected_sensor_ids),
          trajectory_options(trajectory_options),
          client_id(client_id) {}
    const int uplink_trajectory_id;
    const std::set<SensorId> expected_sensor_ids;
    const mapping::proto::TrajectoryBuilderOptions trajectory_options;
    const std::string client_id;
  };

 public:
  LocalTrajectoryUploader(const std::string& uplink_server_address,
                          int batch_size, bool enable_ssl_encryption,
                          const std::string& token_file_path);
  ~LocalTrajectoryUploader();

  // Starts the upload thread.
  void Start() final;

  // Shuts down the upload thread. This method blocks until the shutdown is
  // complete.
  void Shutdown() final;

  bool AddTrajectory(
      const std::string& client_id, int local_trajectory_id,
      const std::set<SensorId>& expected_sensor_ids,
      const mapping::proto::TrajectoryBuilderOptions& trajectory_options) final;
  void FinishTrajectory(const std::string& client_id,
                        int local_trajectory_id) final;
  void EnqueueSensorData(std::unique_ptr<proto::SensorData> sensor_data) final;
  void TryRecovery();

  SensorId GetLocalSlamResultSensorId(int local_trajectory_id) const final {
    return SensorId{SensorId::SensorType::LOCAL_SLAM_RESULT,
                    "local_slam_result_" + std::to_string(local_trajectory_id)};
  }

 private:
  void ProcessSendQueue();
  void TranslateTrajectoryId(proto::SensorMetadata* sensor_metadata);

  std::shared_ptr<::grpc::Channel> client_channel_;
  int batch_size_;
  std::map<int, TrajectoryInfo> local_trajectory_id_to_trajectory_info_;
  common::BlockingQueue<std::unique_ptr<proto::SensorData>> send_queue_;
  bool shutting_down_ = false;
  std::unique_ptr<std::thread> upload_thread_;
};

LocalTrajectoryUploader::LocalTrajectoryUploader(
    const std::string& uplink_server_address, int batch_size,
    bool enable_ssl_encryption, const std::string& token_file_path)
    : batch_size_(batch_size) {
  auto channel_creds =
      enable_ssl_encryption
          ? ::grpc::SslCredentials(::grpc::SslCredentialsOptions())
          : ::grpc::InsecureChannelCredentials();

  if (!token_file_path.empty()) {
    auto call_creds = async_grpc::TokenFileCredentials(
        token_file_path, std::chrono::seconds(kTokenRefreshIntervalInSeconds));
    channel_creds =
        grpc::CompositeChannelCredentials(channel_creds, call_creds);
  }
  client_channel_ = ::grpc::CreateChannel(uplink_server_address, channel_creds);
  std::chrono::system_clock::time_point deadline(
      std::chrono::system_clock::now() +
      std::chrono::seconds(kConnectionTimeoutInSeconds));
  LOG(INFO) << "Connecting to uplink " << uplink_server_address;
  if (!client_channel_->WaitForConnected(deadline)) {
    LOG(FATAL) << "Failed to connect to " << uplink_server_address;
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
  // Wind the sensor_data_queue forward to the next new submap.
  while (true) {
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

  // Attempt to recreate the trajectories.
  const auto local_trajectory_id_to_trajectory_info =
      local_trajectory_id_to_trajectory_info_;
  local_trajectory_id_to_trajectory_info_.clear();
  for (const auto& entry : local_trajectory_id_to_trajectory_info) {
    if (!AddTrajectory(entry.second.client_id, entry.first,
                       entry.second.expected_sensor_ids,
                       entry.second.trajectory_options)) {
      LOG(ERROR) << "Failed to create trajectory. Aborting recovery attempt.";
      return;
    }
  }
}

void LocalTrajectoryUploader::ProcessSendQueue() {
  LOG(INFO) << "Starting uploader thread.";
  proto::AddSensorDataBatchRequest batch_request;
  while (!shutting_down_) {
    auto sensor_data = send_queue_.PopWithTimeout(kPopTimeout);
    if (sensor_data) {
      proto::SensorData* added_sensor_data = batch_request.add_sensor_data();
      *added_sensor_data = *sensor_data;
      TranslateTrajectoryId(added_sensor_data->mutable_sensor_metadata());

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
            client_channel_, common::FromSeconds(10),
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

void LocalTrajectoryUploader::TranslateTrajectoryId(
    proto::SensorMetadata* sensor_metadata) {
  int cloud_trajectory_id = local_trajectory_id_to_trajectory_info_
                                .at(sensor_metadata->trajectory_id())
                                .uplink_trajectory_id;
  sensor_metadata->set_trajectory_id(cloud_trajectory_id);
}

bool LocalTrajectoryUploader::AddTrajectory(
    const std::string& client_id, int local_trajectory_id,
    const std::set<SensorId>& expected_sensor_ids,
    const mapping::proto::TrajectoryBuilderOptions& trajectory_options) {
  proto::AddTrajectoryRequest request;
  request.set_client_id(client_id);
  *request.mutable_trajectory_builder_options() = trajectory_options;
  for (const SensorId& sensor_id : expected_sensor_ids) {
    // Range sensors are not forwarded, but combined into a LocalSlamResult.
    if (sensor_id.type != SensorId::SensorType::RANGE) {
      *request.add_expected_sensor_ids() = cloud::ToProto(sensor_id);
    }
  }
  *request.add_expected_sensor_ids() =
      cloud::ToProto(GetLocalSlamResultSensorId(local_trajectory_id));
  async_grpc::Client<handlers::AddTrajectorySignature> client(client_channel_);
  ::grpc::Status status;
  if (!client.Write(request, &status)) {
    LOG(ERROR) << status.error_message();
    return false;
  }
  LOG(INFO) << "Created trajectory for client_id: " << client_id
            << " local trajectory_id: " << local_trajectory_id
            << " uplink trajectory_id: " << client.response().trajectory_id();
  CHECK_EQ(local_trajectory_id_to_trajectory_info_.count(local_trajectory_id),
           0);
  local_trajectory_id_to_trajectory_info_.emplace(
      std::piecewise_construct, std::forward_as_tuple(local_trajectory_id),
      std::forward_as_tuple(client.response().trajectory_id(),
                            expected_sensor_ids, trajectory_options,
                            client_id));
  return true;
}

void LocalTrajectoryUploader::FinishTrajectory(const std::string& client_id,
                                               int local_trajectory_id) {
  CHECK_EQ(local_trajectory_id_to_trajectory_info_.count(local_trajectory_id),
           1);
  int cloud_trajectory_id =
      local_trajectory_id_to_trajectory_info_.at(local_trajectory_id)
          .uplink_trajectory_id;
  proto::FinishTrajectoryRequest request;
  request.set_client_id(client_id);
  request.set_trajectory_id(cloud_trajectory_id);
  async_grpc::Client<handlers::FinishTrajectorySignature> client(
      client_channel_);
  CHECK(client.Write(request));
}

void LocalTrajectoryUploader::EnqueueSensorData(
    std::unique_ptr<proto::SensorData> sensor_data) {
  send_queue_.Push(std::move(sensor_data));
}

}  // namespace

std::unique_ptr<LocalTrajectoryUploaderInterface> CreateLocalTrajectoryUploader(
    const std::string& uplink_server_address, int batch_size,
    bool enable_ssl_encryption, const std::string& token_file_path) {
  return make_unique<LocalTrajectoryUploader>(uplink_server_address, batch_size,
                                              enable_ssl_encryption,
                                              token_file_path);
}

}  // namespace cloud
}  // namespace cartographer
