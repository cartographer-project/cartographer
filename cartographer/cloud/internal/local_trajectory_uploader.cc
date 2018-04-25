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

#include "async_grpc/client.h"
#include "cartographer/cloud/internal/handlers/add_sensor_data_batch_handler.h"
#include "cartographer/cloud/internal/handlers/add_trajectory_handler.h"
#include "cartographer/cloud/internal/handlers/finish_trajectory_handler.h"
#include "cartographer/cloud/internal/sensor/serialization.h"
#include "cartographer/common/blocking_queue.h"
#include "cartographer/common/make_unique.h"
#include "glog/logging.h"
#include "grpc++/grpc++.h"

namespace cartographer {
namespace cloud {
namespace {

using common::make_unique;

constexpr int kConnectionTimeoutInSecond = 10;
const common::Duration kPopTimeout = common::FromMilliseconds(100);

class LocalTrajectoryUploader : public LocalTrajectoryUploaderInterface {
 public:
  LocalTrajectoryUploader(const std::string &uplink_server_address,
                          int batch_size, bool enable_ssl_encryption);
  ~LocalTrajectoryUploader();

  // Starts the upload thread.
  void Start() final;

  // Shuts down the upload thread. This method blocks until the shutdown is
  // complete.
  void Shutdown() final;

  void AddTrajectory(
      int local_trajectory_id, const std::set<SensorId> &expected_sensor_ids,
      const mapping::proto::TrajectoryBuilderOptions &trajectory_options) final;
  void FinishTrajectory(int local_trajectory_id) final;
  void EnqueueSensorData(std::unique_ptr<proto::SensorData> sensor_data) final;

  SensorId GetLocalSlamResultSensorId(int local_trajectory_id) const final {
    return SensorId{SensorId::SensorType::LOCAL_SLAM_RESULT,
                    "local_slam_result_" + std::to_string(local_trajectory_id)};
  }

 private:
  void ProcessSendQueue();
  void TranslateTrajectoryId(proto::SensorMetadata *sensor_metadata);

  std::shared_ptr<::grpc::Channel> client_channel_;
  int batch_size_;
  std::map<int, int> local_to_cloud_trajectory_id_map_;
  common::BlockingQueue<std::unique_ptr<proto::SensorData>> send_queue_;
  bool shutting_down_ = false;
  std::unique_ptr<std::thread> upload_thread_;
};

LocalTrajectoryUploader::LocalTrajectoryUploader(
    const std::string &uplink_server_address, int batch_size,
    bool enable_ssl_encryption)
    : client_channel_(::grpc::CreateChannel(
          uplink_server_address,
          enable_ssl_encryption
              ? ::grpc::SslCredentials(::grpc::SslCredentialsOptions())
              : ::grpc::InsecureChannelCredentials())),
      batch_size_(batch_size) {
  std::chrono::system_clock::time_point deadline(
      std::chrono::system_clock::now() +
      std::chrono::seconds(kConnectionTimeoutInSecond));
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

void LocalTrajectoryUploader::ProcessSendQueue() {
  LOG(INFO) << "Starting uploader thread.";
  proto::AddSensorDataBatchRequest batch_request;
  while (!shutting_down_) {
    auto sensor_data = send_queue_.PopWithTimeout(kPopTimeout);
    if (sensor_data) {
      proto::SensorData *added_sensor_data = batch_request.add_sensor_data();
      *added_sensor_data = *sensor_data;
      TranslateTrajectoryId(added_sensor_data->mutable_sensor_metadata());

      // A submap also holds a trajectory id that must be translated to uplink's
      // trajectory id.
      if (added_sensor_data->has_local_slam_result_data()) {
        for (mapping::proto::Submap &mutable_submap :
             *added_sensor_data->mutable_local_slam_result_data()
                  ->mutable_submaps()) {
          mutable_submap.mutable_submap_id()->set_trajectory_id(
              added_sensor_data->sensor_metadata().trajectory_id());
        }
      }

      if (batch_request.sensor_data_size() == batch_size_) {
        async_grpc::Client<handlers::AddSensorDataBatchSignature> client(
            client_channel_, async_grpc::CreateUnlimitedConstantDelayStrategy(
                                 common::FromSeconds(1)));
        CHECK(client.Write(batch_request));
        batch_request.clear_sensor_data();
      }
    }
  }
}

void LocalTrajectoryUploader::TranslateTrajectoryId(
    proto::SensorMetadata *sensor_metadata) {
  int cloud_trajectory_id =
      local_to_cloud_trajectory_id_map_.at(sensor_metadata->trajectory_id());
  sensor_metadata->set_trajectory_id(cloud_trajectory_id);
}

void LocalTrajectoryUploader::AddTrajectory(
    int local_trajectory_id, const std::set<SensorId> &expected_sensor_ids,
    const mapping::proto::TrajectoryBuilderOptions &trajectory_options) {
  proto::AddTrajectoryRequest request;
  *request.mutable_trajectory_builder_options() = trajectory_options;
  for (const SensorId &sensor_id : expected_sensor_ids) {
    // Range sensors are not forwarded, but combined into a LocalSlamResult.
    if (sensor_id.type != SensorId::SensorType::RANGE) {
      *request.add_expected_sensor_ids() = cloud::ToProto(sensor_id);
    }
  }
  *request.add_expected_sensor_ids() =
      cloud::ToProto(GetLocalSlamResultSensorId(local_trajectory_id));
  async_grpc::Client<handlers::AddTrajectorySignature> client(client_channel_);
  CHECK(client.Write(request));
  CHECK_EQ(local_to_cloud_trajectory_id_map_.count(local_trajectory_id), 0);
  local_to_cloud_trajectory_id_map_[local_trajectory_id] =
      client.response().trajectory_id();
}

void LocalTrajectoryUploader::FinishTrajectory(int local_trajectory_id) {
  CHECK_EQ(local_to_cloud_trajectory_id_map_.count(local_trajectory_id), 1);
  int cloud_trajectory_id =
      local_to_cloud_trajectory_id_map_[local_trajectory_id];
  proto::FinishTrajectoryRequest request;
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
    const std::string &uplink_server_address, int batch_size,
    bool enable_ssl_encryption) {
  return make_unique<LocalTrajectoryUploader>(uplink_server_address, batch_size,
                                              enable_ssl_encryption);
}

}  // namespace cloud
}  // namespace cartographer
