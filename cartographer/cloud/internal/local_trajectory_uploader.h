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

#ifndef CARTOGRAPHER_CLOUD_INTERNAL_LOCAL_TRAJECTORY_UPLOADER_H
#define CARTOGRAPHER_CLOUD_INTERNAL_LOCAL_TRAJECTORY_UPLOADER_H

#include <memory>
#include <set>
#include <string>

#include "cartographer/cloud/proto/map_builder_service.pb.h"
#include "cartographer/mapping/proto/trajectory_builder_options.pb.h"
#include "cartographer/mapping/trajectory_builder_interface.h"

namespace cartographer {
namespace cloud {

class LocalTrajectoryUploaderInterface {
 public:
  using SensorId = mapping::TrajectoryBuilderInterface::SensorId;

  virtual ~LocalTrajectoryUploaderInterface() = default;

  // Starts the upload thread.
  virtual void Start() = 0;

  // Shuts down the upload thread. This method blocks until the shutdown is
  // complete.
  virtual void Shutdown() = 0;

  // Enqueue an Add*DataRequest message to be uploaded.
  virtual void EnqueueSensorData(
      std::unique_ptr<proto::SensorData> sensor_data) = 0;
  virtual void AddTrajectory(
      int local_trajectory_id, const std::set<SensorId>& expected_sensor_ids,
      const mapping::proto::TrajectoryBuilderOptions& trajectory_options) = 0;
  virtual void FinishTrajectory(int local_trajectory_id) = 0;

  virtual SensorId GetLocalSlamResultSensorId(
      int local_trajectory_id) const = 0;
};

// Returns LocalTrajectoryUploader with the actual implementation.
std::unique_ptr<LocalTrajectoryUploaderInterface> CreateLocalTrajectoryUploader(
    const std::string& uplink_server_address, int batch_size,
    bool enable_ssl_encryption);

}  // namespace cloud
}  // namespace cartographer

#endif  // CARTOGRAPHER_CLOUD_INTERNAL_LOCAL_TRAJECTORY_UPLOADER_H
