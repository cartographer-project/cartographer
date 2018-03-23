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

#include "cartographer/cloud/internal/handlers/add_trajectory_handler.h"

#include "async_grpc/rpc_handler.h"
#include "cartographer/cloud/internal/map_builder_context_interface.h"
#include "cartographer/cloud/internal/sensor/serialization.h"
#include "cartographer/cloud/proto/map_builder_service.pb.h"
#include "cartographer/common/make_unique.h"

namespace cartographer {
namespace cloud {
namespace handlers {

void AddTrajectoryHandler::OnRequest(
    const proto::AddTrajectoryRequest& request) {
  auto local_slam_result_callback =
      GetUnsynchronizedContext<MapBuilderContextInterface>()
          ->GetLocalSlamResultCallbackForSubscriptions();
  std::set<mapping::TrajectoryBuilderInterface::SensorId> expected_sensor_ids;
  for (const auto& sensor_id : request.expected_sensor_ids()) {
    expected_sensor_ids.insert(FromProto(sensor_id));
  }
  const int trajectory_id =
      GetContext<MapBuilderContextInterface>()
          ->map_builder()
          .AddTrajectoryBuilder(expected_sensor_ids,
                                request.trajectory_builder_options(),
                                local_slam_result_callback);
  if (GetUnsynchronizedContext<MapBuilderContextInterface>()
          ->local_trajectory_uploader()) {
    auto trajectory_builder_options = request.trajectory_builder_options();

    // Clear the trajectory builder options to convey to the cloud
    // Cartographer instance that does not need to instantiate a
    // 'LocalTrajectoryBuilder'.
    trajectory_builder_options.clear_trajectory_builder_2d_options();
    trajectory_builder_options.clear_trajectory_builder_3d_options();

    // Don't instantiate the 'PureLocalizationTrimmer' on the server and don't
    // freeze the trajectory on the server.
    trajectory_builder_options.set_pure_localization(false);

    GetContext<MapBuilderContextInterface>()
        ->local_trajectory_uploader()
        ->AddTrajectory(trajectory_id, expected_sensor_ids,
                        trajectory_builder_options);
  }

  auto response = common::make_unique<proto::AddTrajectoryResponse>();
  response->set_trajectory_id(trajectory_id);
  Send(std::move(response));
}

}  // namespace handlers
}  // namespace cloud
}  // namespace cartographer
