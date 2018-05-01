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

#ifndef CARTOGRAPHER_CLOUD_MAP_BUILDER_SERVER_INTERFACE_H
#define CARTOGRAPHER_CLOUD_MAP_BUILDER_SERVER_INTERFACE_H

#include <memory>

#include "cartographer/cloud/proto/map_builder_server_options.pb.h"
#include "cartographer/mapping/map_builder_interface.h"
#include "cartographer/metrics/family_factory.h"

namespace cartographer {
namespace cloud {

class MapBuilderServerInterface {
 public:
  virtual ~MapBuilderServerInterface() {}

  // Starts the gRPC server, the 'LocalTrajectoryUploader' and the SLAM thread.
  virtual void Start() = 0;

  // Waits for the 'MapBuilderServer' to shut down. Note: The server must be
  // either shutting down or some other thread must call 'Shutdown()' for
  // this function to ever return.
  virtual void WaitForShutdown() = 0;

  // Waits until all computation is finished (for testing).
  virtual void WaitUntilIdle() = 0;

  // Shuts down the gRPC server, the 'LocalTrajectoryUploader' and the SLAM
  // thread.
  virtual void Shutdown() = 0;
};

// Registers all metrics for the MapBuilderServer.
void RegisterMapBuilderServerMetrics(metrics::FamilyFactory* factory);

// Returns MapBuilderServer with the actual implementation.
std::unique_ptr<MapBuilderServerInterface> CreateMapBuilderServer(
    const proto::MapBuilderServerOptions& map_builder_server_options,
    std::unique_ptr<mapping::MapBuilderInterface> map_builder);

}  // namespace cloud
}  // namespace cartographer

#endif  // CARTOGRAPHER_CLOUD_MAP_BUILDER_SERVER_INTERFACE_H
