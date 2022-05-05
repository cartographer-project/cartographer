/*
 * Copyright 2017 The Cartographer Authors
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

#include "cartographer/mapping/map_builder.h"
#include "cartographer/common/config.h"
#include "cartographer/io/proto_stream.h"
#include "cartographer/mapping/2d/grid_2d.h"
#include "cartographer/mapping/internal/testing/test_helpers.h"
#include "cartographer/mapping/internal/local_slam_result_data.h"
#include "cartographer/mapping/internal/testing/mock_map_builder.h"
#include "cartographer/mapping/internal/testing/mock_pose_graph.h"
#include "cartographer/mapping/internal/testing/mock_trajectory_builder.h"
#include "cartographer/mapping/internal/testing/test_helpers.h"
#include "cartographer/mapping/trajectory_builder_interface.h"
#include "gmock/gmock.h"
#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/common/configuration_file_resolver.h"
#include "cartographer/mapping/map_builder_interface.h"
#include "gtest/gtest.h"
#include "glog/logging.h"

#include "viam/src/io/read_PCD_file.h"
#include "viam/src/mapping/map_builder.h"

namespace viam {
namespace mapping {

using SensorId = cartographer::mapping::TrajectoryBuilderInterface::SensorId;

const SensorId kRangeSensorId{SensorId::SensorType::RANGE, "range"};
const SensorId kIMUSensorId{SensorId::SensorType::IMU, "imu"};
double kDuration = 4.;         // Seconds.
constexpr double kTimeStep = 0.1;        // Seconds.
constexpr double kTravelDistance = 1.2;  // Meters.


void MapBuilder::SetUp(std::string configuration_directory, std::string configuration_basename) {

  auto file_resolver =
      absl::make_unique<cartographer::common::ConfigurationFileResolver>(
          std::vector<std::string>{configuration_directory});
  const std::string lua_code =
      file_resolver->GetFileContentOrDie(configuration_basename);

  auto options = cartographer::common::LuaParameterDictionary::NonReferenceCounted(lua_code, std::move(file_resolver));

  auto map_builder_parameters = options->GetDictionary("map_builder");
  auto trajectory_builder_parameters = options->GetDictionary("trajectory_builder");

  map_builder_options_ = cartographer::mapping::CreateMapBuilderOptions(map_builder_parameters.get());
  trajectory_builder_options_ = cartographer::mapping::CreateTrajectoryBuilderOptions(trajectory_builder_parameters.get());

  return;
  }

void MapBuilder::BuildMapBuilder() {
    map_builder_ = cartographer::mapping::CreateMapBuilder(map_builder_options_);
  }

cartographer::mapping::MapBuilderInterface::LocalSlamResultCallback MapBuilder::GetLocalSlamResultCallback() {
    return [=](const int trajectory_id, const ::cartographer::common::Time time,
               const ::cartographer::transform::Rigid3d local_pose,
               ::cartographer::sensor::RangeData range_data_in_local,
               const std::unique_ptr<
                   const cartographer::mapping::TrajectoryBuilderInterface::
                       InsertionResult>) {
      local_slam_result_poses_.push_back(local_pose);
    };
  }

cartographer::sensor::TimedPointCloudData MapBuilder::GetDataFromFile(std::string data_directory, std::string initial_filename, int i) {
    viam::io::ReadFile read_file;
    std::vector<std::string> files;
    cartographer::sensor::TimedPointCloudData point_cloud;

    files = read_file.listFilesInDirectory(data_directory);

    if ( files.size() == 0 ) {
      LOG(INFO) << "No files found in data directory\n"; 
      return point_cloud;
    }

    point_cloud = read_file.timedPointCloudDataFromPCDBuilder(files[i], initial_filename);

    LOG(INFO) << "----------PCD-------";
    LOG(INFO) << "Time: " << point_cloud.time;  
    LOG(INFO) << "Range (size): " << point_cloud.ranges.size();
    LOG(INFO) << "Range start (time): " << point_cloud.ranges[0].time;
    LOG(INFO) << "Range end (time): " << (point_cloud.ranges.back()).time;
    LOG(INFO) << "-----------------\n";

    return point_cloud;
  }

}  // namespace mapping
}  // namespace viam
