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

#include <fstream>

#include "cartographer/io/proto_stream.h"
#include "cartographer/io/proto_stream_deserializer.h"
#include "gflags/gflags.h"
#include "glog/logging.h"

DEFINE_string(pbstream_filename, "", "Proto stream file.");
DEFINE_bool(all_debug_strings, false,
            "Print debug strings of all serialized data.");

namespace cartographer {
namespace mapping {
namespace {

void Run(const std::string& pbstream_filename, bool all_debug_strings) {
  LOG(INFO) << "Reading pbstream file from '" << pbstream_filename << "'...";
  io::ProtoStreamReader reader(pbstream_filename);
  io::ProtoStreamDeserializer deserializer(&reader);
  const auto header = deserializer.header();
  LOG(INFO) << "Header: " << header.DebugString();
  for (const proto::TrajectoryBuilderOptionsWithSensorIds& trajectory_options :
       deserializer.all_trajectory_builder_options()
           .options_with_sensor_ids()) {
    LOG(INFO) << "Trajectory options: " << trajectory_options.DebugString();
  }
  const proto::PoseGraph pose_graph = deserializer.pose_graph();
  for (const proto::Trajectory& trajectory : pose_graph.trajectory()) {
    LOG(INFO) << "Trajectory id: " << trajectory.trajectory_id()
              << " has #nodes " << trajectory.node_size() << " has #submaps "
              << trajectory.submap_size();
  }
  if (all_debug_strings) {
    LOG(INFO) << "Pose graph: " << pose_graph.DebugString();
  }

  std::map<std::string, int> data_counts;
  data_counts["submap_2d"];
  data_counts["submap_2d_grid"];
  data_counts["submap_3d"];
  data_counts["submap_3d_high_resolution_hybrid_grid"];
  data_counts["node"];
  data_counts["trajectory_data"];
  data_counts["imu_data"];
  data_counts["odometry_data"];
  data_counts["fixed_frame_pose_data"];
  data_counts["landmark_data"];
  proto::SerializedData proto;
  while (deserializer.ReadNextSerializedData(&proto)) {
    if (all_debug_strings) {
      LOG(INFO) << "Serialized data: " << proto.DebugString();
    }
    switch (proto.data_case()) {
      case proto::SerializedData::kSubmap: {
        if (proto.mutable_submap()->has_submap_2d()) {
          data_counts["submap_2d"]++;
          if (proto.mutable_submap()->mutable_submap_2d()->has_grid()) {
            data_counts["submap_2d_grid"]++;
          }
        }
        if (proto.mutable_submap()->has_submap_3d()) {
          data_counts["submap_3d"]++;
          if (proto.mutable_submap()
                  ->mutable_submap_3d()
                  ->has_high_resolution_hybrid_grid()) {
            data_counts["submap_3d_high_resolution_hybrid_grid"]++;
          }
        }
        break;
      }
      case proto::SerializedData::kNode: {
        data_counts["node"]++;
        break;
      }
      case proto::SerializedData::kTrajectoryData: {
        data_counts["trajectory_data"]++;
        break;
      }
      case proto::SerializedData::kImuData: {
        data_counts["imu_data"]++;
        break;
      }
      case proto::SerializedData::kOdometryData: {
        data_counts["odometry_data"]++;
        break;
      }
      case proto::SerializedData::kFixedFramePoseData: {
        data_counts["fixed_frame_pose_data"]++;
        break;
      }
      case proto::SerializedData::kLandmarkData: {
        data_counts["landmark_data"]++;
        break;
      }
      default:
        LOG(WARNING) << "Skipping unknown message type in stream: "
                     << proto.GetTypeName();
    }
  }

  for (const auto& entry : data_counts) {
    LOG(INFO) << "SerializedData package contains #" << entry.first << ": "
              << entry.second;
  }
}

}  // namespace
}  // namespace mapping
}  // namespace cartographer

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  FLAGS_logtostderr = true;
  google::SetUsageMessage(
      "\n\n"
      "Reads a pbstream file and summarizes its contents.\n");
  google::ParseCommandLineFlags(&argc, &argv, true);

  if (FLAGS_pbstream_filename.empty()) {
    google::ShowUsageWithFlagsRestrict(argv[0], "pbstream_info");
    return EXIT_FAILURE;
  }
  ::cartographer::mapping::Run(FLAGS_pbstream_filename,
                               FLAGS_all_debug_strings);
}
