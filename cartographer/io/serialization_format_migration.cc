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

#include "cartographer/io/serialization_format_migration.h"

#include <vector>

#include "cartographer/common/config.h"
#include "cartographer/common/configuration_file_resolver.h"
#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/common/thread_pool.h"
#include "cartographer/io/internal/mapping_state_serialization.h"
#include "cartographer/io/proto_stream_deserializer.h"
#include "cartographer/mapping/3d/submap_3d.h"
#include "cartographer/mapping/id.h"
#include "cartographer/mapping/internal/3d/pose_graph_3d.h"
#include "cartographer/mapping/internal/3d/scan_matching/rotational_scan_matcher.h"
#include "cartographer/mapping/map_builder.h"
#include "cartographer/mapping/map_builder_interface.h"
#include "cartographer/mapping/pose_graph.h"
#include "cartographer/mapping/probability_values.h"
#include "cartographer/mapping/proto/map_builder_options.pb.h"
#include "cartographer/mapping/proto/trajectory_builder_options.pb.h"
#include "glog/logging.h"

namespace cartographer {
namespace io {

using mapping::proto::SerializedData;

void MigrateStreamVersion1ToVersion2(
    cartographer::io::ProtoStreamReaderInterface* const input,
    cartographer::io::ProtoStreamWriterInterface* const output,
    bool include_unfinished_submaps) {
  auto file_resolver = ::absl::make_unique<common::ConfigurationFileResolver>(
      std::vector<std::string>{std::string(common::kSourceDirectory) +
                               "/configuration_files"});
  const std::string kCode = R"text(
      include "map_builder.lua"
      MAP_BUILDER.use_trajectory_builder_3d = true
      return MAP_BUILDER)text";
  cartographer::common::LuaParameterDictionary lua_parameter_dictionary(
      kCode, std::move(file_resolver));
  const auto options =
      mapping::CreateMapBuilderOptions(&lua_parameter_dictionary);

  common::ThreadPool thread_pool(1);
  CHECK(!options.use_trajectory_builder_2d());
  // We always use 3D here. 2D submaps do not have histograms.
  mapping::PoseGraph3D pose_graph(
      options.pose_graph_options(),
      absl::make_unique<mapping::optimization::OptimizationProblem3D>(
          options.pose_graph_options().optimization_problem_options()),
      &thread_pool);

  ProtoStreamDeserializer deserializer(input);

  // Create a copy of the pose_graph_proto, such that we can re-write the
  // trajectory ids.
  mapping::proto::PoseGraph pose_graph_proto = deserializer.pose_graph();
  const auto& all_builder_options_proto =
      deserializer.all_trajectory_builder_options();

  std::vector<mapping::proto::TrajectoryBuilderOptionsWithSensorIds>
      trajectory_builder_options;
  for (int i = 0; i < pose_graph_proto.trajectory_size(); ++i) {
    auto& trajectory_proto = *pose_graph_proto.mutable_trajectory(i);
    const auto& options_with_sensor_ids_proto =
        all_builder_options_proto.options_with_sensor_ids(i);
    trajectory_builder_options.push_back(options_with_sensor_ids_proto);
    CHECK_EQ(trajectory_proto.trajectory_id(), i);
  }

  // Apply the calculated remapping to constraints in the pose graph proto.
  for (auto& constraint_proto : *pose_graph_proto.mutable_constraint()) {
    constraint_proto.mutable_submap_id()->set_trajectory_id(
        constraint_proto.submap_id().trajectory_id());
    constraint_proto.mutable_node_id()->set_trajectory_id(
        constraint_proto.node_id().trajectory_id());
  }

  mapping::MapById<mapping::SubmapId, transform::Rigid3d> submap_poses;
  for (const mapping::proto::Trajectory& trajectory_proto :
       pose_graph_proto.trajectory()) {
    for (const mapping::proto::Trajectory::Submap& submap_proto :
         trajectory_proto.submap()) {
      submap_poses.Insert(mapping::SubmapId{trajectory_proto.trajectory_id(),
                                            submap_proto.submap_index()},
                          transform::ToRigid3(submap_proto.pose()));
    }
  }

  mapping::MapById<mapping::NodeId, transform::Rigid3d> node_poses;
  for (const mapping::proto::Trajectory& trajectory_proto :
       pose_graph_proto.trajectory()) {
    for (const mapping::proto::Trajectory::Node& node_proto :
         trajectory_proto.node()) {
      node_poses.Insert(mapping::NodeId{trajectory_proto.trajectory_id(),
                                        node_proto.node_index()},
                        transform::ToRigid3(node_proto.pose()));
    }
  }

  // Set global poses of landmarks.
  for (const auto& landmark : pose_graph_proto.landmark_poses()) {
    pose_graph.SetLandmarkPose(landmark.landmark_id(),
                               transform::ToRigid3(landmark.global_pose()),
                               true);
  }

  mapping::MapById<mapping::SubmapId, mapping::proto::Submap>
      submap_id_to_submap;
  mapping::MapById<mapping::NodeId, mapping::proto::Node> node_id_to_node;
  SerializedData proto;
  while (deserializer.ReadNextSerializedData(&proto)) {
    switch (proto.data_case()) {
      case SerializedData::kPoseGraph:
        LOG(FATAL) << "Found multiple serialized `PoseGraph`. Serialized "
                      "stream likely corrupt!.";
      case SerializedData::kAllTrajectoryBuilderOptions:
        LOG(FATAL) << "Found multiple serialized "
                      "`AllTrajectoryBuilderOptions`. Serialized stream likely "
                      "corrupt!.";
      case SerializedData::kSubmap: {
        CHECK(proto.submap().has_submap_3d())
            << "Converting to the new submap format only makes sense for 3D.";
        proto.mutable_submap()->mutable_submap_id()->set_trajectory_id(
            proto.submap().submap_id().trajectory_id());
        submap_id_to_submap.Insert(
            mapping::SubmapId{proto.submap().submap_id().trajectory_id(),
                              proto.submap().submap_id().submap_index()},
            proto.submap());
        break;
      }
      case SerializedData::kNode: {
        proto.mutable_node()->mutable_node_id()->set_trajectory_id(
            proto.node().node_id().trajectory_id());
        const mapping::NodeId node_id(proto.node().node_id().trajectory_id(),
                                      proto.node().node_id().node_index());
        const transform::Rigid3d& node_pose = node_poses.at(node_id);
        pose_graph.AddNodeFromProto(node_pose, proto.node());
        node_id_to_node.Insert(node_id, proto.node());
        break;
      }
      case SerializedData::kTrajectoryData: {
        proto.mutable_trajectory_data()->set_trajectory_id(
            proto.trajectory_data().trajectory_id());
        pose_graph.SetTrajectoryDataFromProto(proto.trajectory_data());
        break;
      }
      case SerializedData::kImuData: {
        pose_graph.AddImuData(proto.imu_data().trajectory_id(),
                              sensor::FromProto(proto.imu_data().imu_data()));
        break;
      }
      case SerializedData::kOdometryData: {
        pose_graph.AddOdometryData(
            proto.odometry_data().trajectory_id(),
            sensor::FromProto(proto.odometry_data().odometry_data()));
        break;
      }
      case SerializedData::kFixedFramePoseData: {
        pose_graph.AddFixedFramePoseData(
            proto.fixed_frame_pose_data().trajectory_id(),
            sensor::FromProto(
                proto.fixed_frame_pose_data().fixed_frame_pose_data()));
        break;
      }
      case SerializedData::kLandmarkData: {
        pose_graph.AddLandmarkData(
            proto.landmark_data().trajectory_id(),
            sensor::FromProto(proto.landmark_data().landmark_data()));
        break;
      }
      default:
        LOG(WARNING) << "Skipping unknown message type in stream: "
                     << proto.GetTypeName();
    }
  }

  // TODO(schwoere): Remove backwards compatibility once the pbstream format
  // version 2 is established.
  if (deserializer.header().format_version() ==
      kFormatVersionWithoutSubmapHistograms) {
    submap_id_to_submap = MigrateSubmapFormatVersion1ToVersion2(
        submap_id_to_submap, node_id_to_node, pose_graph_proto);
  }

  for (const auto& submap_id_submap : submap_id_to_submap) {
    pose_graph.AddSubmapFromProto(submap_poses.at(submap_id_submap.id),
                                  submap_id_submap.data);
  }

  pose_graph.AddSerializedConstraints(
      mapping::FromProto(pose_graph_proto.constraint()));
  CHECK(input->eof());

  WritePbStream(pose_graph, trajectory_builder_options, output,
                include_unfinished_submaps);
}

mapping::MapById<mapping::SubmapId, mapping::proto::Submap>
MigrateSubmapFormatVersion1ToVersion2(
    const mapping::MapById<mapping::SubmapId, mapping::proto::Submap>&
        submap_id_to_submap,
    mapping::MapById<mapping::NodeId, mapping::proto::Node>& node_id_to_node,
    const mapping::proto::PoseGraph& pose_graph_proto) {
  using namespace mapping;
  if (submap_id_to_submap.empty() ||
      submap_id_to_submap.begin()->data.has_submap_2d()) {
    return submap_id_to_submap;
  }

  MapById<SubmapId, proto::Submap> migrated_submaps = submap_id_to_submap;
  for (const proto::PoseGraph::Constraint& constraint_proto :
       pose_graph_proto.constraint()) {
    if (constraint_proto.tag() == proto::PoseGraph::Constraint::INTRA_SUBMAP) {
      NodeId node_id{constraint_proto.node_id().trajectory_id(),
                     constraint_proto.node_id().node_index()};
      CHECK(node_id_to_node.Contains(node_id));
      const TrajectoryNode::Data node_data =
          FromProto(node_id_to_node.at(node_id).node_data());
      const Eigen::VectorXf& rotational_scan_matcher_histogram_in_gravity =
          node_data.rotational_scan_matcher_histogram;

      SubmapId submap_id{constraint_proto.submap_id().trajectory_id(),
                         constraint_proto.submap_id().submap_index()};
      CHECK(migrated_submaps.Contains(submap_id));
      proto::Submap& migrated_submap_proto = migrated_submaps.at(submap_id);
      CHECK(migrated_submap_proto.has_submap_3d());

      proto::Submap3D* submap_3d_proto =
          migrated_submap_proto.mutable_submap_3d();
      const double submap_yaw_from_gravity =
          transform::GetYaw(transform::ToRigid3(submap_3d_proto->local_pose())
                                .inverse()
                                .rotation() *
                            node_data.local_pose.rotation() *
                            node_data.gravity_alignment.inverse());
      const Eigen::VectorXf rotational_scan_matcher_histogram_in_submap =
          scan_matching::RotationalScanMatcher::RotateHistogram(
              rotational_scan_matcher_histogram_in_gravity,
              submap_yaw_from_gravity);

      if (submap_3d_proto->rotational_scan_matcher_histogram_size() == 0) {
        for (Eigen::VectorXf::Index i = 0;
             i != rotational_scan_matcher_histogram_in_submap.size(); ++i) {
          submap_3d_proto->add_rotational_scan_matcher_histogram(
              rotational_scan_matcher_histogram_in_submap(i));
        }
      } else {
        auto submap_histogram =
            submap_3d_proto->mutable_rotational_scan_matcher_histogram();
        for (Eigen::VectorXf::Index i = 0;
             i != rotational_scan_matcher_histogram_in_submap.size(); ++i) {
          *submap_histogram->Mutable(i) +=
              rotational_scan_matcher_histogram_in_submap(i);
        }
      }
    }
  }
  return migrated_submaps;
}

}  // namespace io
}  // namespace cartographer
