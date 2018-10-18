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

#include "absl/container/flat_hash_map.h"
#include "cartographer/mapping/3d/submap_3d.h"
#include "cartographer/mapping/internal/3d/scan_matching/rotational_scan_matcher.h"
#include "cartographer/mapping/probability_values.h"
#include "cartographer/mapping/proto/internal/legacy_serialized_data.pb.h"
#include "cartographer/mapping/proto/internal/legacy_submap.pb.h"
#include "cartographer/mapping/proto/trajectory_builder_options.pb.h"
#include "glog/logging.h"

namespace cartographer {
namespace io {
namespace {

using mapping::proto::SerializedData;
using ProtoMap = absl::flat_hash_map<int, std::vector<SerializedData>>;

bool ReadPoseGraph(cartographer::io::ProtoStreamReaderInterface* const input,
                   ProtoMap* proto_map) {
  auto& pose_graph_vec = (*proto_map)[SerializedData::kPoseGraph];
  pose_graph_vec.emplace_back();
  return input->ReadProto(pose_graph_vec.back().mutable_pose_graph());
}

bool ReadBuilderOptions(
    cartographer::io::ProtoStreamReaderInterface* const input,
    ProtoMap* proto_map) {
  auto& options_vec =
      (*proto_map)[SerializedData::kAllTrajectoryBuilderOptions];
  options_vec.emplace_back();
  return input->ReadProto(
      options_vec.back().mutable_all_trajectory_builder_options());
}

mapping::proto::Submap MigrateLegacySubmap2d(
    const mapping::proto::LegacySubmap& submap_in) {
  mapping::proto::Submap2D submap_2d;

  // Convert probability grid to generalized grid.
  *submap_2d.mutable_grid()->mutable_limits() =
      submap_in.submap_2d().probability_grid().limits();
  *submap_2d.mutable_grid()->mutable_cells() =
      submap_in.submap_2d().probability_grid().cells();
  for (auto& cell : *submap_2d.mutable_grid()->mutable_cells()) {
    cell = -1 * cell;
  }
  // CellBox can't be trivially copied because the protobuf
  // serialization field number doesn't match.
  submap_2d.mutable_grid()->mutable_known_cells_box()->set_max_x(
      submap_in.submap_2d().probability_grid().known_cells_box().max_x());
  submap_2d.mutable_grid()->mutable_known_cells_box()->set_max_y(
      submap_in.submap_2d().probability_grid().known_cells_box().max_y());
  submap_2d.mutable_grid()->mutable_known_cells_box()->set_min_x(
      submap_in.submap_2d().probability_grid().known_cells_box().min_x());
  submap_2d.mutable_grid()->mutable_known_cells_box()->set_min_y(
      submap_in.submap_2d().probability_grid().known_cells_box().min_y());

  // Correspondence costs can be safely set to standard values.
  // Otherwise, this would be done during the next deserialization, but with a
  // warning, which we can avoid by setting it already here.
  submap_2d.mutable_grid()->set_max_correspondence_cost(
      mapping::kMaxCorrespondenceCost);
  submap_2d.mutable_grid()->set_min_correspondence_cost(
      mapping::kMinCorrespondenceCost);
  submap_2d.mutable_grid()->mutable_probability_grid_2d();
  *submap_2d.mutable_local_pose() = submap_in.submap_2d().local_pose();
  submap_2d.set_num_range_data(submap_in.submap_2d().num_range_data());
  submap_2d.set_finished(submap_in.submap_2d().finished());

  mapping::proto::Submap submap_out;
  *submap_out.mutable_submap_2d() = submap_2d;
  *submap_out.mutable_submap_id() = submap_in.submap_id();
  return submap_out;
}

mapping::proto::Submap MigrateLegacySubmap3d(
    const mapping::proto::LegacySubmap& submap_in) {
  mapping::proto::Submap3D submap_3d;
  *submap_3d.mutable_local_pose() = submap_in.submap_3d().local_pose();
  submap_3d.set_num_range_data(submap_in.submap_3d().num_range_data());
  submap_3d.set_finished(submap_in.submap_3d().finished());
  *submap_3d.mutable_high_resolution_hybrid_grid() =
      submap_in.submap_3d().high_resolution_hybrid_grid();
  *submap_3d.mutable_low_resolution_hybrid_grid() =
      submap_in.submap_3d().low_resolution_hybrid_grid();

  mapping::proto::Submap submap_out;
  *submap_out.mutable_submap_3d() = submap_3d;
  *submap_out.mutable_submap_id() = submap_in.submap_id();
  return submap_out;
}

bool DeserializeNext(cartographer::io::ProtoStreamReaderInterface* const input,
                     ProtoMap* proto_map) {
  mapping::proto::LegacySerializedData legacy_data;
  if (!input->ReadProto(&legacy_data)) return false;

  if (legacy_data.has_submap()) {
    LOG_FIRST_N(INFO, 1) << "Migrating submap data.";
    if (legacy_data.submap().has_submap_2d()) {
      CHECK(legacy_data.submap().submap_2d().grid().has_probability_grid_2d() ||
            legacy_data.submap().submap_2d().grid().has_tsdf_2d())
          << "\nThe legacy data contains a 2D submap, but it's not using the "
             "expected grid format. Try to migrate the grid format instead.";
    }
    auto& output_vector = (*proto_map)[SerializedData::kSubmapFieldNumber];
    output_vector.emplace_back();
    *output_vector.back().mutable_submap() = legacy_data.submap();
  }
  if (legacy_data.has_node()) {
    LOG_FIRST_N(INFO, 1) << "Migrating node data.";
    auto& output_vector = (*proto_map)[SerializedData::kNodeFieldNumber];
    output_vector.emplace_back();
    *output_vector.back().mutable_node() = legacy_data.node();
  }
  if (legacy_data.has_trajectory_data()) {
    LOG_FIRST_N(INFO, 1) << "Migrating trajectory data.";
    auto& output_vector =
        (*proto_map)[SerializedData::kTrajectoryDataFieldNumber];
    output_vector.emplace_back();
    *output_vector.back().mutable_trajectory_data() =
        legacy_data.trajectory_data();
  }
  if (legacy_data.has_imu_data()) {
    LOG_FIRST_N(INFO, 1) << "Migrating IMU data.";
    auto& output_vector = (*proto_map)[SerializedData::kImuDataFieldNumber];
    output_vector.emplace_back();
    *output_vector.back().mutable_imu_data() = legacy_data.imu_data();
  }
  if (legacy_data.has_odometry_data()) {
    LOG_FIRST_N(INFO, 1) << "Migrating odometry data.";
    auto& output_vector = (*proto_map)[SerializedData::kOdometryData];
    output_vector.emplace_back();
    *output_vector.back().mutable_odometry_data() = legacy_data.odometry_data();
  }
  if (legacy_data.has_fixed_frame_pose_data()) {
    LOG_FIRST_N(INFO, 1) << "Migrating fixed frame pose data.";
    auto& output_vector =
        (*proto_map)[SerializedData::kFixedFramePoseDataFieldNumber];
    output_vector.emplace_back();
    *output_vector.back().mutable_fixed_frame_pose_data() =
        legacy_data.fixed_frame_pose_data();
  }
  if (legacy_data.has_landmark_data()) {
    LOG_FIRST_N(INFO, 1) << "Migrating landmark data.";
    auto& output_vector =
        (*proto_map)[SerializedData::kLandmarkDataFieldNumber];
    output_vector.emplace_back();
    *output_vector.back().mutable_landmark_data() = legacy_data.landmark_data();
  }
  return true;
}

bool DeserializeNextAndMigrateGridFormat(
    cartographer::io::ProtoStreamReaderInterface* const input,
    ProtoMap* proto_map) {
  mapping::proto::LegacySerializedDataLegacySubmap legacy_data;
  if (!input->ReadProto(&legacy_data)) return false;

  if (legacy_data.has_submap()) {
    LOG_FIRST_N(INFO, 1) << "Migrating submap data.";
    auto& output_vector = (*proto_map)[SerializedData::kSubmapFieldNumber];
    output_vector.emplace_back();
    if (legacy_data.submap().has_submap_2d()) {
      CHECK(legacy_data.submap().submap_2d().has_probability_grid())
          << "\nThe legacy data contains a 2D submap, but it has no legacy "
             "probability grid that can be migrated to the new grid format.";
      *output_vector.back().mutable_submap() =
          MigrateLegacySubmap2d(legacy_data.submap());
    } else {
      *output_vector.back().mutable_submap() =
          MigrateLegacySubmap3d(legacy_data.submap());
    }
  }
  if (legacy_data.has_node()) {
    LOG_FIRST_N(INFO, 1) << "Migrating node data.";
    auto& output_vector = (*proto_map)[SerializedData::kNodeFieldNumber];
    output_vector.emplace_back();
    *output_vector.back().mutable_node() = legacy_data.node();
  }
  if (legacy_data.has_trajectory_data()) {
    LOG_FIRST_N(INFO, 1) << "Migrating trajectory data.";
    auto& output_vector =
        (*proto_map)[SerializedData::kTrajectoryDataFieldNumber];
    output_vector.emplace_back();
    *output_vector.back().mutable_trajectory_data() =
        legacy_data.trajectory_data();
  }
  if (legacy_data.has_imu_data()) {
    LOG_FIRST_N(INFO, 1) << "Migrating IMU data.";
    auto& output_vector = (*proto_map)[SerializedData::kImuDataFieldNumber];
    output_vector.emplace_back();
    *output_vector.back().mutable_imu_data() = legacy_data.imu_data();
  }
  if (legacy_data.has_odometry_data()) {
    LOG_FIRST_N(INFO, 1) << "Migrating odometry data.";
    auto& output_vector = (*proto_map)[SerializedData::kOdometryData];
    output_vector.emplace_back();
    *output_vector.back().mutable_odometry_data() = legacy_data.odometry_data();
  }
  if (legacy_data.has_fixed_frame_pose_data()) {
    LOG_FIRST_N(INFO, 1) << "Migrating fixed frame pose data.";
    auto& output_vector =
        (*proto_map)[SerializedData::kFixedFramePoseDataFieldNumber];
    output_vector.emplace_back();
    *output_vector.back().mutable_fixed_frame_pose_data() =
        legacy_data.fixed_frame_pose_data();
  }
  if (legacy_data.has_landmark_data()) {
    LOG_FIRST_N(INFO, 1) << "Migrating landmark data.";
    auto& output_vector =
        (*proto_map)[SerializedData::kLandmarkDataFieldNumber];
    output_vector.emplace_back();
    *output_vector.back().mutable_landmark_data() = legacy_data.landmark_data();
  }
  return true;
}

ProtoMap ParseLegacyData(
    cartographer::io::ProtoStreamReaderInterface* const input,
    bool migrate_grid_format) {
  ProtoMap proto_map;
  CHECK(ReadPoseGraph(input, &proto_map))
      << "Input stream seems to differ from original stream format. Could "
         "not "
         "read PoseGraph as first message.";
  CHECK(ReadBuilderOptions(input, &proto_map))
      << "Input stream seems to differ from original stream format. Could "
         "not "
         "read AllTrajectoryBuilderOptions as second message.";
  if (migrate_grid_format) {
    do {
    } while (DeserializeNextAndMigrateGridFormat(input, &proto_map));
  } else {
    do {
    } while (DeserializeNext(input, &proto_map));
  }
  return proto_map;
}

mapping::proto::SerializationHeader CreateSerializationHeader() {
  constexpr uint32_t kVersion1 = 1;
  mapping::proto::SerializationHeader header;
  header.set_format_version(kVersion1);
  return header;
}

void SerializeToVersion1Format(
    const ProtoMap& deserialized_data,
    cartographer::io::ProtoStreamWriterInterface* const output) {
  const std::vector<int> kFieldSerializationOrder = {
      SerializedData::kPoseGraphFieldNumber,
      SerializedData::kAllTrajectoryBuilderOptionsFieldNumber,
      SerializedData::kSubmapFieldNumber,
      SerializedData::kNodeFieldNumber,
      SerializedData::kTrajectoryDataFieldNumber,
      SerializedData::kImuDataFieldNumber,
      SerializedData::kOdometryDataFieldNumber,
      SerializedData::kFixedFramePoseDataFieldNumber,
      SerializedData::kLandmarkDataFieldNumber};

  LOG(INFO) << "Writing proto stream.";
  output->WriteProto(CreateSerializationHeader());
  for (auto field_index : kFieldSerializationOrder) {
    const auto proto_vector_it = deserialized_data.find(field_index);
    if (proto_vector_it == deserialized_data.end()) continue;
    for (const auto& proto : proto_vector_it->second) {
      output->WriteProto(proto);
    }
  }
}
}  // namespace

void MigrateStreamFormatToVersion1(
    cartographer::io::ProtoStreamReaderInterface* const input,
    cartographer::io::ProtoStreamWriterInterface* const output,
    bool migrate_grid_format) {
  SerializeToVersion1Format(ParseLegacyData(input, migrate_grid_format),
                            output);
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
