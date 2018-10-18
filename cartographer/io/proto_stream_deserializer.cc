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

#include "cartographer/io/proto_stream_deserializer.h"

#include "cartographer/io/internal/mapping_state_serialization.h"
#include "cartographer/io/proto_stream.h"
#include "glog/logging.h"

namespace cartographer {
namespace io {
namespace {
mapping::proto::SerializationHeader ReadHeaderOrDie(
    ProtoStreamReaderInterface* const reader) {
  mapping::proto::SerializationHeader header;
  CHECK(reader->ReadProto(&header)) << "Failed to read SerializationHeader.";
  return header;
}

bool IsVersionSupported(const mapping::proto::SerializationHeader& header) {
  return header.format_version() == kMappingStateSerializationFormatVersion ||
         header.format_version() == kFormatVersionWithoutSubmapHistograms;
}

}  // namespace

mapping::proto::PoseGraph DeserializePoseGraphFromFile(
    const std::string& file_name) {
  ProtoStreamReader reader(file_name);
  ProtoStreamDeserializer deserializer(&reader);
  return deserializer.pose_graph();
}

ProtoStreamDeserializer::ProtoStreamDeserializer(
    ProtoStreamReaderInterface* const reader)
    : reader_(reader), header_(ReadHeaderOrDie(reader)) {
  CHECK(IsVersionSupported(header_)) << "Unsupported serialization format \""
                                     << header_.format_version() << "\"";

  CHECK(ReadNextSerializedData(&pose_graph_))
      << "Serialized stream misses PoseGraph.";
  CHECK(pose_graph_.has_pose_graph())
      << "Serialized stream order corrupt. Expecting `PoseGraph` after "
         "`SerializationHeader`, but got field tag "
      << pose_graph_.data_case();

  CHECK(ReadNextSerializedData(&all_trajectory_builder_options_))
      << "Serialized stream misses `AllTrajectoryBuilderOptions`.";
  CHECK(all_trajectory_builder_options_.has_all_trajectory_builder_options())
      << "Serialized stream order corrupt. Expecting "
         "`AllTrajectoryBuilderOptions` after "
         "PoseGraph, got field tag "
      << all_trajectory_builder_options_.data_case();

  CHECK_EQ(pose_graph_.pose_graph().trajectory_size(),
           all_trajectory_builder_options_.all_trajectory_builder_options()
               .options_with_sensor_ids_size());
}

bool ProtoStreamDeserializer::ReadNextSerializedData(
    mapping::proto::SerializedData* data) {
  return reader_->ReadProto(data);
}

}  // namespace io
}  // namespace cartographer
