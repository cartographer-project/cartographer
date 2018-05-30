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

#ifndef CARTOGRAPHER_IO_PROTO_STREAM_DESERIALIZER_H_
#define CARTOGRAPHER_IO_PROTO_STREAM_DESERIALIZER_H_

#include "cartographer/io/proto_stream_interface.h"
#include "cartographer/mapping/proto/pose_graph.pb.h"
#include "cartographer/mapping/proto/serialization.pb.h"
#include "cartographer/mapping/proto/trajectory_builder_options.pb.h"

namespace cartographer {
namespace io {

// Helper function for deserializing the PoseGraph from a proto stream file.
mapping::proto::PoseGraph DeserializePoseGraphFromFile(
    const std::string& file_name);

// Helper for deserializing a previously serialized mapping state from a
// proto stream, abstracting away the format parsing logic.
class ProtoStreamDeserializer {
 public:
  explicit ProtoStreamDeserializer(ProtoStreamReaderInterface* const reader);

  ProtoStreamDeserializer(const ProtoStreamDeserializer&) = delete;
  ProtoStreamDeserializer& operator=(const ProtoStreamDeserializer&) = delete;
  ProtoStreamDeserializer(ProtoStreamDeserializer&&) = delete;

  mapping::proto::SerializationHeader& header() { return header_; }

  mapping::proto::PoseGraph& pose_graph() {
    return *pose_graph_.mutable_pose_graph();
  }
  const mapping::proto::PoseGraph& pose_graph() const {
    return pose_graph_.pose_graph();
  }

  const mapping::proto::AllTrajectoryBuilderOptions&
  all_trajectory_builder_options() {
    return all_trajectory_builder_options_.all_trajectory_builder_options();
  }

  // Reads the next `SerializedData` message of the ProtoStream into `data`.
  // Returns `true` if the message was successfully read or `false` in case
  // there are no-more messages or an error occurred.
  bool ReadNextSerializedData(mapping::proto::SerializedData* data);

 private:
  ProtoStreamReaderInterface* reader_;

  mapping::proto::SerializationHeader header_;
  mapping::proto::SerializedData pose_graph_;
  mapping::proto::SerializedData all_trajectory_builder_options_;
};

}  // namespace io
}  // namespace cartographer

#endif  // CARTOGRAPHER_IO_PROTO_STREAM_DESERIALIZER_H_
