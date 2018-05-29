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
#ifndef CARTOGRAPHER_IO_INTERNAL_MAPPING_STATE_SERIALIZATION_H_
#define CARTOGRAPHER_IO_INTERNAL_MAPPING_STATE_SERIALIZATION_H_

#include "cartographer/io/proto_stream_interface.h"
#include "cartographer/mapping/pose_graph.h"
#include "cartographer/mapping/proto/trajectory_builder_options.pb.h"

namespace cartographer {
namespace io {

// The current serialization format version.
static constexpr int kMappingStateSerializationFormatVersion = 1;

// Serialize mapping state to a pbstream.
void WritePbStream(
    const mapping::PoseGraph& pose_graph,
    const std::vector<mapping::proto::TrajectoryBuilderOptionsWithSensorIds>&
        builder_options,
    ProtoStreamWriterInterface* const writer);

}  // namespace io
}  // namespace cartographer

#endif  // CARTOGRAPHER_IO_INTERNAL_MAPPING_STATE_SERIALIZATION_H_
