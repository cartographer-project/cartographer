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

#ifndef CARTOGRAPHER_IO_SERIALIZATION_FORMAT_MIGRATION_H_
#define CARTOGRAPHER_IO_SERIALIZATION_FORMAT_MIGRATION_H_

#include "cartographer/io/proto_stream_interface.h"

namespace cartographer {
namespace io {

// This helper function, migrates the input stream, which is supposed to match
// to the "old" stream format order (PoseGraph, AllTrajectoryBuilderOptions,
// SerializedData*) to the version 1 stream format (SerializationHeader,
// SerializedData*).
void MigrateStreamFormatToVersion1(
    cartographer::io::ProtoStreamReaderInterface* const input,
    cartographer::io::ProtoStreamWriterInterface* const output);

}  // namespace io
}  // namespace cartographer

#endif  // CARTOGRAPHER_IO_SERIALIZATION_FORMAT_MIGRATION_H_
