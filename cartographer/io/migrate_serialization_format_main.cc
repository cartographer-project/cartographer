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

#include "cartographer/io/proto_stream.h"
#include "cartographer/io/serialization_format_migration.h"
#include "gflags/gflags.h"
#include "glog/logging.h"

DEFINE_string(
    original_pbstream_file, "",
    "Path to the pbstream file that will be migrated to the new version.");
DEFINE_string(output_pbstream_file, "",
              "Output filename for the migrated pbstream.");

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  FLAGS_logtostderr = true;
  google::SetUsageMessage(
      "\n\n"
      "Tool for migrating files that use the serialization output of "
      "Cartographer 0.3, to the new serialization format, which includes a "
      "header (Version 1).");
  google::ParseCommandLineFlags(&argc, &argv, true);

  if (FLAGS_original_pbstream_file.empty() ||
      FLAGS_output_pbstream_file.empty()) {
    google::ShowUsageWithFlagsRestrict(argv[0], "migrate_serialization_format");
    return EXIT_FAILURE;
  }
  cartographer::io::ProtoStreamReader input(FLAGS_original_pbstream_file);
  cartographer::io::ProtoStreamWriter output(FLAGS_output_pbstream_file);
  cartographer::io::MigrateStreamFormatToVersion1(&input, &output);

  return EXIT_SUCCESS;
}
