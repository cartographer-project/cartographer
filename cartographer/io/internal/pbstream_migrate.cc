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

#include <sstream>

#include "cartographer/io/proto_stream.h"
#include "cartographer/io/serialization_format_migration.h"
#include "gflags/gflags.h"
#include "glog/logging.h"

DEFINE_bool(include_unfinished_submaps, true,
            "Whether to include unfinished submaps in the output.");

namespace cartographer {
namespace io {

int pbstream_migrate(int argc, char** argv) {
  std::stringstream ss;
  ss << "\n\nTool for migrating files that use submaps without histograms "
        "to the new submap format, which includes a histogram. You can "
        "set --include_unfinished_submaps to false if you want to exclude "
        "unfinished submaps in the output."
     << "\nUsage: " << argv[0] << " " << argv[1]
     << " <input_filename> <output_filename> [flags]";
  google::SetUsageMessage(ss.str());

  if (argc < 4) {
    google::ShowUsageWithFlagsRestrict(argv[0], "pbstream_migrate");
    return EXIT_FAILURE;
  }
  cartographer::io::ProtoStreamReader input(argv[2]);
  cartographer::io::ProtoStreamWriter output(argv[3]);
  LOG(INFO) << "Migrating serialization format 1 in \"" << argv[2]
            << "\" to serialization format 2 in \"" << argv[3] << "\"";
  cartographer::io::MigrateStreamVersion1ToVersion2(
      &input, &output, FLAGS_include_unfinished_submaps);
  CHECK(output.Close()) << "Could not write migrated pbstream file to: "
                        << argv[3];

  return EXIT_SUCCESS;
}

}  // namespace io
}  // namespace cartographer
