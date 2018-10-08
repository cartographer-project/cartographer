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

#include <functional>

#include "absl/container/flat_hash_set.h"
#include "cartographer/io/internal/pbstream_info.h"
#include "cartographer/io/internal/pbstream_migrate.h"
#include "gflags/gflags.h"
#include "glog/logging.h"

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);

  FLAGS_logtostderr = true;
  const std::string usage_message =
      "Swiss Army knife for pbstreams.\n\n"
      "Currently supported subcommands are:\n"
      "\tinfo    - Prints summary of pbstream.\n"
      "\tmigrate - Migrates old pbstream (w/o header) to new pbstream format.";
  google::ParseCommandLineFlags(&argc, &argv, true);

  if (argc < 2) {
    google::SetUsageMessage(usage_message);
    google::ShowUsageWithFlagsRestrict(argv[0], "pbstream_info_main");
    return EXIT_FAILURE;
  } else if (std::string(argv[1]) == "info") {
    return ::cartographer::io::pbstream_info(argc, argv);
  } else if (std::string(argv[1]) == "migrate") {
    return ::cartographer::io::pbstream_migrate(argc, argv);
  } else {
    LOG(INFO) << "Unknown subtool: \"" << argv[1];
    google::SetUsageMessage(usage_message);
    google::ShowUsageWithFlagsRestrict(argv[0], "pbstream_info_main");
    return EXIT_FAILURE;
  }
}
