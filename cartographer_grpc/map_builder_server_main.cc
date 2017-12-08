/*
 * Copyright 2017 The Cartographer Authors
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

#include "cartographer_grpc/map_builder_server.h"
#include "cartographer_grpc/map_builder_server_options.h"
#include "gflags/gflags.h"
#include "glog/logging.h"

DEFINE_string(configuration_directory, "",
              "First directory in which configuration files are searched, "
              "second is always the Cartographer installation to allow "
              "including files from there.");
DEFINE_string(configuration_basename, "",
              "Basename, i.e. not containing any directory prefix, of the "
              "configuration file.");

namespace cartographer_grpc {

void Run(const std::string& configuration_directory,
         const std::string& configuration_basename) {
  proto::MapBuilderServerOptions map_builder_server_options =
      LoadMapBuilderServerOptions(configuration_directory,
                                  configuration_basename);
  MapBuilderServer map_builder_server(map_builder_server_options);
  map_builder_server.Start();
  map_builder_server.WaitForShutdown();
}

}  // namespace cartographer_grpc

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  FLAGS_logtostderr = true;
  google::SetUsageMessage(
      "\n\n"
      "This program offers a MapBuilder service via a gRPC interface.\n");
  google::ParseCommandLineFlags(&argc, &argv, true);
  if (FLAGS_configuration_directory.empty() ||
      FLAGS_configuration_basename.empty()) {
    google::ShowUsageWithFlagsRestrict(argv[0], "cartographer_grpc_server");
    return EXIT_FAILURE;
  }
  cartographer_grpc::Run(FLAGS_configuration_directory,
                         FLAGS_configuration_basename);
}
