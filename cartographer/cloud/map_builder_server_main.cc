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

#include "cartographer/cloud/map_builder_server_interface.h"
#include "cartographer/cloud/map_builder_server_options.h"
#include "cartographer/mapping/map_builder.h"
#include "cartographer/metrics/register.h"
#include "gflags/gflags.h"
#include "glog/logging.h"
#if USE_PROMETHEUS
#include "cartographer/cloud/metrics/prometheus/family_factory.h"
#include "prometheus/exposer.h"
#endif

DEFINE_string(configuration_directory, "",
              "First directory in which configuration files are searched, "
              "second is always the Cartographer installation to allow "
              "including files from there.");
DEFINE_string(configuration_basename, "",
              "Basename, i.e. not containing any directory prefix, of the "
              "configuration file.");

namespace cartographer {
namespace cloud {

void Run(const std::string& configuration_directory,
         const std::string& configuration_basename) {
#if USE_PROMETHEUS
  metrics::prometheus::FamilyFactory registry;
  ::cartographer::metrics::RegisterAllMetrics(&registry);
  RegisterMapBuilderServerMetrics(&registry);
  ::prometheus::Exposer exposer("0.0.0.0:9100");
  exposer.RegisterCollectable(registry.GetCollectable());
  LOG(INFO) << "Exposing metrics at http://localhost:9100/metrics";
#endif

  proto::MapBuilderServerOptions map_builder_server_options =
      LoadMapBuilderServerOptions(configuration_directory,
                                  configuration_basename);
  auto map_builder = absl::make_unique<mapping::MapBuilder>(
      map_builder_server_options.map_builder_options());
  std::unique_ptr<MapBuilderServerInterface> map_builder_server =
      CreateMapBuilderServer(map_builder_server_options,
                             std::move(map_builder));
  map_builder_server->Start();
  map_builder_server->WaitForShutdown();
}

}  // namespace cloud
}  // namespace cartographer

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  FLAGS_logtostderr = true;
  google::SetUsageMessage(
      "\n\n"
      "This program offers a MapBuilder service via a gRPC interface.\n");
  google::ParseCommandLineFlags(&argc, &argv, true);
  if (FLAGS_configuration_directory.empty() ||
      FLAGS_configuration_basename.empty()) {
    google::ShowUsageWithFlagsRestrict(argv[0], "map_builder_server");
    return EXIT_FAILURE;
  }
  cartographer::cloud::Run(FLAGS_configuration_directory,
                           FLAGS_configuration_basename);
}
