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

#include "cartographer/cloud/map_builder_server_options.h"

#include "cartographer/common/configuration_file_resolver.h"
#include "cartographer/common/make_unique.h"
#include "cartographer/mapping/map_builder.h"

namespace cartographer {
namespace cloud {

proto::MapBuilderServerOptions CreateMapBuilderServerOptions(
    common::LuaParameterDictionary* lua_parameter_dictionary) {
  proto::MapBuilderServerOptions map_builder_server_options;
  map_builder_server_options.set_server_address(
      lua_parameter_dictionary->GetString("server_address"));
  map_builder_server_options.set_num_grpc_threads(
      lua_parameter_dictionary->GetInt("num_grpc_threads"));
  map_builder_server_options.set_num_event_threads(
      lua_parameter_dictionary->GetInt("num_event_threads"));
  *map_builder_server_options.mutable_map_builder_options() =
      mapping::CreateMapBuilderOptions(
          lua_parameter_dictionary->GetDictionary("map_builder").get());
  map_builder_server_options.set_uplink_server_address(
      lua_parameter_dictionary->GetString("uplink_server_address"));
  map_builder_server_options.set_upload_batch_size(
      lua_parameter_dictionary->GetInt("upload_batch_size"));
  map_builder_server_options.set_enable_ssl_encryption(
      lua_parameter_dictionary->GetBool("enable_ssl_encryption"));
  return map_builder_server_options;
}

proto::MapBuilderServerOptions LoadMapBuilderServerOptions(
    const std::string& configuration_directory,
    const std::string& configuration_basename) {
  auto file_resolver = common::make_unique<common::ConfigurationFileResolver>(
      std::vector<std::string>{configuration_directory});
  const std::string code =
      file_resolver->GetFileContentOrDie(configuration_basename);
  common::LuaParameterDictionary lua_parameter_dictionary(
      code, std::move(file_resolver));
  return CreateMapBuilderServerOptions(&lua_parameter_dictionary);
}

}  // namespace cloud
}  // namespace cartographer
