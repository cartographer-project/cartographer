-- Copyright 2017 The Cartographer Authors
--
-- Licensed under the Apache License, Version 2.0 (the "License");
-- you may not use this file except in compliance with the License.
-- You may obtain a copy of the License at
--
--      http://www.apache.org/licenses/LICENSE-2.0
--
-- Unless required by applicable law or agreed to in writing, software
-- distributed under the License is distributed on an "AS IS" BASIS,
-- WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
-- See the License for the specific language governing permissions and
-- limitations under the License.

include "map_builder.lua"

MAP_BUILDER_SERVER = {
  map_builder = MAP_BUILDER,
  num_event_threads = 4,
  num_grpc_threads = 4,
  server_address = "0.0.0.0:50051",
  uplink_server_address = "",
  upload_batch_size = 100,
  enable_ssl_encryption = false,
  token_file_path = "",
}
