/*
 * Copyright 2016 The Cartographer Authors
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

#include "cartographer/io/map_io.h"

namespace cartographer {
namespace io {

//constructor takes a const reference to the map_builder
cartographer::io::MapIO::MapIO(const cartographer::mapping::MapBuilder
    &map_builder){}

bool MapIO::Write(const string file_name){
  // TODO(brandon-northcutt) Pause optimization so that submaps, trajectories,
  //  and constraints are immutable for serialization of state.

  // TODO(brandon-northcutt) Get a file handle to send protobufs to.

  // TODO(brandon-northcutt) Send map_builder_options protobuf to file first, so
  //  deserialization constructor can create a map builder object as it's first
  //  action.
  
  // TODO(brandon-northcutt) Serialize all other single copy options protobufs
  //  ceres_scan_matcher_options, range_data_inserter_options,
  //  sparse_pose_graph_options, real_time_correlative_scan_matcher_options,
  //  kalman_local_trajectory_builder_options, local_trajectory_builder_options,
  //  motion_filter_options, optimizing_local_trajectory_builder_options, 
  //  adaptive_voxel_filter_options,...)

  // TODO(brandon-northcutt) Iterate trajectories { serialize trajectory builder
  //  options, serialize sensor configuration proto, serialize all nodes in this
  //  trajectory.
  
  // TODO(brandon-northcutt) Serialize trajectory connectivity.

  // TODO(brandon-northcutt) Iterate submaps(serialize submap options, serialize
  //  probability grids.
  
  // TODO(brandon-northcutt) Serialize the sparse pose graph,  (Should the SPG
  //  proto contain trajectories?)
  return true;
}

bool MapIO::Read(const string file_name){
  // TODO(brandon-northcutt) Same steps as write but feed protobufs into
  //  the appropriate constructors.
  return true;
}

} // namespace io
} // namespace cartographer
