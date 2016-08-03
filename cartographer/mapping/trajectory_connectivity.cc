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

#include "cartographer/mapping/trajectory_connectivity.h"

#include <algorithm>
#include <unordered_set>

#include "cartographer/mapping/proto/trajectory_connectivity.pb.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping {

TrajectoryConnectivity::TrajectoryConnectivity()
    : lock_(), forest_(), connection_map_() {}

void TrajectoryConnectivity::Add(const Submaps* trajectory) {
  CHECK(trajectory != nullptr);
  common::MutexLocker locker(&lock_);
  forest_.emplace(trajectory, trajectory);
}

void TrajectoryConnectivity::Connect(const Submaps* trajectory_a,
                                     const Submaps* trajectory_b) {
  CHECK(trajectory_a != nullptr);
  CHECK(trajectory_b != nullptr);
  common::MutexLocker locker(&lock_);
  Union(trajectory_a, trajectory_b);
  auto sorted_pair =
      std::minmax(trajectory_a, trajectory_b, std::less<const Submaps*>());
  ++connection_map_[sorted_pair];
}

void TrajectoryConnectivity::Union(const Submaps* const trajectory_a,
                                   const Submaps* const trajectory_b) {
  forest_.emplace(trajectory_a, trajectory_a);
  forest_.emplace(trajectory_b, trajectory_b);
  const Submaps* const representative_a = FindSet(trajectory_a);
  const Submaps* const representative_b = FindSet(trajectory_b);
  forest_[representative_a] = representative_b;
}

const Submaps* TrajectoryConnectivity::FindSet(
    const Submaps* const trajectory) {
  auto it = forest_.find(trajectory);
  CHECK(it != forest_.end());
  if (it->first != it->second) {
    it->second = FindSet(it->second);
  }
  return it->second;
}

bool TrajectoryConnectivity::TransitivelyConnected(
    const Submaps* trajectory_a, const Submaps* trajectory_b) {
  CHECK(trajectory_a != nullptr);
  CHECK(trajectory_b != nullptr);
  common::MutexLocker locker(&lock_);

  if (forest_.count(trajectory_a) == 0 || forest_.count(trajectory_b) == 0) {
    return false;
  }
  return FindSet(trajectory_a) == FindSet(trajectory_b);
}

std::vector<std::vector<const Submaps*>>
TrajectoryConnectivity::ConnectedComponents() {
  // Map from cluster exemplar -> growing cluster.
  std::unordered_map<const Submaps*, std::vector<const Submaps*>> map;
  common::MutexLocker locker(&lock_);
  for (const auto& trajectory_entry : forest_) {
    map[FindSet(trajectory_entry.first)].push_back(trajectory_entry.first);
  }

  std::vector<std::vector<const Submaps*>> result;
  result.reserve(map.size());
  for (auto& pair : map) {
    result.emplace_back(std::move(pair.second));
  }
  return result;
}

int TrajectoryConnectivity::ConnectionCount(const Submaps* trajectory_a,
                                            const Submaps* trajectory_b) {
  CHECK(trajectory_a != nullptr);
  CHECK(trajectory_b != nullptr);
  common::MutexLocker locker(&lock_);
  const auto it = connection_map_.find(
      std::minmax(trajectory_a, trajectory_b, std::less<const Submaps*>()));
  return it != connection_map_.end() ? it->second : 0;
}

proto::TrajectoryConnectivity ToProto(
    std::vector<std::vector<const Submaps*>> connected_components,
    std::unordered_map<const mapping::Submaps*, int> trajectory_indices) {
  proto::TrajectoryConnectivity proto;
  std::vector<std::vector<int>> connected_components_by_indices;
  for (const auto& connected_component : connected_components) {
    connected_components_by_indices.emplace_back();
    for (const mapping::Submaps* trajectory : connected_component) {
      connected_components_by_indices.back().push_back(
          trajectory_indices.at(trajectory));
    }
    std::sort(connected_components_by_indices.back().begin(),
              connected_components_by_indices.back().end());
  }
  std::sort(connected_components_by_indices.begin(),
            connected_components_by_indices.end());
  for (const auto& connected_component : connected_components_by_indices) {
    auto* proto_connected_component = proto.add_connected_component();
    for (const int trajectory_id : connected_component) {
      proto_connected_component->add_trajectory_id(trajectory_id);
    }
  }
  return proto;
}

proto::TrajectoryConnectivity::ConnectedComponent FindConnectedComponent(
    const proto::TrajectoryConnectivity& trajectory_connectivity,
    const int trajectory_id) {
  for (const auto& connected_component :
       trajectory_connectivity.connected_component()) {
    if (std::find(connected_component.trajectory_id().begin(),
                  connected_component.trajectory_id().end(),
                  trajectory_id) != connected_component.trajectory_id().end()) {
      return connected_component;
    }
  }

  proto::TrajectoryConnectivity::ConnectedComponent connected_component;
  connected_component.add_trajectory_id(trajectory_id);
  return connected_component;
}

}  // namespace mapping
}  // namespace cartographer
