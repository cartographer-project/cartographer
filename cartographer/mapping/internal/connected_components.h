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

#ifndef CARTOGRAPHER_MAPPING_INTERNAL_CONNECTED_COMPONENTS_H_
#define CARTOGRAPHER_MAPPING_INTERNAL_CONNECTED_COMPONENTS_H_

#include <map>

#include "absl/container/flat_hash_map.h"
#include "absl/synchronization/mutex.h"
#include "cartographer/mapping/proto/connected_components.pb.h"
#include "cartographer/mapping/submaps.h"

namespace cartographer {
namespace mapping {

// A class that tracks the connectivity structure between trajectories.
//
// Connectivity includes both the count ("How many times have I _directly_
// connected trajectories i and j?") and the transitive connectivity.
//
// This class is thread-safe.
class ConnectedComponents {
 public:
  ConnectedComponents();

  ConnectedComponents(const ConnectedComponents&) = delete;
  ConnectedComponents& operator=(const ConnectedComponents&) = delete;

  // Add a trajectory which is initially connected to only itself.
  void Add(int trajectory_id) LOCKS_EXCLUDED(lock_);

  // Connect two trajectories. If either trajectory is untracked, it will be
  // tracked. This function is invariant to the order of its arguments. Repeated
  // calls to Connect increment the connectivity count.
  void Connect(int trajectory_id_a, int trajectory_id_b) LOCKS_EXCLUDED(lock_);

  // Determines if two trajectories have been (transitively) connected. If
  // either trajectory is not being tracked, returns false, except when it is
  // the same trajectory, where it returns true. This function is invariant to
  // the order of its arguments.
  bool TransitivelyConnected(int trajectory_id_a, int trajectory_id_b)
      LOCKS_EXCLUDED(lock_);

  // Return the number of _direct_ connections between 'trajectory_id_a' and
  // 'trajectory_id_b'. If either trajectory is not being tracked, returns 0.
  // This function is invariant to the order of its arguments.
  int ConnectionCount(int trajectory_id_a, int trajectory_id_b)
      LOCKS_EXCLUDED(lock_);

  // The trajectory IDs, grouped by connectivity.
  std::vector<std::vector<int>> Components() LOCKS_EXCLUDED(lock_);

  // The list of trajectory IDs that belong to the same connected component as
  // 'trajectory_id'.
  std::vector<int> GetComponent(int trajectory_id) LOCKS_EXCLUDED(lock_);

 private:
  // Find the representative and compresses the path to it.
  int FindSet(int trajectory_id) EXCLUSIVE_LOCKS_REQUIRED(lock_);
  void Union(int trajectory_id_a, int trajectory_id_b)
      EXCLUSIVE_LOCKS_REQUIRED(lock_);

  absl::Mutex lock_;
  // Tracks transitive connectivity using a disjoint set forest, i.e. each
  // entry points towards the representative for the given trajectory.
  std::map<int, int> forest_ GUARDED_BY(lock_);
  // Tracks the number of direct connections between a pair of trajectories.
  std::map<std::pair<int, int>, int> connection_map_ GUARDED_BY(lock_);
};

// Returns a proto encoding connected components.
proto::ConnectedComponents ToProto(
    std::vector<std::vector<int>> connected_components);

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_INTERNAL_CONNECTED_COMPONENTS_H_
