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

#ifndef CARTOGRAPHER_MAPPING_ID_H_
#define CARTOGRAPHER_MAPPING_ID_H_

#include <algorithm>
#include <ostream>
#include <tuple>
#include <vector>

namespace cartographer {
namespace mapping {

// Uniquely identifies a trajectory node using a combination of a unique
// trajectory ID and a zero-based index of the node inside that trajectory.
struct NodeId {
  int trajectory_id;
  int node_index;

  bool operator==(const NodeId& other) const {
    return std::forward_as_tuple(trajectory_id, node_index) ==
        std::forward_as_tuple(other.trajectory_id, other.node_index);
  }

  bool operator!=(const NodeId& other) const { return !operator==(other); }

  bool operator<(const NodeId& other) const {
    return std::forward_as_tuple(trajectory_id, node_index) <
           std::forward_as_tuple(other.trajectory_id, other.node_index);
  }
};

inline std::ostream& operator<<(std::ostream& os, const NodeId& v) {
  return os << "(" << v.trajectory_id << ", " << v.node_index << ")";
}

// Uniquely identifies a submap using a combination of a unique trajectory ID
// and a zero-based index of the submap inside that trajectory.
struct SubmapId {
  int trajectory_id;
  int submap_index;

  bool operator==(const SubmapId& other) const {
    return std::forward_as_tuple(trajectory_id, submap_index) ==
           std::forward_as_tuple(other.trajectory_id, other.submap_index);
  }

  bool operator!=(const SubmapId& other) const { return !operator==(other); }

  bool operator<(const SubmapId& other) const {
    return std::forward_as_tuple(trajectory_id, submap_index) <
           std::forward_as_tuple(other.trajectory_id, other.submap_index);
  }
};

inline std::ostream& operator<<(std::ostream& os, const SubmapId& v) {
  return os << "(" << v.trajectory_id << ", " << v.submap_index << ")";
}

template <typename ValueType, typename IdType>
class NestedVectorsById {
 public:
  // Appends data to a trajectory, creating trajectories as needed.
  IdType Append(int trajectory_id, const ValueType& value) {
    data_.resize(std::max<size_t>(data_.size(), trajectory_id + 1));
    const IdType id{trajectory_id,
                    static_cast<int>(data_[trajectory_id].size())};
    data_[trajectory_id].push_back(value);
    return id;
  }

  const ValueType& at(const IdType& id) const {
    return data_.at(id.trajectory_id).at(GetIndex(id));
  }
  ValueType& at(const IdType& id) {
    return data_.at(id.trajectory_id).at(GetIndex(id));
  }

  int num_trajectories() const { return static_cast<int>(data_.size()); }
  int num_indices(int trajectory_id) const {
    return static_cast<int>(data_.at(trajectory_id).size());
  }

  // TODO(whess): Remove once no longer needed.
  const std::vector<std::vector<ValueType>> data() const { return data_; }

 private:
  static int GetIndex(const NodeId& id) { return id.node_index; }
  static int GetIndex(const SubmapId& id) { return id.submap_index; }

  std::vector<std::vector<ValueType>> data_;
};

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_ID_H_
