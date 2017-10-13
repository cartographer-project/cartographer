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
#include <iterator>
#include <limits>
#include <map>
#include <memory>
#include <ostream>
#include <tuple>
#include <vector>

#include "cartographer/common/make_unique.h"
#include "cartographer/common/port.h"
#include "glog/logging.h"

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

// Reminiscent of std::map, but indexed by 'IdType' which can be 'NodeId' or
// 'SubmapId'.
template <typename IdType, typename DataType>
class MapById {
 private:
  class MapByIndex;

 public:
  struct IdDataReference {
    IdType id;
    const DataType& data;
  };

  class ConstIterator {
   public:
    using iterator_category = std::bidirectional_iterator_tag;
    using value_type = IdDataReference;
    using difference_type = int64;
    using pointer = std::unique_ptr<const IdDataReference>;
    using reference = const IdDataReference&;

    explicit ConstIterator(const MapById& map_by_id, const int trajectory_id)
        : current_trajectory_(
              map_by_id.trajectories_.lower_bound(trajectory_id)),
          end_trajectory_(map_by_id.trajectories_.end()) {
      if (current_trajectory_ != end_trajectory_) {
        current_data_ = current_trajectory_->second.data_.begin();
        AdvanceToValidDataIterator();
      }
    }

    IdDataReference operator*() const {
      CHECK(current_trajectory_ != end_trajectory_);
      return IdDataReference{
          IdType{current_trajectory_->first, current_data_->first},
          current_data_->second};
    }

    std::unique_ptr<const IdDataReference> operator->() const {
      return common::make_unique<const IdDataReference>(this->operator*());
    }

    ConstIterator& operator++() {
      CHECK(current_trajectory_ != end_trajectory_);
      ++current_data_;
      AdvanceToValidDataIterator();
      return *this;
    }

    ConstIterator& operator--() {
      while (current_trajectory_ == end_trajectory_ ||
             current_data_ == current_trajectory_->second.data_.begin()) {
        --current_trajectory_;
        current_data_ = current_trajectory_->second.data_.end();
      }
      --current_data_;
      return *this;
    }

    bool operator==(const ConstIterator& it) const {
      if (current_trajectory_ == end_trajectory_ ||
          it.current_trajectory_ == it.end_trajectory_) {
        return current_trajectory_ == it.current_trajectory_;
      }
      return current_trajectory_ == it.current_trajectory_ &&
             current_data_ == it.current_data_;
    }

    bool operator!=(const ConstIterator& it) const { return !operator==(it); }

   private:
    void AdvanceToValidDataIterator() {
      CHECK(current_trajectory_ != end_trajectory_);
      while (current_data_ == current_trajectory_->second.data_.end()) {
        ++current_trajectory_;
        if (current_trajectory_ == end_trajectory_) {
          return;
        }
        current_data_ = current_trajectory_->second.data_.begin();
      }
    }

    typename std::map<int, MapByIndex>::const_iterator current_trajectory_;
    typename std::map<int, MapByIndex>::const_iterator end_trajectory_;
    typename std::map<int, DataType>::const_iterator current_data_;
  };

  // Appends data to a 'trajectory_id', creating trajectories as needed.
  IdType Append(const int trajectory_id, const DataType& data) {
    CHECK_GE(trajectory_id, 0);
    auto& trajectory = trajectories_[trajectory_id];
    CHECK(trajectory.can_append_);
    const int index =
        trajectory.data_.empty() ? 0 : trajectory.data_.rbegin()->first + 1;
    trajectory.data_.emplace(index, data);
    return IdType{trajectory_id, index};
  }

  // Inserts data (which must not exist already) into a trajectory.
  void Insert(const IdType& id, const DataType& data) {
    auto& trajectory = trajectories_[id.trajectory_id];
    trajectory.can_append_ = false;
    CHECK(trajectory.data_.emplace(GetIndex(id), data).second);
  }

  // Removes the data for 'id' which must exist.
  void Trim(const IdType& id) {
    auto& trajectory = trajectories_.at(id.trajectory_id);
    const auto it = trajectory.data_.find(GetIndex(id));
    CHECK(it != trajectory.data_.end());
    if (std::next(it) == trajectory.data_.end()) {
      // We are removing the data with the highest index from this trajectory.
      // We assume that we will never append to it anymore. If we did, we would
      // have to make sure that gaps in indices are properly chosen to maintain
      // correct connectivity.
      trajectory.can_append_ = false;
    }
    trajectory.data_.erase(it);
  }

  bool Contains(const IdType& id) const {
    return trajectories_.count(id.trajectory_id) != 0 &&
           trajectories_.at(id.trajectory_id).data_.count(GetIndex(id)) != 0;
  }

  const DataType& at(const IdType& id) const {
    return trajectories_.at(id.trajectory_id).data_.at(GetIndex(id));
  }

  DataType& at(const IdType& id) {
    return trajectories_.at(id.trajectory_id).data_.at(GetIndex(id));
  }

  // Support querying by trajectory.
  ConstIterator BeginOfTrajectory(const int trajectory_id) const {
    return ConstIterator(*this, trajectory_id);
  }
  ConstIterator EndOfTrajectory(const int trajectory_id) const {
    return BeginOfTrajectory(trajectory_id + 1);
  }

  // Returns 0 if 'trajectory_id' does not exist.
  size_t SizeOfTrajectoryOrZero(const int trajectory_id) const {
    return trajectories_.count(trajectory_id)
               ? trajectories_.at(trajectory_id).data_.size()
               : 0;
  }

  ConstIterator begin() const { return BeginOfTrajectory(0); }
  ConstIterator end() const {
    return BeginOfTrajectory(std::numeric_limits<int>::max());
  }

  bool empty() const { return begin() == end(); }

 private:
  struct MapByIndex {
    bool can_append_ = true;
    std::map<int, DataType> data_;
  };

  static int GetIndex(const NodeId& id) { return id.node_index; }
  static int GetIndex(const SubmapId& id) { return id.submap_index; }

  std::map<int, MapByIndex> trajectories_;
};

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_ID_H_
