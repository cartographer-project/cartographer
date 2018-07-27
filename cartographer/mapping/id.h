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
#include <iostream>
#include <iterator>
#include <limits>
#include <map>
#include <memory>
#include <ostream>
#include <tuple>
#include <vector>

#include "absl/memory/memory.h"
#include "cartographer/common/port.h"
#include "cartographer/common/time.h"
#include "cartographer/mapping/proto/pose_graph.pb.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping {
namespace internal {

template <class T>
auto GetTimeImpl(const T& t, int) -> decltype(t.time()) {
  return t.time();
}
template <class T>
auto GetTimeImpl(const T& t, unsigned) -> decltype(t.time) {
  return t.time;
}
template <class T>
common::Time GetTime(const T& t) {
  return GetTimeImpl(t, 0);
}

}  // namespace internal

// Uniquely identifies a trajectory node using a combination of a unique
// trajectory ID and a zero-based index of the node inside that trajectory.
struct NodeId {
  NodeId(int trajectory_id, int node_index)
      : trajectory_id(trajectory_id), node_index(node_index) {}

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

  void ToProto(proto::NodeId* proto) const {
    proto->set_trajectory_id(trajectory_id);
    proto->set_node_index(node_index);
  }
};

inline std::ostream& operator<<(std::ostream& os, const NodeId& v) {
  return os << "(" << v.trajectory_id << ", " << v.node_index << ")";
}

// Uniquely identifies a submap using a combination of a unique trajectory ID
// and a zero-based index of the submap inside that trajectory.
struct SubmapId {
  SubmapId(int trajectory_id, int submap_index)
      : trajectory_id(trajectory_id), submap_index(submap_index) {}

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

  void ToProto(proto::SubmapId* proto) const {
    proto->set_trajectory_id(trajectory_id);
    proto->set_submap_index(submap_index);
  }
};

inline std::ostream& operator<<(std::ostream& os, const SubmapId& v) {
  return os << "(" << v.trajectory_id << ", " << v.submap_index << ")";
}

template <typename IteratorType>
class Range {
 public:
  Range(const IteratorType& begin, const IteratorType& end)
      : begin_(begin), end_(end) {}

  IteratorType begin() const { return begin_; }
  IteratorType end() const { return end_; }

 private:
  IteratorType begin_;
  IteratorType end_;
};

// Reminiscent of std::map, but indexed by 'IdType' which can be 'NodeId' or
// 'SubmapId'.
// Note: This container will only ever contain non-empty trajectories. Trimming
// the last remaining node of a trajectory drops the trajectory.
template <typename IdType, typename DataType>
class MapById {
 private:
  struct MapByIndex;

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

    explicit ConstIterator(const MapById& map_by_id, const IdType& id)
        : current_trajectory_(map_by_id.trajectories_.find(id.trajectory_id)),
          end_trajectory_(map_by_id.trajectories_.end()) {
      if (current_trajectory_ != end_trajectory_) {
        current_data_ =
            current_trajectory_->second.data_.find(MapById::GetIndex(id));
        if (current_data_ == current_trajectory_->second.data_.end()) {
          current_trajectory_ = end_trajectory_;
        }
      }
    }

    IdDataReference operator*() const {
      CHECK(current_trajectory_ != end_trajectory_);
      return IdDataReference{
          IdType{current_trajectory_->first, current_data_->first},
          current_data_->second};
    }

    std::unique_ptr<const IdDataReference> operator->() const {
      return absl::make_unique<const IdDataReference>(this->operator*());
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

  class ConstTrajectoryIterator {
   public:
    using iterator_category = std::bidirectional_iterator_tag;
    using value_type = int;
    using difference_type = int64;
    using pointer = const int*;
    using reference = const int&;

    explicit ConstTrajectoryIterator(
        typename std::map<int, MapByIndex>::const_iterator current_trajectory)
        : current_trajectory_(current_trajectory) {}

    int operator*() const { return current_trajectory_->first; }

    ConstTrajectoryIterator& operator++() {
      ++current_trajectory_;
      return *this;
    }

    ConstTrajectoryIterator& operator--() {
      --current_trajectory_;
      return *this;
    }

    bool operator==(const ConstTrajectoryIterator& it) const {
      return current_trajectory_ == it.current_trajectory_;
    }

    bool operator!=(const ConstTrajectoryIterator& it) const {
      return !operator==(it);
    }

   private:
    typename std::map<int, MapByIndex>::const_iterator current_trajectory_;
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

  // Returns an iterator to the element at 'id' or the end iterator if it does
  // not exist.
  ConstIterator find(const IdType& id) const {
    return ConstIterator(*this, id);
  }

  // Inserts data (which must not exist already) into a trajectory.
  void Insert(const IdType& id, const DataType& data) {
    CHECK_GE(id.trajectory_id, 0);
    CHECK_GE(GetIndex(id), 0);
    auto& trajectory = trajectories_[id.trajectory_id];
    trajectory.can_append_ = false;
    CHECK(trajectory.data_.emplace(GetIndex(id), data).second);
  }

  // Removes the data for 'id' which must exist.
  void Trim(const IdType& id) {
    auto& trajectory = trajectories_.at(id.trajectory_id);
    const auto it = trajectory.data_.find(GetIndex(id));
    CHECK(it != trajectory.data_.end()) << id;
    if (std::next(it) == trajectory.data_.end()) {
      // We are removing the data with the highest index from this trajectory.
      // We assume that we will never append to it anymore. If we did, we would
      // have to make sure that gaps in indices are properly chosen to maintain
      // correct connectivity.
      trajectory.can_append_ = false;
    }
    trajectory.data_.erase(it);
    if (trajectory.data_.empty()) {
      trajectories_.erase(id.trajectory_id);
    }
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

  // Returns count of all elements.
  size_t size() const {
    size_t size = 0;
    for (const auto& item : trajectories_) {
      size += item.second.data_.size();
    }
    return size;
  }

  // Returns Range object for range-based loops over the nodes of a trajectory.
  Range<ConstIterator> trajectory(const int trajectory_id) const {
    return Range<ConstIterator>(BeginOfTrajectory(trajectory_id),
                                EndOfTrajectory(trajectory_id));
  }

  // Returns Range object for range-based loops over the trajectory IDs.
  Range<ConstTrajectoryIterator> trajectory_ids() const {
    return Range<ConstTrajectoryIterator>(
        ConstTrajectoryIterator(trajectories_.begin()),
        ConstTrajectoryIterator(trajectories_.end()));
  }

  ConstIterator begin() const { return BeginOfTrajectory(0); }
  ConstIterator end() const {
    return BeginOfTrajectory(std::numeric_limits<int>::max());
  }

  bool empty() const { return begin() == end(); }

  // Returns an iterator to the first element in the container belonging to
  // trajectory 'trajectory_id' whose time is not considered to go before
  // 'time', or EndOfTrajectory(trajectory_id) if all keys are considered to go
  // before 'time'.
  ConstIterator lower_bound(const int trajectory_id,
                            const common::Time time) const {
    if (SizeOfTrajectoryOrZero(trajectory_id) == 0) {
      return EndOfTrajectory(trajectory_id);
    }

    const std::map<int, DataType>& trajectory =
        trajectories_.at(trajectory_id).data_;
    if (internal::GetTime(std::prev(trajectory.end())->second) < time) {
      return EndOfTrajectory(trajectory_id);
    }
    auto left = trajectory.begin();
    auto right = std::prev(trajectory.end());
    while (left != right) {
      const int middle = left->first + (right->first - left->first) / 2;
      const auto lower_bound_middle = trajectory.lower_bound(middle);
      if (internal::GetTime(lower_bound_middle->second) < time) {
        left = std::next(lower_bound_middle);
      } else {
        right = lower_bound_middle;
      }
    }

    return ConstIterator(*this, IdType{trajectory_id, left->first});
  }

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
