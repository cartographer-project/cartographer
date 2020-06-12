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

#include "cartographer/mapping/detect_floors.h"

#include <algorithm>
#include <fstream>
#include <vector>

#include "Eigen/Core"
#include "cartographer/transform/transform.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping {

namespace {

// A union-find structure for assigning levels to 'spans' in the trajectory
// implemented as a disjoint-set forest:
// https://en.wikipedia.org/wiki/Disjoint-set_data_structure#Disjoint-set_forests
// TODO(hrapp): We use this elsewhere. Pull out into a common place.
using Levels = std::map<int, int>;

constexpr double kMaxShortSpanLengthMeters = 25.;
constexpr double kLevelHeightMeters = 2.5;
constexpr double kMinLevelSeparationMeters = 1.;

// Indices into 'trajectory.node', so that 'start_index' <= i < 'end_index'.
struct Span {
  int start_index;
  int end_index;
  std::vector<double> z_values;

  bool operator<(const Span& other) const {
    return std::forward_as_tuple(start_index, end_index) <
           std::forward_as_tuple(other.start_index, other.end_index);
  }
};

// Union-find implementation for classifying spans into levels.
int LevelFind(const int i, const Levels& levels) {
  auto it = levels.find(i);
  CHECK(it != levels.end());
  if (it->first == it->second) {
    return it->second;
  }
  return LevelFind(it->second, levels);
}

void LevelUnion(int i, int j, Levels* levels) {
  const int repr_i = LevelFind(i, *levels);
  const int repr_j = LevelFind(j, *levels);
  (*levels)[repr_i] = repr_j;
}

void InsertSorted(const double val, std::vector<double>* vals) {
  vals->insert(std::upper_bound(vals->begin(), vals->end(), val), val);
}

double Median(const std::vector<double>& sorted) {
  CHECK(!sorted.empty());
  return sorted.at(sorted.size() / 2);
}

// Cut the trajectory at jumps in z. A new span is started when the current
// node's z differes by more than kLevelHeightMeters from the median z values.
std::vector<Span> SliceByAltitudeChange(const proto::Trajectory& trajectory) {
  CHECK_GT(trajectory.node_size(), 0);

  std::vector<Span> spans;
  spans.push_back(Span{0, 0, {trajectory.node(0).pose().translation().z()}});
  for (int i = 1; i < trajectory.node_size(); ++i) {
    const auto& node = trajectory.node(i);
    const double z = node.pose().translation().z();
    if (std::abs(Median(spans.back().z_values) - z) > kLevelHeightMeters) {
      spans.push_back(Span{i, i, {}});
    }
    InsertSorted(z, &spans.back().z_values);
    spans.back().end_index = i + 1;
  }
  return spans;
}

// Returns the length of 'span' in meters.
double SpanLength(const proto::Trajectory& trajectory, const Span& span) {
  double length = 0;
  for (int i = span.start_index + 1; i < span.end_index; ++i) {
    const auto a =
        transform::ToEigen(trajectory.node(i - 1).pose().translation());
    const auto b = transform::ToEigen(trajectory.node(i).pose().translation());
    length += (a - b).head<2>().norm();
  }
  return length;
}

// True if 'span' is considered to be short, i.e. not interesting on its own,
// but should be folded into the levels before and after entering it.
bool IsShort(const proto::Trajectory& trajectory, const Span& span) {
  return SpanLength(trajectory, span) < kMaxShortSpanLengthMeters;
}

// Merges all 'spans' that have similar median z value into the same level.
void GroupSegmentsByAltitude(const proto::Trajectory& trajectory,
                             const std::vector<Span>& spans, Levels* levels) {
  for (size_t i = 0; i < spans.size(); ++i) {
    for (size_t j = i + 1; j < spans.size(); ++j) {
      if (std::abs(Median(spans[i].z_values) - Median(spans[j].z_values)) <
          kMinLevelSeparationMeters) {
        LevelUnion(i, j, levels);
      }
    }
  }
}

std::vector<Floor> FindFloors(const proto::Trajectory& trajectory,
                              const std::vector<Span>& spans,
                              const Levels& levels) {
  std::map<int, std::vector<Span>> level_spans;

  // Initialize the levels to start out with only long spans.
  for (size_t i = 0; i < spans.size(); ++i) {
    const Span& span = spans[i];
    if (!IsShort(trajectory, span)) {
      level_spans[LevelFind(i, levels)].push_back(span);
    }
  }

  for (size_t i = 0; i < spans.size(); ++i) {
    const Span& span = spans[i];
    if (!IsShort(trajectory, span)) {
      continue;
    }

    // If we have a long piece on this floor already, merge this short piece
    // into it.
    int level = LevelFind(i, levels);
    if (!level_spans[level].empty()) {
      level_spans[level].push_back(span);
      continue;
    }

    // Otherwise, add this short piece to the level before and after it. It is
    // likely some intermediate level on stairs.
    size_t index = i - 1;
    if (index < spans.size()) {
      level_spans[LevelFind(index, levels)].push_back(span);
    }
    index = i + 1;
    if (index < spans.size()) {
      level_spans[LevelFind(index, levels)].push_back(span);
    }
  }

  // Convert the level_spans structure to 'Floor'.
  std::vector<Floor> floors;
  for (auto& level : level_spans) {
    if (level.second.empty()) {
      continue;
    }

    std::vector<double> z_values;
    std::sort(level.second.begin(), level.second.end());
    floors.emplace_back();
    for (const auto& span : level.second) {
      if (!IsShort(trajectory, span)) {
        // To figure out the median height of this floor, we only care for the
        // long pieces that are guaranteed to be in the structure. This is a
        // heuristic to leave out intermediate (short) levels.
        z_values.insert(z_values.end(), span.z_values.begin(),
                        span.z_values.end());
      }
      floors.back().timespans.push_back(Timespan{
          common::FromUniversal(trajectory.node(span.start_index).timestamp()),
          common::FromUniversal(
              trajectory.node(span.end_index - 1).timestamp())});
    }
    if (!z_values.empty()) {
      std::sort(z_values.begin(), z_values.end());
      floors.back().z = Median(z_values);
    } else {
      LOG(ERROR) << "All spans in level are short";
      floors.pop_back();
    }
  }
  return floors;
}

}  // namespace

std::vector<Floor> DetectFloors(const proto::Trajectory& trajectory) {
  const std::vector<Span> spans = SliceByAltitudeChange(trajectory);
  Levels levels;
  for (size_t i = 0; i < spans.size(); ++i) {
    levels[i] = i;
  }
  GroupSegmentsByAltitude(trajectory, spans, &levels);

  std::vector<Floor> floors = FindFloors(trajectory, spans, levels);
  std::sort(floors.begin(), floors.end(),
            [](const Floor& a, const Floor& b) { return a.z < b.z; });
  return floors;
}

}  // namespace mapping
}  // namespace cartographer
