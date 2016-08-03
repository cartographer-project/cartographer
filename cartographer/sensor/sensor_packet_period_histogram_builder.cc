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

#include "cartographer/sensor/sensor_packet_period_histogram_builder.h"

#include "glog/logging.h"

namespace cartographer {
namespace sensor {
namespace {

string ToBucket(int ticks) {
  if (ticks < 1 * 10000) {
    return "<   1ms";
  } else if (ticks < 3 * 10000) {
    return "<   3ms";
  } else if (ticks < 5 * 10000) {
    return "<   5ms";
  } else if (ticks < 7 * 10000) {
    return "<   7ms";
  } else if (ticks < 10 * 10000) {
    return "<  10ms";
  } else if (ticks < 30 * 10000) {
    return "<  30ms";
  } else if (ticks < 100 * 10000) {
    return "< 100ms";
  } else if (ticks < 500 * 10000) {
    return "< 500ms";
  }
  return "> 500ms";
}

}  // namespace

void SensorPacketPeriodHistogramBuilder::Add(const int trajectory_id,
                                             const int64 timestamp,
                                             const string& frame_id) {
  if (histograms_.count(trajectory_id) == 0) {
    histograms_.emplace(trajectory_id,
                        std::unordered_map<string, common::BucketHistogram>());
  }
  if (histograms_.at(trajectory_id).count(frame_id) == 0) {
    histograms_.at(trajectory_id).emplace(frame_id, common::BucketHistogram());
  }
  const Key key = std::make_pair(trajectory_id, frame_id);
  if (last_timestamps_.count(key) != 0) {
    const int64 previous_timestamp = last_timestamps_.at(key);
    histograms_.at(trajectory_id)
        .at(frame_id)
        .Hit(ToBucket(timestamp - previous_timestamp));
  }
  last_timestamps_[key] = timestamp;
}

void SensorPacketPeriodHistogramBuilder::LogHistogramsAndClear() {
  for (const auto& trajectory_map_entry : histograms_) {
    LOG(INFO) << "Printing histograms for trajectory with id "
              << trajectory_map_entry.first;
    for (const auto& frame_id_to_histogram_map : trajectory_map_entry.second) {
      LOG(INFO) << "Sensor packet period histogram for '"
                << frame_id_to_histogram_map.first << "' from trajectory '"
                << trajectory_map_entry.first << "':\n"
                << frame_id_to_histogram_map.second.ToString();
    }
  }
  histograms_.clear();
  last_timestamps_.clear();
}

}  // namespace sensor
}  // namespace cartographer
