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

#ifndef CARTOGRAPHER_MAPPING_DETECT_FLOORS_H_
#define CARTOGRAPHER_MAPPING_DETECT_FLOORS_H_

#include "cartographer/common/time.h"
#include "cartographer/mapping/proto/trajectory.pb.h"

namespace cartographer {
namespace mapping {

struct Timespan {
  common::Time start;
  common::Time end;
};

struct Floor {
  // The spans of time we spent on this floor. Since we might have walked up and
  // down many times in this place, there can be many spans of time we spent on
  // a particular floor.
  std::vector<Timespan> timespans;

  // The median z-value of this floor.
  double z;
};

// Uses a heuristic which looks at z-values of the poses to detect individual
// floors of a building. This requires that floors are *mostly* on the same
// z-height and that level changes happen *relatively* abrubtly, e.g. by taking
// the stairs.
std::vector<Floor> DetectFloors(const proto::Trajectory& trajectory);

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_DETECT_FLOORS_H_
