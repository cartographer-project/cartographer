/*
 * Copyright 2018 The Cartographer Authors
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

#ifndef CARTOGRAPHER_GROUND_TRUTH_AUTOGENERATE_GROUND_TRUTH_H_
#define CARTOGRAPHER_GROUND_TRUTH_AUTOGENERATE_GROUND_TRUTH_H_

#include "cartographer/ground_truth/proto/relations.pb.h"
#include "cartographer/mapping/proto/pose_graph.pb.h"

namespace cartographer {
namespace ground_truth {

// Generates GroundTruth proto from the given pose graph using the specified
// criteria parameters. See
// 'https://google-cartographer.readthedocs.io/en/latest/evaluation.html' for
// more details.
proto::GroundTruth GenerateGroundTruth(
    const mapping::proto::PoseGraph& pose_graph, double min_covered_distance,
    double outlier_threshold_meters, double outlier_threshold_radians);

}  // namespace ground_truth
}  // namespace cartographer

#endif  // CARTOGRAPHER_GROUND_TRUTH_AUTOGENERATE_GROUND_TRUTH_H
