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

#ifndef CARTOGRAPHER_ROS_GOOGLE_CARTOGRAPHER_SRC_NODE_CONSTANTS_H_
#define CARTOGRAPHER_ROS_GOOGLE_CARTOGRAPHER_SRC_NODE_CONSTANTS_H_

namespace cartographer_ros {

// The topic that the node will subscrbe under.
constexpr char kSubmapListTopic[] = "submap_list";

// The service we serve in the Node and query in the RViz plugin for submap
// which are used for visualization.
constexpr char kSubmapQueryServiceName[] = "submap_query";

}  // namespace cartographer_ros

#endif  // CARTOGRAPHER_ROS_GOOGLE_CARTOGRAPHER_SRC_NODE_CONSTANTS_H_
