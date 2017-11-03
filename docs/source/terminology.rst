.. Copyright 2017 The Cartographer Authors

.. Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

..      http://www.apache.org/licenses/LICENSE-2.0

.. Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License.

===========
Terminology
===========

This documents a few common patterns that exist in the Cartographer codebase.

Frames
======

global map frame
  This is the frame in which global SLAM results are expressed. It is the fixed
  map frame including all loop closure and optimization results. The transform
  between this frame and any other frame can jump when new optimization results
  are available. Its z-axis points upwards, i.e. the gravitational acceleration
  vector points in the -z direction, i.e. the gravitational component measured
  by an accelerometer is in the +z direction.

local map frame
  This is the frame in which local SLAM results are expressed. It is the fixed
  map frame excluding loop closures and the pose graph optimization. For a given
  point in time, the transform between this and the global map frame may change,
  but the transform between this and all other frames does not change.

submap frame
  Each submap has a separate fixed frame.

tracking frame
  The frame in which sensor data is expressed. It is not fixed, i.e. it changes
  over time. It is also different for different trajectories.

gravity-aligned frame
  Only used in 2D. A frame colocated with the tracking frame but with a
  different orientation that is approximately aligned with gravity, i.e. the
  gravitational acceleration vector points approximately in the -z direction. No
  assumption about yaw (rotation around the z axis between this and the tracking
  frame) should be made. A different gravity-aligned frame is used for different
  trajectory nodes, e.g. yaw can change arbitrarily between gravity-aligned
  frames of consecutive nodes.



Transforms
==========

local_pose
  Transforms data from the tracking frame (or a submap frame, depending on
  context) to the local map frame.

global_pose
  Transforms data from the tracking frame (or a submap frame, depending on
  context) to the global map frame.

local_submap_pose
  Transforms data from a submap frame to the local map frame.

global_submap_pose
  Transforms data from a submap frame to the global map frame.
