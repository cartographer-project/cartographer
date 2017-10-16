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

global (map) frame
  This is the frame in which global SLAM results are expressed. It is the fixed
  map frame including all loop closure and optimization results. The transform
  between this frame and any other frame can jump when new optimization results
  are available. Its z-axis is pointing upwards.

local (map) frame
  This is the frame in which local SLAM results are expressed. It is the fixed
  map frame excluding loop closures and the pose graph optimization. The
  transform for a given point in time, between this frame and all other frames
  will never be changed later, the only exception being the global map frame.

tracking frame
  The frame in which all sensor data is expected to be.



Transforms
==========

local_pose
  Transforms data from the tracking frame to the local frame.

global_pose
  Transforms data from the tracking frame to the global frame.
