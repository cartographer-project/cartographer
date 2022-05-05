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

#include "cartographer/io/image.h"
#include "cartographer/transform/transform.h"

#include "viam/src/io/draw_trajectories.h"

namespace viam {
namespace io {

cartographer::io::UniqueCairoSurfacePtr DrawTrajectoryNodes(
        const cartographer::mapping::MapById<cartographer::mapping::NodeId,
                                              cartographer::mapping::TrajectoryNode>& trajectory_nodes,
        float resolution,
        cartographer::transform::Rigid3d slice_pose,
        cairo_surface_t* surface) {

  auto cr = cairo_create(surface);
  if (trajectory_nodes.size() == 0) {
    return cartographer::io::MakeUniqueCairoSurfacePtr(cairo_get_target(cr));
  }

  double kTrajectoryWidth = 1.;
  double kTrajectoryEndMarkers = 4;
  constexpr double kAlpha = 1.;

  cairo_set_line_width(cr, kTrajectoryWidth);

  // Keep track of the current position of the sensor/robot
  int last_traj = 0;
  double px_curr = 0;
  double py_curr = 0;

  // Draw trajectory paths
  for (const int trajectory_id : trajectory_nodes.trajectory_ids()) {
    cartographer::io::FloatColor color = viam::io::GetColor(trajectory_id);
    cairo_set_source_rgba(cr, color[0], color[1], color[2], kAlpha);
    cairo_set_operator(cr, CAIRO_OPERATOR_SOURCE);
    last_traj = trajectory_id;

    for (const auto& node : trajectory_nodes.trajectory(trajectory_id)) {
      
      const auto t_global_pose = node.data.global_pose;
      const Eigen::Vector3d pixel = t_global_pose.translation();
      double px =  (slice_pose.translation().y() - pixel.y())/resolution;
      double py = (slice_pose.translation().x() - pixel.x())/resolution;

      // Keep track of the current position of the sensor/robot
      px_curr = px;
      py_curr = py;

      cairo_line_to(cr, px, py);

    }
    cairo_stroke(cr);

  }

  // Draw start point for most recent trajectory
  bool print_first = true;
  for (const auto& node : trajectory_nodes.trajectory(last_traj)) {
    if (print_first) {
        auto t_global_pose = node.data.global_pose;
      const Eigen::Vector3d pixel = t_global_pose.translation();
      double px_start =  (slice_pose.translation().y() - pixel.y())/resolution;
      double py_start = (slice_pose.translation().x() - pixel.x())/resolution;
      cairo_arc(cr, px_start, py_start, kTrajectoryEndMarkers, 0, 2 * M_PI);
      cairo_fill(cr); 
      print_first = false;   
    }
  }

  // Draw end point for most recent trajectory
  cairo_arc(cr, px_curr, py_curr, kTrajectoryEndMarkers, 0, 2 * M_PI);
  cairo_fill(cr);

  // Draw origin point for submap
  double origin_x =  slice_pose.translation().y()/resolution;
  double origin_y = slice_pose.translation().x()/resolution;

  cairo_set_source_rgba(cr, 1.0, 0.0, 0.0, 0.5);
  cairo_arc(cr, origin_x, origin_y, kTrajectoryEndMarkers, 0, 2 * M_PI);
  cairo_fill(cr);

  return cartographer::io::MakeUniqueCairoSurfacePtr(cairo_get_target(cr));
}

}  // namespace io
}  // namespace viam
