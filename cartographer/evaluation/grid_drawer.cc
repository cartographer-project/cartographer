
#include "cartographer/evaluation/grid_drawer.h"

namespace cartographer {
namespace evaluation {
namespace {}  // namespace

GridDrawer::GridDrawer() : scale_(4.0) {
  max_tsd_ = 0.3f;
  resolution_ = 0.05f;
  axis0_ = 0;
  axis1_ = 1;
  axis2_ = 2;
  min_limits_ = {-15.0, -15.0, 0.8};
  max_limits_ = {15.0, 15.0, 0.9};
  scaled_num_cells_ =
      ((scale_ / resolution_) * (max_limits_ - min_limits_)).cast<int>();
  grid_surface_ =
      cairo_image_surface_create(CAIRO_FORMAT_ARGB32, scaled_num_cells_[axis0_],
                                 scaled_num_cells_[axis1_]);
  cairo_pattern_t* pattern = cairo_pattern_create_for_surface(grid_surface_);
  grid_surface_context_ = cairo_create(grid_surface_);
  //  cairo_set_antialias(grid_surface_context_, CAIRO_ANTIALIAS_NONE);
  //  cairo_pattern_set_filter (cairo_get_source (grid_surface_context_),
  //  CAIRO_FILTER_NEAREST);
  //  cairo_device_to_user_distance(grid_surface_context_, &scale_, &scale_);
  cairo_set_source_rgba(grid_surface_context_, 1, 1, 1, 1);
  cairo_paint(grid_surface_context_);
}

GridDrawer::GridDrawer(const cartographer::mapping::HybridGridTSDF& grid)
    : scale_(4.0) {
  max_tsd_ = grid.ValueConverter().getMaxTSD();
  resolution_ = grid.resolution();
  axis0_ = 0;
  axis1_ = 1;
  axis2_ = 2;
  min_limits_ = {1.0, -12.0, 1.0};
  max_limits_ = {1.1, 12.0, 2.0};
  scaled_num_cells_ =
      ((scale_ / resolution_) * (max_limits_ - min_limits_)).cast<int>();
  grid_surface_ =
      cairo_image_surface_create(CAIRO_FORMAT_ARGB32, scaled_num_cells_[axis0_],
                                 scaled_num_cells_[axis1_]);
  cairo_pattern_t* pattern = cairo_pattern_create_for_surface(grid_surface_);
  grid_surface_context_ = cairo_create(grid_surface_);
  //  cairo_set_antialias(grid_surface_context_, CAIRO_ANTIALIAS_NONE);
  //  cairo_pattern_set_filter (cairo_get_source (grid_surface_context_),
  //  CAIRO_FILTER_NEAREST);
  //  cairo_device_to_user_distance(grid_surface_context_, &scale_, &scale_);
  cairo_set_source_rgba(grid_surface_context_, 1, 1, 1, 1);
  cairo_paint(grid_surface_context_);
}

void GridDrawer::DrawTSD(const cartographer::mapping::HybridGridTSDF& grid,
                         double z) {
  Eigen::Array3i zero_index = grid.GetCellIndex(
      {min_limits_[axis0_], min_limits_[axis1_], min_limits_[axis2_]});
  for (int ix = 0; ix < scaled_num_cells_[axis0_]; ++ix) {
    for (int iy = 0; iy < scaled_num_cells_[axis1_]; ++iy) {
      float r = 1.f;
      float g = 1.f;
      float b = 1.f;
      Eigen::Array3i index_shift = {0, 0, 0};
      index_shift[axis0_] = ix;
      index_shift[axis1_] = iy;
      float normalized_tsdf = grid.GetTSD(zero_index + index_shift) /
                              grid.ValueConverter().getMaxTSD();
      if (normalized_tsdf > 0.f) {
        g = 1. - std::pow(std::abs(normalized_tsdf), 1.0);  // 0.7
        b = g;
      } else {
        r = 1. - std::pow(std::abs(normalized_tsdf), 1.0);
        g = r;
      }
      if (grid.GetWeight(zero_index + index_shift) == 0.f) {
        r = 0.2;
        g = 0.2;
        b = 0.2;
      }
      cairo_set_source_rgb(grid_surface_context_, r, g, b);
      cairo_rectangle(grid_surface_context_, (float(ix) - 0.5f) * scale_,
                      ((float)iy - 0.5f) * scale_, scale_, scale_);
      cairo_fill(grid_surface_context_);
    }
  }
}

void GridDrawer::DrawInterpolatedTSD(
    const cartographer::mapping::HybridGridTSDF& grid, double z) {
  mapping::scan_matching::InterpolatedTSDF interpolated_tsdf(grid);
  for (int ix = 0; ix < scaled_num_cells_[axis0_]; ++ix) {
    for (int iy = 0; iy < scaled_num_cells_[axis1_]; ++iy) {
      float r = 1.f;
      float g = 1.f;
      float b = 1.f;
      double a0 = min_limits_[axis0_] +
                  (max_limits_[axis0_] - min_limits_[axis0_]) * double(ix) /
                      double(scaled_num_cells_[axis0_]);
      double a1 = min_limits_[axis1_] +
                  (max_limits_[axis1_] - min_limits_[axis1_]) * double(iy) /
                      double(scaled_num_cells_[axis1_]);
      double a2 = z;
      Eigen::Array3d pos = {a0, a1, a2};
      float normalized_tsdf =
          interpolated_tsdf.GetTSD(pos[axis0_], pos[axis1_], pos[axis2_]) /
          grid.ValueConverter().getMaxTSD();

      if (normalized_tsdf > 0.f) {
        g = 1. - std::pow(std::abs(normalized_tsdf), 0.5);
        b = g;
      } else {
        r = 1. - std::pow(std::abs(normalized_tsdf), 0.5);
        g = r;
      }

      if (interpolated_tsdf.GetWeight(pos[axis0_], pos[axis1_], pos[axis2_]) ==
          0.f) {
        r = 0.2;
        g = 0.2;
        b = 0.2;
      }
      cairo_set_source_rgb(grid_surface_context_, r, g, b);
      cairo_rectangle(grid_surface_context_, ix, iy, 1, 1);
      cairo_fill(grid_surface_context_);
    }
  }
}

void GridDrawer::DrawSinglePointCostFunction(
    const cartographer::mapping::HybridGridTSDF& grid,
    mapping::scan_matching::CeresScanMatcher3D& scan_matcher) {
  const Eigen::Vector3d target_translation({0.0, 0.0, 0.0});
  sensor::RangefinderPoint point;
  point.position = {0.f, 0.f, 0.f};
  sensor::PointCloud single_point = {point};
  double cost;
  std::vector<double> residuals;
  std::vector<double> jacobians;
  for (int ix = 0; ix < scaled_num_cells_[axis0_]; ++ix) {
    for (int iy = 0; iy < scaled_num_cells_[axis1_]; ++iy) {
      float r = 1.f;
      float g = 1.f;
      float b = 1.f;
      float x = min_limits_[axis0_] +
                (max_limits_[axis0_] - min_limits_[axis0_]) * double(ix) /
                    double(scaled_num_cells_[axis0_]);
      float y = min_limits_[axis1_] +
                (max_limits_[axis1_] - min_limits_[axis1_]) * double(iy) /
                    double(scaled_num_cells_[axis1_]);

      const transform::Rigid3d initial_pose_estimate(
          {x, y, min_limits_[axis2_]}, {0.0, 0.0, 0.0, 0.0});
      scan_matcher.Evaluate(target_translation, initial_pose_estimate,
                            {{&single_point, &grid}}, &cost, &residuals,
                            &jacobians);

      double cost_drawing_scale = 10.0;
      double scaled_cost = cost_drawing_scale * cost;

      g = 1. - std::pow(std::abs(scaled_cost), 0.7);
      b = g;

      cairo_set_source_rgb(grid_surface_context_, r, g, b);
      cairo_rectangle(grid_surface_context_, float(ix), float(iy), scale_,
                      scale_);
      cairo_fill(grid_surface_context_);
    }
  }
}
void GridDrawer::DrawPointcloud(
    const sensor::PointCloud& range_data,
    const cartographer::transform::Rigid3d& transform) {
  sensor::PointCloud matched_range_data =
      cartographer::sensor::TransformPointCloud(range_data,
                                                transform.cast<float>());
  cairo_set_source_rgb(grid_surface_context_, 0.0, 0.0, 0);
  float z_range = 0.6;
  for (auto& scan : matched_range_data) {
    if (std::abs(scan.position[axis2_]) < z_range) {
      float x = (scan.position[0] - min_limits_[axis0_]) *
                scaled_num_cells_[axis0_] /
                (max_limits_[axis0_] - min_limits_[axis0_]);
      float y = (scan.position[1] - min_limits_[axis1_]) *
                scaled_num_cells_[axis1_] /
                (max_limits_[axis1_] - min_limits_[axis1_]);
      float point_size = 0.5f * scale_;
      cairo_rectangle(grid_surface_context_, (x - 0.5 * point_size),
                      (y - 0.5 * point_size), point_size, point_size);
    }
  }
  cairo_fill(grid_surface_context_);
}


void GridDrawer::DrawPose(const cartographer::transform::Rigid3d& transform, float r, float g, float b) {


  cairo_set_source_rgb(grid_surface_context_, r, g, b);
  float z_range = 0.6;
      float x = (transform.translation()[axis0_] - min_limits_[axis0_]) *
          scaled_num_cells_[axis0_] /
          (max_limits_[axis0_] - min_limits_[axis0_]);
      float y = (transform.translation()[axis1_] - min_limits_[axis1_]) *
          scaled_num_cells_[axis1_] /
          (max_limits_[axis1_] - min_limits_[axis1_]);

  float point_size = 0.5f * scale_;
  cairo_rectangle(grid_surface_context_, (x - 0.5 * point_size),
                  (y - 0.5 * point_size), point_size, point_size);

  cairo_fill(grid_surface_context_);

}

void GridDrawer::ToFile(std::string filename) {
  cairo_surface_write_to_png(grid_surface_, filename.c_str());
}

}  // namespace evaluation
}  // namespace cartographer
