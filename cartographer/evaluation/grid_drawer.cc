
#include "cartographer/evaluation/grid_drawer.h"

namespace cartographer {
namespace evaluation {
namespace {}  // namespace

GridDrawer::GridDrawer() : scale_(4.0) {
  min_x_ = -20.f;
  min_y_ = -1.f;
  max_x_ = 20.f;
  max_y_ = 1.f;
  z_ = 0.0f;
  max_tsd_ = 0.3f;
  resolution_ = 0.05f;
  scaled_num_x_cells_ = scale_ * (max_x_ - min_x_) / resolution_;
  scaled_num_y_cells_ = scale_ * (max_y_ - min_y_) / resolution_;
  grid_surface_ = cairo_image_surface_create(
      CAIRO_FORMAT_ARGB32, scaled_num_x_cells_, scaled_num_y_cells_);
  cairo_pattern_t* pattern = cairo_pattern_create_for_surface(grid_surface_);
  //  cairo_pattern_set_filter (pattern, CAIRO_FILTER_NEAREST);

  grid_surface_context_ = cairo_create(grid_surface_);
  //  cairo_set_antialias(grid_surface_context_, CAIRO_ANTIALIAS_NONE);
  //
  //  cairo_pattern_set_filter (cairo_get_source (grid_surface_context_),
  //  CAIRO_FILTER_NEAREST);
  //  cairo_device_to_user_distance(grid_surface_context_, &scale_, &scale_);
  cairo_set_source_rgba(grid_surface_context_, 1, 1, 1, 1);
  cairo_paint(grid_surface_context_);
}

void GridDrawer::DrawTSD(const cartographer::mapping::HybridGridTSDF& grid) {
  Eigen::Array3i zero_index = grid.GetCellIndex({min_x_, min_y_, z_});
  for (int ix = 0; ix < scaled_num_x_cells_; ++ix) {
    for (int iy = 0; iy < scaled_num_y_cells_; ++iy) {
      float r = 1.f;
      float g = 1.f;
      float b = 1.f;
      float normalized_tsdf =
          grid.GetTSD(zero_index + Eigen::Array3i({ix, iy, 0})) / max_tsd_;
      if (normalized_tsdf > 0.f) {
        g = 1. - std::pow(std::abs(normalized_tsdf), 0.7);
        b = g;
      } else {
        r = 1. - std::pow(std::abs(normalized_tsdf), 0.7);
        g = r;
      }
      cairo_set_source_rgb(grid_surface_context_, r, g, b);
      cairo_rectangle(grid_surface_context_, (float(ix) - 0.5f) * scale_,
                      ((float)iy - 0.5f) * scale_, scale_, scale_);
      cairo_fill(grid_surface_context_);
    }
  }
}

void GridDrawer::DrawInterpolatedTSD(
    const cartographer::mapping::HybridGridTSDF& grid) {
  mapping::scan_matching::InterpolatedTSDF interpolated_tsdf(grid);
  for (int ix = 0; ix < scaled_num_x_cells_; ++ix) {
    for (int iy = 0; iy < scaled_num_y_cells_; ++iy) {
      float r = 1.f;
      float g = 1.f;
      float b = 1.f;
      float normalized_tsdf =
          interpolated_tsdf.GetTSD(min_x_ + (max_x_ - min_x_) * double(ix) /
                                                double(scaled_num_x_cells_),
                                   min_y_ + (max_y_ - min_y_) * double(iy) /
                                                double(scaled_num_y_cells_),
                                   z_) /
          max_tsd_;

      if (normalized_tsdf > 0.f) {
        g = 1. - std::pow(std::abs(normalized_tsdf), 0.7);
        b = g;
      } else {
        r = 1. - std::pow(std::abs(normalized_tsdf), 0.7);
        g = r;
      }
      cairo_set_source_rgb(grid_surface_context_, r, g, b);
      cairo_rectangle(grid_surface_context_, float(ix), (float)iy, scale_,
                      scale_);
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
  for (int ix = 0; ix < scaled_num_x_cells_; ++ix) {
    for (int iy = 0; iy < scaled_num_y_cells_; ++iy) {
      float r = 1.f;
      float g = 1.f;
      float b = 1.f;
      float x =
          min_x_ + (max_x_ - min_x_) * double(ix) / double(scaled_num_x_cells_);
      float y =
          min_y_ + (max_y_ - min_y_) * double(iy) / double(scaled_num_y_cells_);

      const transform::Rigid3d initial_pose_estimate({x, y, z_},
                                                     {0.0, 0.0, 0.0, 0.0});
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
    if (std::abs(scan.position[2] - z_) < z_range) {
      float x =
          (scan.position[0] - min_x_) * scaled_num_x_cells_ / (max_x_ - min_x_);
      float y =
          (scan.position[1] - min_y_) * scaled_num_y_cells_ / (max_y_ - min_y_);
      float point_size = 1.5f * scale_;
      cairo_rectangle(grid_surface_context_, (x - 0.5 * point_size),
                      (y - 0.5 * point_size), point_size, point_size);
    }
  }
  cairo_fill(grid_surface_context_);
}

void GridDrawer::ToFile(std::string filename) {
  cairo_surface_write_to_png(grid_surface_, filename.c_str());
}

}  // namespace evaluation
}  // namespace cartographer