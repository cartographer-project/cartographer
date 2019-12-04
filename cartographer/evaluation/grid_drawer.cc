
#include "cartographer/evaluation/grid_drawer.h"

namespace cartographer {
namespace evaluation {
namespace {}  // namespace

GridDrawer::GridDrawer() : scale_(6.0) {
  int scaled_num_x_cells = 100 * scale_;
  int scaled_num_y_cells = 100 * scale_;
  grid_surface_ = cairo_image_surface_create(
      CAIRO_FORMAT_ARGB32, scaled_num_x_cells, scaled_num_y_cells);
  cairo_pattern_t* pattern = cairo_pattern_create_for_surface(grid_surface_);
  //  cairo_pattern_set_filter (pattern, CAIRO_FILTER_NEAREST);

  grid_surface_context_ = cairo_create(grid_surface_);
  //  cairo_set_antialias(grid_surface_context_, CAIRO_ANTIALIAS_NONE);
  //
  //  cairo_pattern_set_filter (cairo_get_source (grid_surface_context_),
  //  CAIRO_FILTER_NEAREST);
  cairo_device_to_user_distance(grid_surface_context_, &scale_, &scale_);
  cairo_set_source_rgba(grid_surface_context_, 1, 1, 1, 1);
  cairo_paint(grid_surface_context_);
}

void GridDrawer::DrawTSD(const cartographer::mapping::HybridGridTSDF& grid) {
  //  int scaled_num_x_cells = limits_.cell_limits().num_y_cells;
  //  int scaled_num_y_cells = limits_.cell_limits().num_x_cells;
  //  for (int ix = 0; ix < scaled_num_x_cells; ++ix) {
  //    for (int iy = 0; iy < scaled_num_y_cells; ++iy) {
  //      float r = 1.f;
  //      float g = 1.f;
  //      float b = 1.f;
  //      float val = grid.GetTSD({iy, ix}) / grid.GetMaxCorrespondenceCost();
  //      if (val > 0.f) {
  //        g = 1. - std::pow(std::abs(val), 0.5);
  //        b = g;
  //      } else {
  //        r = 1. - std::pow(std::abs(val), 0.5);
  //        g = r;
  //      }
  //      if (std::abs(val) > 0.999) {
  //        r = 0.95f;
  //        g = 0.95f;
  //        b = 0.95f;
  //      }
  //      //      r = 0.2 + 0.6* std::abs(val);
  //      //      g = 0.2 + 0.6* std::abs(val);
  //      //      b = 0.2 + 0.6* std::abs(val);
  //      cairo_set_source_rgb(grid_surface_context_, r, g, b);
  //      //      cairo_pattern_set_filter (cairo_get_source
  //      //      (grid_surface_context_), CAIRO_FILTER_NEAREST);
  //      cairo_rectangle(grid_surface_context_, scale_ * (float(ix)),
  //                      scale_ * ((float)iy), scale_, scale_);
  //      cairo_fill(grid_surface_context_);
  //    }
  //  }
}

void GridDrawer::DrawWeights(
    const cartographer::mapping::HybridGridTSDF& grid) {
  //  int scaled_num_x_cells = limits_.cell_limits().num_y_cells * scale_;
  //  int scaled_num_y_cells = limits_.cell_limits().num_x_cells * scale_;
  //  for (int ix = 0; ix < scaled_num_x_cells; ++ix) {
  //    for (int iy = 0; iy < scaled_num_y_cells; ++iy) {
  //      float r = 1.f;
  //      float g = 1.f;
  //      float b = 1.f;
  //      float val =
  //          grid.GetWeight({iy, ix}) / grid.value_converter_->getMaxWeight();
  //      if (val > 0.f) {
  //        g = 1. - std::pow(std::abs(val), 0.5);
  //        b = g;
  //      } else {
  //        r = 1. - std::pow(std::abs(val), 0.5);
  //        g = r;
  //      }
  //      cairo_set_source_rgb(grid_surface_context_, r, g, b);
  //      cairo_rectangle(grid_surface_context_, scale_ * (float(ix)),
  //                      scale_ * ((float)iy), scale_, scale_);
  //      cairo_fill(grid_surface_context_);
  //    }
  //  }
}

void GridDrawer::DrawScan(
    const sensor::RangeData& range_data,
    const cartographer::transform::Rigid2d& initial_transform,
    const cartographer::transform::Rigid2d& matched_transform) {
  sensor::RangeData initial_pose_estimate_range_data =
      cartographer::sensor::TransformRangeData(
          range_data, transform::Embed3D(initial_transform.cast<float>()));
  sensor::RangeData matched_range_data =
      cartographer::sensor::TransformRangeData(
          range_data, transform::Embed3D(matched_transform.cast<float>()));

  //  double scale_ = 1. / limits_.resolution();
  //
  //  // Scan Points
  //  cairo_set_source_rgb(grid_surface_context_, 0.8, 0.0, 0);
  //  for (auto& scan : initial_pose_estimate_range_data.returns) {
  //    float x = scale_ * (limits_.max().x() - scan.position[0]);
  //    float y = scale_ * (limits_.max().y() - scan.position[1]);
  //    cairo_rectangle(grid_surface_context_, (x - 0.15) * scale_,
  //                    (y - 0.15) * scale_, 0.3 * scale_, 0.3 * scale_);
  //  }
  //  cairo_fill(grid_surface_context_);
  //
  //  cairo_set_source_rgb(grid_surface_context_, 0.0, 0.8, 0);
  //  for (auto& scan : matched_range_data.returns) {
  //    float x = scale_ * (limits_.max().x() - scan.position[0]);
  //    float y = scale_ * (limits_.max().y() - scan.position[1]);
  //    cairo_rectangle(grid_surface_context_, (x - 0.15) * scale_,
  //                    (y - 0.15) * scale_, 0.3 * scale_, 0.3 * scale_);
  //  }
  cairo_fill(grid_surface_context_);
}

void GridDrawer::DrawPointcloud(
    const sensor::PointCloud& range_data,
    const cartographer::transform::Rigid2d& initial_transform,
    const cartographer::transform::Rigid2d& matched_transform) {
  sensor::PointCloud initial_pose_estimate_range_data =
      cartographer::sensor::TransformPointCloud(
          range_data, transform::Embed3D(initial_transform.cast<float>()));
  sensor::PointCloud matched_range_data =
      cartographer::sensor::TransformPointCloud(
          range_data, transform::Embed3D(matched_transform.cast<float>()));

  //  double scale_ = 1. / limits_.resolution();

  // Scan Points
  //
  //  cairo_set_source_rgb(grid_surface_context_, 0.0, 0.0, 0);
  //  for (auto& scan : matched_range_data) {
  //    float x = scale_ * (limits_.max().x() - scan.position[0]);
  //    float y = scale_ * (limits_.max().y() - scan.position[1]);
  //    cairo_rectangle(grid_surface_context_, (x - 0.125) * scale_,
  //                    (y - 0.125) * scale_, 0.25 * scale_, 0.25 * scale_);
  //  }
  cairo_fill(grid_surface_context_);
}

void GridDrawer::ToFile(std::string filename) {
  cairo_surface_write_to_png(grid_surface_, filename.c_str());
}

}  // namespace evaluation
}  // namespace cartographer