#ifndef CARTOGRAPHER_EVALUATION_GRID_DRAWER_H
#define CARTOGRAPHER_EVALUATION_GRID_DRAWER_H

#include "cartographer/mapping/3d/hybrid_grid_tsdf.h"
#include "cartographer/mapping/internal/3d/scan_matching/ceres_scan_matcher_3d.h"
#include "cartographer/mapping/internal/3d/scan_matching/interpolated_tsdf.h"
#include "cartographer/sensor/point_cloud.h"
#include "cartographer/sensor/range_data.h"

#include "cairo/cairo-svg.h"
#include "cairo/cairo.h"

namespace cartographer {
namespace evaluation {

class GridDrawer {
 public:
  GridDrawer();

  void DrawTSD(const cartographer::mapping::HybridGridTSDF& grid);

  void DrawInterpolatedTSD(const cartographer::mapping::HybridGridTSDF& grid);

  void DrawSinglePointCostFunction(
      const cartographer::mapping::HybridGridTSDF& grid,
      mapping::scan_matching::CeresScanMatcher3D& scan_matcher);

  //  void DrawWeights(const cartographer::mapping::HybridGridTSDF& grid);
  //
  //  void DrawScan(const sensor::RangeData& range_data,
  //                const cartographer::transform::Rigid2d& initial_transform,
  //                const cartographer::transform::Rigid2d& matched_transform);

  void DrawPointcloud(const sensor::PointCloud& range_data,
                      const cartographer::transform::Rigid3d& transform);

  void ToFile(std::string filename);

 private:
  cairo_surface_t* grid_surface_;
  cairo_t* grid_surface_context_;
  double scale_;
  int scaled_num_x_cells_;
  int scaled_num_y_cells_;
  double min_x_;
  double min_y_;
  double max_x_;
  double max_y_;
  double z_;
  float max_tsd_;
  float resolution_;
};

}  // namespace evaluation
}  // namespace cartographer
#endif  // CARTOGRAPHER_EVALUATION_GRID_DRAWER_H
