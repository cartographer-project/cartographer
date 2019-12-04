#include <chrono>
#include <fstream>
#include <random>
#include <string>

#include "cairo/cairo.h"

//#include "cartographer/common/lua_parameter_dictionary.h"
//#include "cartographer/common/lua_parameter_dictionary_test_helpers.h"
//#include "cartographer/common/time.h"
//#include "cartographer/evaluation/scan_cloud_generator.h"
//#include "cartographer/mapping/2d/probability_grid.h"
//#include "cartographer/mapping/2d/probability_grid_range_data_inserter_2d.h"
//#include "cartographer/mapping/2d/tsd_value_converter.h"
//#include "cartographer/mapping/2d/tsdf_2d.h"
//#include "cartographer/mapping/3d/hybrid_grid_tsdf.h"
//#include "cartographer/mapping/3d/range_data_inserter_3d.h"
//#include "cartographer/mapping/3d/tsdf_range_data_inserter_3d.h"
//#include "cartographer/mapping/internal/2d/normal_estimation_2d.h"
//#include
//"cartographer/mapping/internal/2d/scan_matching/ceres_scan_matcher_2d.h"
//#include
//"cartographer/mapping/internal/2d/scan_matching/occupied_space_cost_function_2d.h"
//#include "cartographer/mapping/internal/3d/scan_matching/interpolated_tsdf.h"
//#include
//"cartographer/mapping/internal/3d/scan_matching/ceres_scan_matcher_3d.h"
// namespace cartographer {
// namespace evaluation {
// namespace {
// static int rendered_grid_id = 0;
// static std::random_device rd;
// static std::default_random_engine e1(42);
// static mapping::ValueConversionTables conversion_tables;
//}  // namespace
//
// struct Sample {
//  cartographer::sensor::RangeData range_data;
//  cartographer::transform::Rigid3d ground_truth_pose;
//};
//
// void GenerateRangeData(const ScanCloudGenerator::ModelType model_type,
//                       const Eigen::Vector3d size, const double resolution,
//                       cartographer::sensor::RangeData& range_data,
//                       float noise_std_dev) {
//  cartographer::sensor::PointCloud scan_cloud;
//  ScanCloudGenerator test_set_generator(resolution);
//  switch (model_type) {
//    case ScanCloudGenerator::ModelType::NONE:
//      LOG(ERROR) << "ScanCloudGenerator::ModelType::NONE";
//      break;
//    case ScanCloudGenerator::ModelType::CUBE:
//      test_set_generator.generateCube(scan_cloud, size[0], noise_std_dev);
//      break;
//    case ScanCloudGenerator::ModelType::CUBOID:
//      test_set_generator.generateCuboid(scan_cloud, size[0], size[1], size[2],
//                                        noise_std_dev);
//      break;
//  }
//  range_data.returns = scan_cloud;
//  range_data.origin = Eigen::Vector3f{0, 0, 0};
//}
//
// Sample GenerateDefinedSample(double error_x, double error_y, double
// error_phi,
//                             const ScanCloudGenerator::ModelType model_type,
//                             const Eigen::Vector3d size,
//                             const double resolution,
//                             float cloud_noise_std_dev) {
//  cartographer::sensor::RangeData range_data;
//  GenerateRangeData(model_type, size, resolution, range_data,
//                    cloud_noise_std_dev);
//
//  double x = error_x;
//  double y = error_y;
//  double z = 0.0;
//  double rx = 0.0;
//  double ry = 0.0;
//  double rz = 0.0;
//  double rw = 1.0;
//  Sample sample;
//  sample.ground_truth_pose =
//      cartographer::transform::Rigid3d({x, y, z}, {rx, ry, rz, rw});
//  sensor::RangeData initial_pose_estimate_range_data =
//      cartographer::sensor::TransformRangeData(
//          range_data, sample.ground_truth_pose.cast<float>());
//  sample.range_data = initial_pose_estimate_range_data;
//  return sample;
//}
//
// void renderGridwithScan(
//    const cartographer::mapping::ProbabilityGrid& grid, const Sample& sample,
//    const cartographer::transform::Rigid2d& initial_transform,
//    const cartographer::transform::Rigid2d& matched_transform,
//    const cartographer::mapping::proto::RangeDataInserterOptions& options) {
//  sensor::RangeData initial_pose_estimate_range_data =
//      cartographer::sensor::TransformRangeData(
//          sample.range_data,
//          transform::Embed3D(initial_transform.cast<float>()));
//  sensor::RangeData matched_range_data =
//      cartographer::sensor::TransformRangeData(
//          sample.range_data,
//          transform::Embed3D(matched_transform.cast<float>()));
//
//  const cartographer::mapping::MapLimits& limits = grid.limits();
//  double scale = 1. / limits.resolution();
//  cairo_surface_t* grid_surface;
//  cairo_t* grid_surface_context;
//
//  int scaled_num_x_cells = limits.cell_limits().num_x_cells * scale;
//  int scaled_num_y_cells = limits.cell_limits().num_y_cells * scale;
//  grid_surface = cairo_image_surface_create(
//      CAIRO_FORMAT_ARGB32, scaled_num_x_cells, scaled_num_y_cells);
//  grid_surface_context = cairo_create(grid_surface);
//  cairo_device_to_user_distance(grid_surface_context, &scale, &scale);
//  for (int ix = 0; ix < scaled_num_x_cells; ++ix) {
//    for (int iy = 0; iy < scaled_num_y_cells; ++iy) {
//      float p = 1. - grid.GetProbability({iy, ix});
//      cairo_set_source_rgb(grid_surface_context, p, p, p);
//      cairo_rectangle(grid_surface_context, scale * (float(ix)),
//                      scale * ((float)iy), scale, scale);
//      cairo_fill(grid_surface_context);
//    }
//  }
//
//  cairo_set_source_rgb(grid_surface_context, 0.8, 0.0, 0);
//  for (auto& scan : initial_pose_estimate_range_data.returns) {
//    float x = scale * (limits.max().x() - scan.position[0]);
//    float y = scale * (limits.max().y() - scan.position[1]);
//    cairo_rectangle(grid_surface_context, (x - 0.15) * scale,
//                    (y - 0.15) * scale, 0.3 * scale, 0.3 * scale);
//  }
//  cairo_fill(grid_surface_context);
//
//  cairo_set_source_rgb(grid_surface_context, 0.0, 0.8, 0);
//  for (auto& scan : matched_range_data.returns) {
//    float x = scale * (limits.max().x() - scan.position[0]);
//    float y = scale * (limits.max().y() - scan.position[1]);
//    cairo_rectangle(grid_surface_context, (x - 0.15) * scale,
//                    (y - 0.15) * scale, 0.3 * scale, 0.3 * scale);
//  }
//
//  cairo_fill(grid_surface_context);
//
//  time_t seconds;
//  time(&seconds);
//  std::string filename =
//      "grid_with_inserted_cloud" + std::to_string(rendered_grid_id) + ".png";
//  rendered_grid_id++;
//  cairo_surface_write_to_png(grid_surface, filename.c_str());
//}
//
// void renderGridWeightswithScan(
//    const cartographer::mapping::TSDF2D& grid, const Sample& sample,
//    const cartographer::transform::Rigid2d& initial_transform,
//    const cartographer::transform::Rigid2d& matched_transform,
//    const cartographer::mapping::proto::RangeDataInserterOptions& options) {
//  sensor::RangeData initial_pose_estimate_range_data =
//      cartographer::sensor::TransformRangeData(
//          sample.range_data,
//          transform::Embed3D(initial_transform.cast<float>()));
//  sensor::RangeData matched_range_data =
//      cartographer::sensor::TransformRangeData(
//          sample.range_data,
//          transform::Embed3D(matched_transform.cast<float>()));
//
//  const cartographer::mapping::MapLimits& limits = grid.limits();
//  double scale = 1. / limits.resolution();
//  cairo_surface_t* grid_surface;
//  cairo_t* grid_surface_context;
//
//  int scaled_num_x_cells = limits.cell_limits().num_x_cells * scale;
//  int scaled_num_y_cells = limits.cell_limits().num_y_cells * scale;
//  grid_surface = cairo_image_surface_create(
//      CAIRO_FORMAT_ARGB32, scaled_num_x_cells, scaled_num_y_cells);
//  grid_surface_context = cairo_create(grid_surface);
//  cairo_device_to_user_distance(grid_surface_context, &scale, &scale);
//  for (int ix = 0; ix < scaled_num_x_cells; ++ix) {
//    for (int iy = 0; iy < scaled_num_y_cells; ++iy) {
//      float r = 1.f;
//      float g = 1.f;
//      float b = 1.f;
//      float normalized_tsdf =
//          grid.GetWeight({iy, ix}) /
//          options.tsdf_range_data_inserter_options_2d().maximum_weight();
//      if (normalized_tsdf > 0.f) {
//        g = 1. - std::pow(std::abs(normalized_tsdf), 0.5);
//        b = g;
//      } else {
//        r = 1. - std::pow(std::abs(normalized_tsdf), 0.5);
//        g = r;
//      }
//      cairo_set_source_rgb(grid_surface_context, r, g, b);
//      cairo_rectangle(grid_surface_context, scale * (float(ix)),
//                      scale * ((float)iy), scale, scale);
//      cairo_fill(grid_surface_context);
//    }
//  }
//
//  // Scan Points
//  cairo_set_source_rgb(grid_surface_context, 0.8, 0.0, 0);
//  for (auto& scan : initial_pose_estimate_range_data.returns) {
//    float x = scale * (limits.max().x() - scan.position[0]);
//    float y = scale * (limits.max().y() - scan.position[1]);
//    cairo_rectangle(grid_surface_context, (x - 0.15) * scale,
//                    (y - 0.15) * scale, 0.3 * scale, 0.3 * scale);
//  }
//  cairo_fill(grid_surface_context);
//
//  cairo_set_source_rgb(grid_surface_context, 0.0, 0.8, 0);
//  for (auto& scan : matched_range_data.returns) {
//    float x = scale * (limits.max().x() - scan.position[0]);
//    float y = scale * (limits.max().y() - scan.position[1]);
//    cairo_rectangle(grid_surface_context, (x - 0.15) * scale,
//                    (y - 0.15) * scale, 0.3 * scale, 0.3 * scale);
//  }
//  cairo_fill(grid_surface_context);
//
//  time_t seconds;
//  time(&seconds);
//  std::string filename = "grid_weights_with_inserted_cloud" +
//                         std::to_string(rendered_grid_id) + ".png";
//  rendered_grid_id++;
//  cairo_surface_write_to_png(grid_surface, filename.c_str());
//}
//
// void renderGridwithScanBase(
//    const cartographer::mapping::TSDF2D& grid, const Sample& sample,
//    const cartographer::transform::Rigid2d& initial_transform,
//    const cartographer::transform::Rigid2d& matched_transform,
//    const cartographer::mapping::proto::RangeDataInserterOptions& options) {
//  sensor::RangeData initial_pose_estimate_range_data =
//      cartographer::sensor::TransformRangeData(
//          sample.range_data,
//          transform::Embed3D(initial_transform.cast<float>()));
//  sensor::RangeData matched_range_data =
//      cartographer::sensor::TransformRangeData(
//          sample.range_data,
//          transform::Embed3D(matched_transform.cast<float>()));
//
//  const cartographer::mapping::MapLimits& limits = grid.limits();
//  double scale = 1. / limits.resolution();
//  cairo_surface_t* grid_surface;
//  cairo_t* grid_surface_context;
//
//  int scaled_num_x_cells = limits.cell_limits().num_x_cells * scale;
//  int scaled_num_y_cells = limits.cell_limits().num_y_cells * scale;
//  grid_surface = cairo_image_surface_create(
//      CAIRO_FORMAT_ARGB32, scaled_num_x_cells, scaled_num_y_cells);
//  grid_surface_context = cairo_create(grid_surface);
//  cairo_device_to_user_distance(grid_surface_context, &scale, &scale);
//  for (int ix = 0; ix < scaled_num_x_cells; ++ix) {
//    for (int iy = 0; iy < scaled_num_y_cells; ++iy) {
//      float r = 1.f;
//      float g = 1.f;
//      float b = 1.f;
//      float normalized_tsdf =
//          grid.GetTSD({iy, ix}) / grid.GetMaxCorrespondenceCost();
//      if (normalized_tsdf > 0.99f) {
//        r = 1.f;
//        g = 1.f;
//        b = 1.f;
//      } else if (normalized_tsdf < -0.99f) {
//        r = 0.7f;
//        g = 0.7f;
//        b = 0.7f;
//      } else if (normalized_tsdf > 0.f) {
//        g = 1. - std::pow(std::abs(normalized_tsdf), 0.7);
//        b = g;
//      } else {
//        r = 1. - std::pow(std::abs(normalized_tsdf), 0.7);
//        g = r;
//      }
//      cairo_set_source_rgb(grid_surface_context, r, g, b);
//      cairo_rectangle(grid_surface_context, scale * (float(ix)),
//                      scale * ((float)iy), scale, scale);
//      cairo_fill(grid_surface_context);
//    }
//  }
//
//  // Scan Points
//  cairo_set_source_rgb(grid_surface_context, 0.8, 0.0, 0);
//  for (auto& scan : initial_pose_estimate_range_data.returns) {
//    float x = scale * (limits.max().x() - scan.position[0]);
//    float y = scale * (limits.max().y() - scan.position[1]);
//    cairo_rectangle(grid_surface_context, (x - 0.15) * scale,
//                    (y - 0.15) * scale, 0.3 * scale, 0.3 * scale);
//  }
//  cairo_fill(grid_surface_context);
//
//  cairo_set_source_rgb(grid_surface_context, 0.0, 0.8, 0);
//  for (auto& scan : matched_range_data.returns) {
//    float x = scale * (limits.max().x() - scan.position[0]);
//    float y = scale * (limits.max().y() - scan.position[1]);
//    cairo_rectangle(grid_surface_context, (x - 0.15) * scale,
//                    (y - 0.15) * scale, 0.3 * scale, 0.3 * scale);
//  }
//  cairo_fill(grid_surface_context);
//
//  time_t seconds;
//  time(&seconds);
//  std::string filename =
//      "grid_with_inserted_cloud" + std::to_string(rendered_grid_id) + ".png";
//  rendered_grid_id++;
//  cairo_surface_write_to_png(grid_surface, filename.c_str());
//  //    renderGridWeightswithScan(grid, sample, initial_transform,
//  //    matched_transform,
//  //                              options);
//}
//
// void RunScanMatchingEvaluation() {
//  cartographer::mapping::proto::RangeDataInserterOptions3D
//      range_data_inserter_options;
//  auto parameter_dictionary_range_data_inserter =
//  common::MakeDictionary(R"text(
//        return {
//          hit_probability = 0.55,
//          miss_probability = 0.49,
//          num_free_space_voxels = 2,
//          range_data_inserter_type = "TSDF_INSERTER_3D",
//          truncation_distance = 0.3,
//        })text");
//  range_data_inserter_options =
//      cartographer::mapping::CreateRangeDataInserterOptions3D(
//          parameter_dictionary_range_data_inserter.get());
//  Sample sample =
//      GenerateDefinedSample(0.0, 0.0, 0.0,
//      ScanCloudGenerator::ModelType::CUBE,
//                            {1.0, 0.0, 0.0}, 0.03, 0.f);
//
//  mapping::ValueConversionTables conversion_tables_;
//  mapping::HybridGridTSDF hybrid_grid_tsdf(0.05, 0.3, 1000,
//                                           &conversion_tables_);
//  mapping::TSDFRangeDataInserter3D tsdf_range_data_inserter(
//      range_data_inserter_options);
//  tsdf_range_data_inserter.Insert(sample.range_data, &hybrid_grid_tsdf);
//
//  for (float x = -1.f; x < 1.f; x += 0.05) {
//    LOG(INFO) << "x\t" << x << "\ttsd\t"
//              << hybrid_grid_tsdf.GetWeight(
//                  hybrid_grid_tsdf.GetCellIndex({x, 0.0, 0.0}));
//  }
//  mapping::scan_matching::InterpolatedTSDF
//  interpolated_tsdf(hybrid_grid_tsdf); for (double x = -1.f; x < 1.f; x +=
//  0.01) {
//    LOG(INFO) << "x\t" << x << "\titsd\t"
//              << interpolated_tsdf.GetTSD(x, 0.0, 0.0);
//  }
//
//
//  cartographer::mapping::scan_matching::proto::CeresScanMatcherOptions3D
//      scan_matcher_options;
//  auto parameter_dictionary_scan_matcher = common::MakeDictionary(R"text(
//    return {
//      occupied_space_weight_0 = 1.,
//      translation_weight = 0.,
//      rotation_weight = 0.,
//      only_optimize_yaw = false,
//      ceres_solver_options = {
//        use_nonmonotonic_steps = false,
//        max_num_iterations = 120,
//        num_threads = 1,
//      },
//    })text");
//  scan_matcher_options =
//      cartographer::mapping::scan_matching::CreateCeresScanMatcherOptions3D(
//          parameter_dictionary_scan_matcher.get());
//  mapping::scan_matching::CeresScanMatcher3D
//  ceres_scan_matcher(scan_matcher_options); const Eigen::Vector3d
//  target_translation({0.0,0.0,0.0});
//  std::vector<mapping::scan_matching::PointCloudAndHybridGridPointers>
//      point_clouds_and_hybrid_grids;
//  double cost;
//  std::vector<double> residuals;
//  std::vector<double> jacobians;
//
//  sensor::RangefinderPoint point;
//  point.position = {0.f,0.f,0.f};
//  sensor::PointCloud single_point = {point};
//
//  for (float x = -0.9f; x < 0.901f; x += 0.01) {
//    const transform::Rigid3d initial_pose_estimate({x, 0.0, 0.0}, {0.0, 0.0,
//    0.0, 0.0}); ceres_scan_matcher.Evaluate(target_translation,
//                                initial_pose_estimate,
//                                {{&single_point, &hybrid_grid_tsdf}},
//                                &cost,
//                                &residuals,
//                                &jacobians);
//
//    LOG(INFO) << "x\t" << x << "\tcost\t"
//              <<cost;
//
////    transform::Rigid3d pose_estimate;
////    ceres::Solver::Summary summary;
////    ceres_scan_matcher.Match(target_translation,
////                             initial_pose_estimate,
////                             {{&(sample.range_data.returns),
///&hybrid_grid_tsdf}}, /                             &pose_estimate, /
///&summary); /    LOG(INFO)<<"match "<< pose_estimate.DebugString();
//  }
//
//}
//
//}  // namespace evaluation
//}  // namespace cartographer

int main(int argc, char** argv) {
  //  google::InitGoogleLogging(argv[0]);
  //  FLAGS_logtostderr = true;
  //  google::ParseCommandLineFlags(&argc, &argv, true);
  //  cartographer::evaluation::RunScanMatchingEvaluation();
}
