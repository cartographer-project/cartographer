#include <chrono>
#include <fstream>
#include <random>
#include <string>

#include "cairo/cairo.h"
#include "matplotlibcpp.h"

#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/common/lua_parameter_dictionary_test_helpers.h"
#include "cartographer/common/time.h"
#include "cartographer/evaluation/grid_drawer.h"
#include "cartographer/evaluation/scan_cloud_generator.h"
#include "cartographer/mapping/2d/probability_grid.h"
#include "cartographer/mapping/2d/tsd_value_converter.h"
#include "cartographer/mapping/2d/tsdf_2d.h"
#include "cartographer/mapping/3d/hybrid_grid_tsdf.h"
#include "cartographer/mapping/3d/range_data_inserter_3d.h"
#include "cartographer/mapping/3d/tsdf_range_data_inserter_3d.h"
#include "cartographer/mapping/internal/3d/scan_matching/ceres_scan_matcher_3d.h"
#include "cartographer/mapping/internal/3d/scan_matching/interpolated_tsdf.h"

namespace cartographer {
namespace evaluation {
namespace {
static int rendered_grid_id = 0;
static std::random_device rd;
static std::default_random_engine e1(42);
static mapping::ValueConversionTables conversion_tables;
}  // namespace

struct Sample {
  cartographer::sensor::RangeData range_data;
  cartographer::transform::Rigid3d ground_truth_pose;
};

void GenerateRangeData(const ScanCloudGenerator::ModelType model_type,
                       const Eigen::Vector3d size, const double resolution,
                       cartographer::sensor::RangeData& range_data,
                       float noise_std_dev) {
  cartographer::sensor::PointCloud scan_cloud;
  ScanCloudGenerator test_set_generator(resolution);
  switch (model_type) {
    case ScanCloudGenerator::ModelType::NONE:
      LOG(ERROR) << "ScanCloudGenerator::ModelType::NONE";
      break;
    case ScanCloudGenerator::ModelType::CUBE:
      test_set_generator.generateCube(scan_cloud, size[0], noise_std_dev);
      break;
    case ScanCloudGenerator::ModelType::CUBOID:
      test_set_generator.generateCuboid(scan_cloud, size[0], size[1], size[2],
                                        noise_std_dev);
      break;
  }
  range_data.returns = scan_cloud;
  range_data.origin = Eigen::Vector3f{0, 0, 0};
}


void GenerateRangeDataSlice(const ScanCloudGenerator::ModelType model_type,
                       const Eigen::Vector3d size, const double resolution,
                       cartographer::sensor::RangeData& range_data,
                       float noise_std_dev, float azimuth, float fov) {
  cartographer::sensor::PointCloud scan_cloud;
  ScanCloudGenerator test_set_generator(resolution);
  switch (model_type) {
    case ScanCloudGenerator::ModelType::NONE:
      LOG(ERROR) << "ScanCloudGenerator::ModelType::NONE";
      break;
    case ScanCloudGenerator::ModelType::CUBE:
      test_set_generator.generateCubeSlice(scan_cloud, size[0], noise_std_dev, azimuth, fov);
      break;
    case ScanCloudGenerator::ModelType::CUBOID:
      test_set_generator.generateCuboidSlice(scan_cloud, size[0], size[1], size[2],
                                        noise_std_dev, azimuth, fov);
      break;
  }
  range_data.returns = scan_cloud;
  range_data.origin = Eigen::Vector3f{0, 0, 0};
}

Sample GenerateDefinedSample(double error_x, double error_y, double error_phi,
                             const ScanCloudGenerator::ModelType model_type,
                             const Eigen::Vector3d size,
                             const double resolution,
                             float cloud_noise_std_dev) {
  cartographer::sensor::RangeData range_data;
  GenerateRangeData(model_type, size, resolution, range_data,
                    cloud_noise_std_dev);

  double x = error_x;
  double y = error_y;
  double z = 0.0;
  double rx = 0.0;
  double ry = 0.0;
  double rz = 0.0;
  double rw = 1.0;
  Sample sample;
  sample.ground_truth_pose =
      cartographer::transform::Rigid3d({x, y, z}, {rx, ry, rz, rw});
  sensor::RangeData initial_pose_estimate_range_data =
      cartographer::sensor::TransformRangeData(
          range_data, sample.ground_truth_pose.cast<float>());
  sample.range_data = initial_pose_estimate_range_data;
  return sample;
}

Sample GenerateDefinedSampleSlice(
    double error_x, double error_y, double error_phi,
    const ScanCloudGenerator::ModelType model_type, const Eigen::Vector3d size,
    const double resolution, float cloud_noise_std_dev, float azimuth,
    float fov) {
  cartographer::sensor::RangeData range_data;
  GenerateRangeDataSlice(model_type, size, resolution, range_data,
                         cloud_noise_std_dev, azimuth, fov);

  double x = error_x;
  double y = error_y;
  double z = 0.0;
  double rx = 0.0;
  double ry = 0.0;
  double rz = 0.0;
  double rw = 1.0;
  Sample sample;
  sample.ground_truth_pose =
      cartographer::transform::Rigid3d({x, y, z}, {rx, ry, rz, rw});
  sensor::RangeData initial_pose_estimate_range_data =
      cartographer::sensor::TransformRangeData(
          range_data, sample.ground_truth_pose.cast<float>());
  sample.range_data = initial_pose_estimate_range_data;
  return sample;
}

Sample GenerateDefinedScan(double error_x, double error_y, double error_phi,
                           const ScanCloudGenerator::ModelType model_type,
                           const Eigen::Vector3d size, const double resolution,
                           float cloud_noise_std_dev, float azimuth,
                           float fov) {
  cartographer::sensor::RangeData range_data;
  GenerateRangeDataSlice(model_type, size, resolution, range_data,
                         cloud_noise_std_dev, azimuth, fov);

  double x = error_x;
  double y = error_y;
  double z = 0.0;
  double rx = 0.0;
  double ry = 0.0;
  double rz = 0.0;
  double rw = 1.0;
  Sample sample;
  sample.ground_truth_pose =
      cartographer::transform::Rigid3d({x, y, z}, {rx, ry, rz, rw});
  sensor::RangeData initial_pose_estimate_range_data =
      cartographer::sensor::TransformRangeData(
          range_data, sample.ground_truth_pose.cast<float>());
  sample.range_data = initial_pose_estimate_range_data;
  return sample;
}

void renderGridwithScan(
    const cartographer::mapping::ProbabilityGrid& grid, const Sample& sample,
    const cartographer::transform::Rigid2d& initial_transform,
    const cartographer::transform::Rigid2d& matched_transform,
    const cartographer::mapping::proto::RangeDataInserterOptions& options) {
  sensor::RangeData initial_pose_estimate_range_data =
      cartographer::sensor::TransformRangeData(
          sample.range_data,
          transform::Embed3D(initial_transform.cast<float>()));
  sensor::RangeData matched_range_data =
      cartographer::sensor::TransformRangeData(
          sample.range_data,
          transform::Embed3D(matched_transform.cast<float>()));

  const cartographer::mapping::MapLimits& limits = grid.limits();
  double scale = 1. / limits.resolution();
  cairo_surface_t* grid_surface;
  cairo_t* grid_surface_context;

  int scaled_num_x_cells = limits.cell_limits().num_x_cells * scale;
  int scaled_num_y_cells = limits.cell_limits().num_y_cells * scale;
  grid_surface = cairo_image_surface_create(
      CAIRO_FORMAT_ARGB32, scaled_num_x_cells, scaled_num_y_cells);
  grid_surface_context = cairo_create(grid_surface);
  cairo_device_to_user_distance(grid_surface_context, &scale, &scale);
  for (int ix = 0; ix < scaled_num_x_cells; ++ix) {
    for (int iy = 0; iy < scaled_num_y_cells; ++iy) {
      float p = 1. - grid.GetProbability({iy, ix});
      cairo_set_source_rgb(grid_surface_context, p, p, p);
      cairo_rectangle(grid_surface_context, scale * (float(ix)),
                      scale * ((float)iy), scale, scale);
      cairo_fill(grid_surface_context);
    }
  }

  cairo_set_source_rgb(grid_surface_context, 0.8, 0.0, 0);
  for (auto& scan : initial_pose_estimate_range_data.returns) {
    float x = scale * (limits.max().x() - scan.position[0]);
    float y = scale * (limits.max().y() - scan.position[1]);
    cairo_rectangle(grid_surface_context, (x - 0.15) * scale,
                    (y - 0.15) * scale, 0.3 * scale, 0.3 * scale);
  }
  cairo_fill(grid_surface_context);

  cairo_set_source_rgb(grid_surface_context, 0.0, 0.8, 0);
  for (auto& scan : matched_range_data.returns) {
    float x = scale * (limits.max().x() - scan.position[0]);
    float y = scale * (limits.max().y() - scan.position[1]);
    cairo_rectangle(grid_surface_context, (x - 0.15) * scale,
                    (y - 0.15) * scale, 0.3 * scale, 0.3 * scale);
  }

  cairo_fill(grid_surface_context);

  time_t seconds;
  time(&seconds);
  std::string filename =
      "grid_with_inserted_cloud" + std::to_string(rendered_grid_id) + ".png";
  rendered_grid_id++;
  cairo_surface_write_to_png(grid_surface, filename.c_str());
}

void renderGridWeightswithScan(
    const cartographer::mapping::TSDF2D& grid, const Sample& sample,
    const cartographer::transform::Rigid2d& initial_transform,
    const cartographer::transform::Rigid2d& matched_transform,
    const cartographer::mapping::proto::RangeDataInserterOptions& options) {
  sensor::RangeData initial_pose_estimate_range_data =
      cartographer::sensor::TransformRangeData(
          sample.range_data,
          transform::Embed3D(initial_transform.cast<float>()));
  sensor::RangeData matched_range_data =
      cartographer::sensor::TransformRangeData(
          sample.range_data,
          transform::Embed3D(matched_transform.cast<float>()));

  const cartographer::mapping::MapLimits& limits = grid.limits();
  double scale = 1. / limits.resolution();
  cairo_surface_t* grid_surface;
  cairo_t* grid_surface_context;

  int scaled_num_x_cells = limits.cell_limits().num_x_cells * scale;
  int scaled_num_y_cells = limits.cell_limits().num_y_cells * scale;
  grid_surface = cairo_image_surface_create(
      CAIRO_FORMAT_ARGB32, scaled_num_x_cells, scaled_num_y_cells);
  grid_surface_context = cairo_create(grid_surface);
  cairo_device_to_user_distance(grid_surface_context, &scale, &scale);
  for (int ix = 0; ix < scaled_num_x_cells; ++ix) {
    for (int iy = 0; iy < scaled_num_y_cells; ++iy) {
      float r = 1.f;
      float g = 1.f;
      float b = 1.f;
      float normalized_tsdf =
          grid.GetWeight({iy, ix}) /
          options.tsdf_range_data_inserter_options_2d().maximum_weight();
      if (normalized_tsdf > 0.f) {
        g = 1. - std::pow(std::abs(normalized_tsdf), 0.5);
        b = g;
      } else {
        r = 1. - std::pow(std::abs(normalized_tsdf), 0.5);
        g = r;
      }
      cairo_set_source_rgb(grid_surface_context, r, g, b);
      cairo_rectangle(grid_surface_context, scale * (float(ix)),
                      scale * ((float)iy), scale, scale);
      cairo_fill(grid_surface_context);
    }
  }

  // Scan Points
  cairo_set_source_rgb(grid_surface_context, 0.8, 0.0, 0);
  for (auto& scan : initial_pose_estimate_range_data.returns) {
    float x = scale * (limits.max().x() - scan.position[0]);
    float y = scale * (limits.max().y() - scan.position[1]);
    cairo_rectangle(grid_surface_context, (x - 0.15) * scale,
                    (y - 0.15) * scale, 0.3 * scale, 0.3 * scale);
  }
  cairo_fill(grid_surface_context);

  cairo_set_source_rgb(grid_surface_context, 0.0, 0.8, 0);
  for (auto& scan : matched_range_data.returns) {
    float x = scale * (limits.max().x() - scan.position[0]);
    float y = scale * (limits.max().y() - scan.position[1]);
    cairo_rectangle(grid_surface_context, (x - 0.15) * scale,
                    (y - 0.15) * scale, 0.3 * scale, 0.3 * scale);
  }
  cairo_fill(grid_surface_context);

  time_t seconds;
  time(&seconds);
  std::string filename = "grid_weights_with_inserted_cloud" +
                         std::to_string(rendered_grid_id) + ".png";
  rendered_grid_id++;
  cairo_surface_write_to_png(grid_surface, filename.c_str());
}

void renderGrid(const cartographer::mapping::HybridGridTSDF& grid) {
  double min_x = -20.f;
  double min_y = -1.f;
  double max_x = 20.f;
  double max_y = 1.f;
  double z = 0.1f;
  float max_tsd = 0.3f;
  float resolution = 0.05f;
  mapping::scan_matching::InterpolatedTSDF interpolated_tsdf(grid);
  double scale = 4;
  cairo_surface_t* grid_surface;
  cairo_t* grid_surface_context;
  int scaled_num_x_cells = scale * (max_x - min_x) / resolution;
  int scaled_num_y_cells = scale * (max_y - min_y) / resolution;
  grid_surface = cairo_image_surface_create(
      CAIRO_FORMAT_ARGB32, scaled_num_x_cells, scaled_num_y_cells);
  grid_surface_context = cairo_create(grid_surface);
  //  cairo_device_to_user_distance(grid_surface_context, &scale, &scale);
  for (int ix = 0; ix < scaled_num_x_cells; ++ix) {
    for (int iy = 0; iy < scaled_num_y_cells; ++iy) {
      float r = 1.f;
      float g = 1.f;
      float b = 1.f;
      float normalized_tsdf =
          interpolated_tsdf.GetTSD(
              min_x + (max_x - min_x) * double(ix) / double(scaled_num_x_cells),
              min_y + (max_y - min_y) * double(iy) / double(scaled_num_y_cells),
              z) /
          max_tsd;

      if (normalized_tsdf > 0.f) {
        g = 1. - std::pow(std::abs(normalized_tsdf), 0.7);
        b = g;
      } else {
        r = 1. - std::pow(std::abs(normalized_tsdf), 0.7);
        g = r;
      }
      cairo_set_source_rgb(grid_surface_context, r, g, b);
      cairo_rectangle(grid_surface_context, float(ix), (float)iy, scale, scale);
      cairo_fill(grid_surface_context);
    }
  }
  time_t seconds;
  time(&seconds);
  std::string filename =
      "grid_with_inserted_cloud" + std::to_string(rendered_grid_id) + ".png";
  rendered_grid_id++;
  cairo_surface_write_to_png(grid_surface, filename.c_str());
}

void RunScanMatchingEvaluationRotatingScan() {
  cartographer::mapping::proto::RangeDataInserterOptions3D
      range_data_inserter_options;
  auto parameter_dictionary_range_data_inserter = common::MakeDictionary(R"text(
        return {
      range_data_inserter_type = "PROBABILITY_GRID_INSERTER_3D",
      probability_grid_range_data_inserter = {
        hit_probability = 0.55,
        miss_probability = 0.49,
        num_free_space_voxels = 2,
      },
      tsdf_range_data_inserter = {
        relative_truncation_distance = 4,
        maximum_weight = 1000.,
        num_free_space_voxels = 0,
        project_sdf_distance_to_scan_normal = true,
        weight_function_epsilon = 1,
        weight_function_sigma = 4.,
        normal_estimate_max_nn = 20.,
        normal_estimate_radius = 0.3,
      },})text");
  range_data_inserter_options =
      cartographer::mapping::CreateRangeDataInserterOptions3D(
          parameter_dictionary_range_data_inserter.get());
  Sample sample = GenerateDefinedSample(0.0, 0.0, 0.0,
                                        ScanCloudGenerator::ModelType::CUBOID,
                                        {17.25, 1.25, 1.25}, 0.031, 0.002f);

  mapping::ValueConversionTables conversion_tables_;
  mapping::HybridGridTSDF hybrid_grid_tsdf(
      0.05,
      range_data_inserter_options.tsdf_range_data_inserter_options_3d()
          .relative_truncation_distance(),
      range_data_inserter_options.tsdf_range_data_inserter_options_3d()
          .maximum_weight(),
      &conversion_tables_);
  mapping::TSDFRangeDataInserter3D tsdf_range_data_inserter(
      range_data_inserter_options);
  tsdf_range_data_inserter.Insert(sample.range_data, &hybrid_grid_tsdf);

  cartographer::mapping::scan_matching::proto::CeresScanMatcherOptions3D
      scan_matcher_options;
  auto parameter_dictionary_scan_matcher = common::MakeDictionary(R"text(
    return {
      occupied_space_weight_0 = 1.,
      translation_weight = 0.,
      rotation_weight = 0.,
      only_optimize_yaw = false,
      ceres_solver_options = {
        use_nonmonotonic_steps = false,
        max_num_iterations = 120,
        num_threads = 1,
      },
    })text");
  scan_matcher_options =
      cartographer::mapping::scan_matching::CreateCeresScanMatcherOptions3D(
          parameter_dictionary_scan_matcher.get());
  mapping::scan_matching::CeresScanMatcher3D ceres_scan_matcher(
      scan_matcher_options);
  const Eigen::Vector3d target_translation({0.0, 0.0, 0.0});
  std::vector<mapping::scan_matching::PointCloudAndHybridGridPointers>
      point_clouds_and_hybrid_grids;

  sensor::RangefinderPoint point;
  point.position = {0.f, 0.f, 0.f};
  sensor::PointCloud single_point = {point};

  //  for (float x = -0.4f; x < 0.401f; x += 0.01) {
  //    const transform::Rigid3d initial_pose_estimate({x, 0.0, 0.0},
  //                                                   {1.0, 0.0, 0.0, 0.0});
  //    transform::Rigid3d pose_estimate = transform::Rigid3d::Identity();
  //    ceres::Solver::Summary summary;
  //    ceres_scan_matcher.Match(
  //        target_translation, initial_pose_estimate,
  //        {{&(sample.range_data.returns), &hybrid_grid_tsdf}}, &pose_estimate,
  //        &summary);
  //    LOG(INFO) << "x " << x << " \tmatch \t" << pose_estimate.DebugString();
  //  }

  GridDrawer grid_drawer;
  grid_drawer.DrawInterpolatedTSD(hybrid_grid_tsdf, 0.2);
  grid_drawer.DrawPointcloud(sample.range_data.returns,
                             transform::Rigid3d::Identity());
  grid_drawer.ToFile("interpolated_grid_with_cloud.png");

  //  grid_drawer = GridDrawer();
  //  grid_drawer.DrawSinglePointCostFunction(hybrid_grid_tsdf,
  //  ceres_scan_matcher); grid_drawer.ToFile("cost.png");

  grid_drawer = GridDrawer();
  grid_drawer.DrawTSD(hybrid_grid_tsdf, 0.2);
  grid_drawer.ToFile("tsdf.png");
  LOG(INFO) << "wrote file";

  transform::Rigid3d initial_pose_estimate = transform::Rigid3d::Identity();
  transform::Rigid3d pose_estimate = transform::Rigid3d::Identity();
  int idx = 0;

  std::normal_distribution<float> normal_distribution(0,
                                                      0.05);  // 0.0
  std::vector<float> positions_x = {};
  std::vector<float> positions_y = {};
  std::vector<float> positions_z = {};

  for (double phi = -M_PI; phi < 10 * M_PI; phi += 0.05 * M_PI) {
    cartographer::transform::Rigid3d pertubation =
        cartographer::transform::Rigid3d(
            {normal_distribution(e1), normal_distribution(e1), 0},
            {1, 0, 0, 0});
    initial_pose_estimate = pertubation * initial_pose_estimate;

    Sample sample = GenerateDefinedSampleSlice(
        0.0, 0.0, 0.0, ScanCloudGenerator::ModelType::CUBOID,
        {17.25, 1.25, 1.25}, 0.031, 0.002f,
        std::fmod(phi + M_PI, 2.0 * M_PI) - M_PI, (1.0 / 6.0) * M_PI);
    ceres::Solver::Summary summary;
    ceres_scan_matcher.Match(
        target_translation, initial_pose_estimate,
        {{&(sample.range_data.returns), &hybrid_grid_tsdf}}, &pose_estimate,
        &summary);
    LOG(INFO) << "phi " << phi << " \tmatch \t" << pose_estimate.DebugString();

    grid_drawer = GridDrawer();
    grid_drawer.DrawInterpolatedTSD(hybrid_grid_tsdf, 0.2);
    grid_drawer.DrawPointcloud(sample.range_data.returns, pose_estimate);
    grid_drawer.DrawPose(initial_pose_estimate, 1, 0, 0);
    grid_drawer.DrawPose(pose_estimate, 0, 1, 0);
    grid_drawer.DrawPose(transform::Rigid3d::Identity(), 1, 1, 1);
    grid_drawer.ToFile("interpolated_grid_with_cloud_" + std::to_string(idx) +
                       ".png");

    tsdf_range_data_inserter.Insert(
        cartographer::sensor::TransformRangeData(sample.range_data,
                                                 pose_estimate.cast<float>()),
        &hybrid_grid_tsdf);
    positions_x.push_back(pose_estimate.translation()[0]);
    positions_y.push_back(pose_estimate.translation()[1]);
    positions_z.push_back(pose_estimate.translation()[2]);
    initial_pose_estimate = pose_estimate;
    idx++;
    }

    matplotlibcpp::plot();
    matplotlibcpp::named_plot("x", positions_x);
    matplotlibcpp::named_plot("y", positions_y);
    matplotlibcpp::named_plot("z", positions_z);
    matplotlibcpp::legend();
    matplotlibcpp::show();
}


void RunScanMatchingEvaluationDriftTest() {
  cartographer::mapping::proto::RangeDataInserterOptions3D
      range_data_inserter_options;
  auto parameter_dictionary_range_data_inserter = common::MakeDictionary(R"text(
        return {
      range_data_inserter_type = "PROBABILITY_GRID_INSERTER_3D",
      probability_grid_range_data_inserter = {
        hit_probability = 0.55,
        miss_probability = 0.49,
        num_free_space_voxels = 2,
      },
      tsdf_range_data_inserter = {
        relative_truncation_distance = 4,
        maximum_weight = 1000.,
        num_free_space_voxels = 0,
        project_sdf_distance_to_scan_normal = true,
        weight_function_epsilon = 1,
        weight_function_sigma = 4.,
        normal_estimate_max_nn = 20.,
        normal_estimate_radius = 0.3,
      },})text");
  range_data_inserter_options =
      cartographer::mapping::CreateRangeDataInserterOptions3D(
          parameter_dictionary_range_data_inserter.get());
  Sample sample = GenerateDefinedSample(0.0, 0.0, 0.0,
                                        ScanCloudGenerator::ModelType::CUBOID,
                                        {17.25, 1.25, 1.25}, 0.031, 0.002f);


  mapping::ValueConversionTables conversion_tables_;
  mapping::HybridGridTSDF hybrid_grid_tsdf(
      0.05,
      range_data_inserter_options.tsdf_range_data_inserter_options_3d()
          .relative_truncation_distance(),
      range_data_inserter_options.tsdf_range_data_inserter_options_3d()
          .maximum_weight(),
      &conversion_tables_);
  mapping::TSDFRangeDataInserter3D tsdf_range_data_inserter(
      range_data_inserter_options);
  tsdf_range_data_inserter.Insert(sample.range_data, &hybrid_grid_tsdf);

  cartographer::mapping::scan_matching::proto::CeresScanMatcherOptions3D
      scan_matcher_options;
  auto parameter_dictionary_scan_matcher = common::MakeDictionary(R"text(
    return {
      occupied_space_weight_0 = 1.,
      translation_weight = 0.,
      rotation_weight = 0.,
      only_optimize_yaw = false,
      ceres_solver_options = {
        use_nonmonotonic_steps = false,
        max_num_iterations = 120,
        num_threads = 1,
      },
    })text");
  scan_matcher_options =
      cartographer::mapping::scan_matching::CreateCeresScanMatcherOptions3D(
          parameter_dictionary_scan_matcher.get());
  mapping::scan_matching::CeresScanMatcher3D ceres_scan_matcher(
      scan_matcher_options);
  const Eigen::Vector3d target_translation({0.0, 0.0, 0.0});
  std::vector<mapping::scan_matching::PointCloudAndHybridGridPointers>
      point_clouds_and_hybrid_grids;

  sensor::RangefinderPoint point;
  point.position = {0.f, 0.f, 0.f};
  sensor::PointCloud single_point = {point};

  //  for (float x = -0.4f; x < 0.401f; x += 0.01) {
  //    const transform::Rigid3d initial_pose_estimate({x, 0.0, 0.0},
  //                                                   {1.0, 0.0, 0.0, 0.0});
  //    transform::Rigid3d pose_estimate = transform::Rigid3d::Identity();
  //    ceres::Solver::Summary summary;
  //    ceres_scan_matcher.Match(
  //        target_translation, initial_pose_estimate,
  //        {{&(sample.range_data.returns), &hybrid_grid_tsdf}}, &pose_estimate,
  //        &summary);
  //    LOG(INFO) << "x " << x << " \tmatch \t" << pose_estimate.DebugString();
  //  }

  GridDrawer grid_drawer;
  grid_drawer.DrawInterpolatedTSD(hybrid_grid_tsdf, 0.2);
  grid_drawer.DrawPointcloud(sample.range_data.returns,
                             transform::Rigid3d::Identity());
  grid_drawer.ToFile("interpolated_grid_with_cloud.png");
  //
  //  grid_drawer = GridDrawer();
  //  grid_drawer.DrawSinglePointCostFunction(hybrid_grid_tsdf,
  //  ceres_scan_matcher); grid_drawer.ToFile("cost.png");

  grid_drawer = GridDrawer();
  grid_drawer.DrawTSD(hybrid_grid_tsdf, 0.2);
  grid_drawer.ToFile("tsdf.png");
  LOG(INFO) << "wrote file";



  transform::Rigid3d initial_pose_estimate = transform::Rigid3d::Identity();
  transform::Rigid3d pose_estimate = transform::Rigid3d::Identity();
  int idx = 0;

  std::normal_distribution<float> normal_distribution(0,
                                                      0.05);  // 0.0
  std::vector<float> positions_x = {};
  std::vector<float> positions_y = {};
  std::vector<float> positions_z = {};

  for (double phi = -M_PI; phi < 5 * M_PI; phi += 0.1 * M_PI) {
    cartographer::transform::Rigid3d pertubation = cartographer::transform::Rigid3d({normal_distribution(e1),normal_distribution(e1),0}, {1, 0, 0, 0});
    initial_pose_estimate = pertubation * initial_pose_estimate;

    //
    //    Sample sample = GenerateDefinedSampleSlice(0, 0, 0.0,
    //                                               ScanCloudGenerator::ModelType::CUBOID,
    //                                               {17.25, 1.25, 1.25}, 0.031,
    //                                               0.002f, std::fmod(phi +
    //                                               M_PI, 2.0* M_PI) - M_PI,
    //                                               (1.0/6.0) * M_PI);

    Sample sample =
        GenerateDefinedSample(0, 0, 0.0, ScanCloudGenerator::ModelType::CUBOID,
                              {17.25, 1.25, 1.25}, 0.031, 0.002f);

    ceres::Solver::Summary summary;
    ceres_scan_matcher.Match(
        target_translation, initial_pose_estimate,
        {{&(sample.range_data.returns), &hybrid_grid_tsdf}}, &pose_estimate,
        &summary);
    LOG(INFO) << "phi " << phi << " \tmatch \t" << pose_estimate.DebugString();


    grid_drawer = GridDrawer();
    grid_drawer.DrawInterpolatedTSD(hybrid_grid_tsdf, 0.2);
    grid_drawer.DrawPointcloud(sample.range_data.returns,
                               pose_estimate);
    grid_drawer.DrawPose(initial_pose_estimate, 1, 0 ,0);
    grid_drawer.DrawPose(pose_estimate, 0, 1 ,0);
    grid_drawer.DrawPose(transform::Rigid3d::Identity(), 1, 1 ,1);
    grid_drawer.ToFile("interpolated_grid_with_cloud_"+std::to_string(idx)+".png");

    tsdf_range_data_inserter.Insert(cartographer::sensor::TransformRangeData(
        sample.range_data, pose_estimate.cast<float>()), &hybrid_grid_tsdf);
    positions_x.push_back(pose_estimate.translation()[0]);
    positions_y.push_back(pose_estimate.translation()[1]);
    positions_z.push_back(pose_estimate.translation()[2]);
    initial_pose_estimate = pose_estimate;
    idx++;
  }

  matplotlibcpp::plot();
  matplotlibcpp::named_plot("x", positions_x);
  matplotlibcpp::named_plot("y", positions_y);
  matplotlibcpp::named_plot("z", positions_z);
  matplotlibcpp::legend();
  matplotlibcpp::show();
}



}  // namespace evaluation
}  // namespace cartographer


int main(int argc, char** argv) {

  google::InitGoogleLogging(argv[0]);
  FLAGS_logtostderr = true;
  google::ParseCommandLineFlags(&argc, &argv, true);
  cartographer::evaluation::RunScanMatchingEvaluationRotatingScan();
}