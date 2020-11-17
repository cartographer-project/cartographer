#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

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
#include "gflags/gflags.h"
#include "glog/logging.h"

#ifdef WITH_OPEN3D
#include "Open3D/Open3D.h"
#endif


#include <sys/resource.h>

#ifdef WITH_PCL
#include "pcl/io/pcd_io.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"

DEFINE_string(pointcloud_filename, "", "Point cloud file name.");

namespace cartographer {
namespace mapping {
namespace {

double GetUsedCPUTime() {
  timespec cpu_timespec = {};
  clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &cpu_timespec);
  //  LOG(INFO) << "Elapsed CPU time: "
  //            << (cpu_timespec.tv_sec + 1e-9 * cpu_timespec.tv_nsec) << " s";
  rusage usage;
  //  CHECK_EQ(getrusage(RUSAGE_SELF, &usage), 0) << strerror(errno);
  //  LOG(INFO) << "Peak memory usage: " << usage.ru_maxrss << " KiB";
  return cpu_timespec.tv_sec + 1e-9 * cpu_timespec.tv_nsec;
}

struct PointXYZIR {
  PCL_ADD_POINT4D;
  float intensity;
  uint16_t ring;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;


void TSDFToPointcloud(::cartographer::mapping::HybridGridTSDF* tsdf) {

  pcl::PointCloud<pcl::PointXYZI>::Ptr map_cloud(
      new pcl::PointCloud<pcl::PointXYZI>);
  std::vector<Eigen::Array4f> cells;
  for (auto it = ::cartographer::mapping::HybridGridTSDF::Iterator(*tsdf);
       !it.Done(); it.Next()) {
    const ::cartographer::mapping::TSDFVoxel voxel = it.GetValue();
    const float tsd = tsdf->ValueConverter().ValueToTSD(voxel.discrete_tsd);
    const float weight =
        tsdf->ValueConverter().ValueToWeight(voxel.discrete_weight);
    const Eigen::Vector3f cell_center_submap =
        tsdf->GetCenterOfCell(it.GetCellIndex());
    const Eigen::Vector3f cell_center_global = cell_center_submap;
    //    if ((std::abs(cell_center_submap.z() - 1.0f) < 0.04) ||
    //        (std::abs(cell_center_submap.x()) < 0.04) ||
    //        (std::abs(cell_center_submap.y()) < 0.04))
    {
      Eigen::Vector3f transformed_cell = cell_center_submap;
      pcl::PointXYZI point;
      point.x = transformed_cell[0];
      point.y = transformed_cell[1];
      point.z = transformed_cell[2];
      point.intensity = tsd;
      map_cloud->push_back(point);
    }
  }
  static int i_cloud = 0;
  pcl::io::savePCDFileASCII("tsdf" + std::to_string(i_cloud) + ".pcd",
                            *map_cloud);
  i_cloud++;

  LOG(INFO)<< "TSDFToPointcloud requires PCL";
}

void Run(const std::string& pointcloud_filename) {

  pcl::PointCloud<PointXYZIR>::Ptr laserCloudIn(
      new pcl::PointCloud<PointXYZIR>);

  if (pcl::io::loadPCDFile<PointXYZIR>(pointcloud_filename, *laserCloudIn) ==
      -1) {
    PCL_ERROR("Couldn't read file test_pcd.pcd \n");
  }
  std::cout << "Loaded " << laserCloudIn->width * laserCloudIn->height
            << std::endl;

  double t_before_sorting = GetUsedCPUTime();

  sensor::RangeData full_cloud;
  {
    int NUM_ROWS = 16;
    int NUM_POINTS_PER_LINE = 1800;
    full_cloud.returns.resize(NUM_ROWS * NUM_POINTS_PER_LINE,
                              {{0.f, 0.f, 0.f}});

    int cloudSize = laserCloudIn->points.size();

    for (int i = 0; i < cloudSize; ++i) {
      PointXYZIR thisPoint;
      thisPoint.x = laserCloudIn->points[i].x;
      thisPoint.y = laserCloudIn->points[i].y;
      thisPoint.z = laserCloudIn->points[i].z;
      thisPoint.intensity = laserCloudIn->points[i].intensity;

      int rowIdn = laserCloudIn->points[i].ring;
      if (rowIdn < 0 || rowIdn >= NUM_ROWS) continue;

      float horizonAngle = std::atan2(thisPoint.x, thisPoint.y) * 180.0 / M_PI;

      static float ang_res_x = 360.f / float(NUM_POINTS_PER_LINE);
      int columnIdn =
          -round((horizonAngle - 90.0) / ang_res_x) + NUM_POINTS_PER_LINE / 2;
      if (columnIdn >= NUM_POINTS_PER_LINE) columnIdn -= NUM_POINTS_PER_LINE;

      if (columnIdn < 0 || columnIdn >= NUM_POINTS_PER_LINE) continue;

      //      float range = thisPoint.x*thisPoint.x +  thisPoint.y*thisPoint.y +
      //      thisPoint.z*thisPoint.z;

      //      if (range < 0.5)
      //        continue;

      int index = columnIdn + rowIdn * NUM_POINTS_PER_LINE;
      full_cloud.returns[index] = {{thisPoint.x, thisPoint.y, thisPoint.z}};
    }

    //    LOG(INFO)<<point_cloud.points.size();
    //    std::vector<Eigen::Vector3i> triangles;
    //
    //    for(int triangle_idx=0; triangle_idx< NUM_POINTS_PER_LINE * (NUM_ROWS
    //    - 1) -1 ; ++triangle_idx) {
    //      int i0 = triangle_idx;
    //      int i1 = triangle_idx + 1;
    //      int i2 = triangle_idx + NUM_POINTS_PER_LINE;
    //      int i3 = triangle_idx + NUM_POINTS_PER_LINE+1;
    //      Eigen::Vector3f p0 = {fullCloud.points[i0].x,
    //      fullCloud.points[i0].y, fullCloud.points[i0].z}; Eigen::Vector3f p1
    //      = {fullCloud.points[i1].x, fullCloud.points[i1].y,
    //      fullCloud.points[i1].z}; Eigen::Vector3f p2 =
    //      {fullCloud.points[i2].x, fullCloud.points[i2].y,
    //      fullCloud.points[i2].z}; Eigen::Vector3f p3 =
    //      {fullCloud.points[i3].x, fullCloud.points[i3].y,
    //      fullCloud.points[i3].z}; float r0 = p0.norm(); float r1 = p1.norm();
    //      float r2 = p2.norm();
    //      float r3 = p3.norm();
    //      float max_range_delta = 1.f;
    //      if(std::abs(r0-r1) < max_range_delta && std::abs(r0-r2) <
    //      max_range_delta && std::abs(r1-r2) < max_range_delta) {
    //        triangles.emplace_back(i0, i2, i1);
    //      }
    //      if(std::abs(r3-r1) < max_range_delta && std::abs(r3-r2) <
    //      max_range_delta && std::abs(r1-r2) < max_range_delta) {
    //        triangles.emplace_back(i1, i2, i3);
    //      }
    //    }
    //
    //    static int idx = 0;
    //    std::ofstream myfile ("example" + std::to_string(idx) +".ply");
    //    ++idx;
    //    myfile << "ply\n";
    //    myfile << "format ascii 1.0\n";
    //    myfile << "comment Created by Cartographer \n";
    //    myfile << "element vertex " << fullCloud.points.size() <<"\n";
    //    myfile << "property float x \n";
    //    myfile << "property float y \n";
    //    myfile << "property float z \n";
    //    myfile << "element face " << triangles.size() <<"\n";
    //    myfile << "property list uchar uint vertex_indices \n";
    //    myfile << "end_header \n";
    //    for(auto& p : fullCloud.points) {
    //      myfile << p.x << " " << p.y << " " << p.z       << "\n";
    //    }
    //    for(auto& t : triangles) {
    //      myfile << 3 << " " <<  t[0] << " " << t[1] << " " << t[2] << "\n";
    //
    //    }
    //    myfile.close();
  }

  double t_after_sorting = GetUsedCPUTime();
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
        normal_estimate_max_nn = 50.,
        normal_estimate_radius = 0.3,
        normal_computation_method = "TRIANGLE_FILL_IN",
      },})text");
  //  CLOUD_STRUCTURE
  // OPEN3D
  // TRIANGLE_FILL_IN
  range_data_inserter_options =
      cartographer::mapping::CreateRangeDataInserterOptions3D(
          parameter_dictionary_range_data_inserter.get());

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

  double t_before_insert = GetUsedCPUTime();
  tsdf_range_data_inserter.Insert(full_cloud, &hybrid_grid_tsdf);
  double t_after_insert = GetUsedCPUTime();
  LOG(INFO) << "sorting took " << t_after_sorting - t_before_sorting;
  LOG(INFO) << "insert took " << t_after_insert - t_before_insert;

  //  evaluation::GridDrawer grid_drawer = evaluation::GridDrawer();
  //  grid_drawer.DrawTSD(hybrid_grid_tsdf, 0.3);
  //  grid_drawer.DrawPointcloud(full_cloud.returns,
  //                             transform::Rigid3d::Identity());
  //  grid_drawer.ToFile("tsdf.png");
  //
  TSDFToPointcloud(&hybrid_grid_tsdf);
  LOG(INFO) << " wrote file ";
}

}  // namespace
}  // namespace mapping
}  // namespace cartographer

POINT_CLOUD_REGISTER_POINT_STRUCT(
    cartographer::mapping::PointXYZIR,
    (float, x, x)(float, y, y)(float, z, z)(float, intensity,
                                            intensity)(uint16_t, ring, ring))

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  FLAGS_logtostderr = true;
  google::SetUsageMessage(
      "\n\n"
      "This program computes maps.");
  google::ParseCommandLineFlags(&argc, &argv, true);

  if (FLAGS_pointcloud_filename.empty()) {
    google::ShowUsageWithFlagsRestrict(argv[0], "pointcloud_filename");
    return EXIT_FAILURE;
  }

  ::cartographer::mapping::Run(FLAGS_pointcloud_filename);
}

#endif