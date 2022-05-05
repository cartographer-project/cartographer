#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <fstream>  // std::ifstream
#include <string>
#include <boost/filesystem.hpp>
#include <stdio.h>
#include <ctime>
#include "glog/logging.h"

#include "viam/src/io/read_PCD_file.h"


namespace viam {
namespace io {

namespace fs = boost::filesystem;

cartographer::sensor::TimedPointCloudData ReadFile::timedPointCloudDataFromPCDBuilder (std::string file_path, std::string initial_filename){

  pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);

  cartographer::sensor::TimedPointCloudData timedPCD;
  cartographer::sensor::TimedPointCloud ranges;
  
  //Open the point cloud file
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
  auto err = pcl::io::loadPCDFile<pcl::PointXYZRGB> (file_path, *cloud);

  if (err == -1) {
    return timedPCD;
  }

  // KAT NOTE: The file name format for the pcd files is assumed to be, e.g.:
  // rplidar_data_2022-02-05T01_00_20.9874.pcd
  int start_pos = initial_filename.find("T") + 1;
  int len_pos = initial_filename.find(".pcd") - initial_filename.find("T") - 1;
  std::string initial_file = initial_filename.substr(start_pos, len_pos);
  std::string next_file = file_path.substr(start_pos, len_pos);

  std::string::size_type sz;

  // Hour
  float hour_f = std::stof(next_file.substr(0,2), &sz);
  float hour_i = std::stof(initial_file.substr(0,2), &sz);

  // Minute
  float min_f = std::stof(next_file.substr(3,2), &sz);
  float min_i = std::stof(initial_file.substr(3,2), &sz);

  // Second
  float sec_f = std::stof(next_file.substr(6), &sz);
  float sec_i = std::stof(initial_file.substr(6), &sz);

  float time_delta = 3600*(hour_f-hour_i) + 60*(min_f-min_i) + (sec_f - sec_i);

  LOG(INFO) << "------------ FILE DATA -------------\n";
  LOG(INFO) << "Accessing file " << file_path << " ... "; 
  LOG(INFO) << "Loaded " << cloud->width * cloud->height << " data points \n";
  LOG(INFO) << "Size " << cloud->points.size() << "\n";
  LOG(INFO) << "TD " << time_delta << "\n";
  LOG(INFO) << "------------------------------------\n";

  for (size_t i = 0; i < cloud->points.size(); ++i) {

    cartographer::sensor::TimedRangefinderPoint TimedRP;
    TimedRP.position = Eigen::Vector3f(cloud->points[i].x, cloud->points[i].y, cloud->points[i].z);
    TimedRP.time = 0 - i*0.0001;
    
    ranges.push_back(TimedRP);
  }

  timedPCD.time = cartographer::common::FromUniversal(123) + cartographer::common::FromSeconds(double(time_delta));
  timedPCD.origin = Eigen::Vector3f::Zero();
  timedPCD.ranges = ranges;

  return timedPCD;
}

std::vector<std::string> ReadFile::listFilesInDirectory(std::string data_directory)
{
    std::vector<std::string> file_paths;

    for (const auto & entry : fs::directory_iterator(data_directory)) {
        file_paths.push_back((entry.path()).string());
    }

    sort(file_paths.begin(), file_paths.end());
    return file_paths;
}

int ReadFile::removeFile (std::string file_path)
{
  if( remove(file_path.c_str()) != 0 ) {
    
    LOG(INFO) << "Error removing file"; 
    return 0;
  }
  return 1;
}

}  // namespace io
}  // namespace viam
