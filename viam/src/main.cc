#include "cartographer/mapping/map_builder.h"
#include "cartographer/metrics/register.h"
#include <string>

#include "cartographer/io/image.h"
#include "cartographer/io/file_writer.h"
#include "cartographer/io/submap_painter.h"
#include "glog/logging.h"

#include "cartographer/mapping/proto/trajectory.pb.h"
#include "cartographer/mapping/proto/pose_graph.pb.h"

#include "viam/src/mapping/map_builder.h"
#include "viam/src/io/draw_trajectories.h"
#include "viam/src/io/read_PCD_file.h"
#include "viam/src/io/submap_painter.h"


DEFINE_string(configuration_directory, "",
              "First directory in which configuration files are searched, "
              "second is always the Cartographer installation to allow "
              "including files from there.");
DEFINE_string(configuration_mapping_basename, "",
              "Basename, i.e. not containing any directory prefix, of the "
              "mapping configuration file.");
DEFINE_string(configuration_localization_basename, "",
              "Basename, i.e. not containing any directory prefix, of the "
              "localization configuration file.");
DEFINE_string(configuration_update_basename, "",
              "Basename, i.e. not containing any directory prefix, of the "
              "update map configuration file.");
DEFINE_string(mapping_data_directory, "",
              "Directory in which rplidar data for mapping is expected.");
DEFINE_string(localization_data_directory, "",
              "Directory in which rplidar data for localization is expected.");
DEFINE_string(update_data_directory, "",
              "Directory in which rplidar data for updating an a-priori map is expected.");
DEFINE_string(output_directory, "",
              "Directory where map images are saved in.");
DEFINE_string(map_output_name, "",
              "Name of the file where we're saving the generated map.");
DEFINE_bool(mapping, false,
              "Boolean that indicates whether or not we want to create a map.");
DEFINE_bool(localization, false,
              "Boolean that indicates whether or not we want to perform localization.");
DEFINE_bool(update, false,
              "Boolean that indicates whether or not we want to update an a-priori map with new data.");
DEFINE_int64(mapping_starting_scan_number, 0,
              "Scan index at which we want to start using the scans for mapping.");
DEFINE_int64(localization_starting_scan_number, 0,
              "Scan index at which we want to start using the scans for localization.");
DEFINE_int64(update_starting_scan_number, 0,
              "Scan index at which we want to start using the scans for updating the map.");
DEFINE_int64(picture_print_interval, 1e8,
              "Frequency at which we want to print pictures while cartographer is running.");

namespace viam {

using SensorId = cartographer::mapping::TrajectoryBuilderInterface::SensorId;

const SensorId kRangeSensorId{SensorId::SensorType::RANGE, "range"};
const SensorId kIMUSensorId{SensorId::SensorType::IMU, "imu"};


void PaintMap(std::unique_ptr<cartographer::mapping::MapBuilderInterface> & map_builder_,
              std::string output_directory,
              std::string appendix) {
  const double kPixelSize = 0.01;
  auto submap_poses = map_builder_->pose_graph()->GetAllSubmapPoses();
  std::map<cartographer::mapping::SubmapId, ::cartographer::io::SubmapSlice> submap_slices;

  if (submap_poses.size() > 0) {
    for (const auto& submap_id_pose : submap_poses) {
        cartographer::mapping::proto::SubmapQuery::Response response_proto;
        const std::string error = map_builder_->SubmapToProto(submap_id_pose.id, &response_proto);

        auto submap_textures = absl::make_unique<::cartographer::io::SubmapTextures>();
        submap_textures->version = response_proto.submap_version();
        for (const auto& texture_proto : response_proto.textures()) {
          const std::string compressed_cells(texture_proto.cells().begin(),
                                            texture_proto.cells().end());
          submap_textures->textures.emplace_back(::cartographer::io::SubmapTexture{
              ::cartographer::io::UnpackTextureData(compressed_cells, texture_proto.width(),
                                                    texture_proto.height()),
              texture_proto.width(), texture_proto.height(), texture_proto.resolution(),
              cartographer::transform::ToRigid3(texture_proto.slice_pose())});
        }

        // Prepares SubmapSlice
        ::cartographer::io::SubmapSlice& submap_slice = submap_slices[submap_id_pose.id];
        const auto fetched_texture = submap_textures->textures.begin();
        submap_slice.pose = submap_id_pose.data.pose;
        submap_slice.width = fetched_texture->width;
        submap_slice.height = fetched_texture->height;
        submap_slice.slice_pose = fetched_texture->slice_pose;
        submap_slice.resolution = fetched_texture->resolution;
        submap_slice.cairo_data.clear();

        submap_slice.surface = ::cartographer::io::DrawTexture(
          fetched_texture->pixels.intensity, fetched_texture->pixels.alpha, fetched_texture->width,
          fetched_texture->height, &submap_slice.cairo_data);

        if (submap_id_pose.id.submap_index == 0 && submap_id_pose.id.trajectory_id == 0) {
          const auto trajectory_nodes = map_builder_->pose_graph()->GetTrajectoryNodes();
          submap_slice.surface = viam::io::DrawTrajectoryNodes(trajectory_nodes, submap_slice.resolution, submap_slice.slice_pose, 
                                              submap_slice.surface.get());
        }
      }

    cartographer::io::PaintSubmapSlicesResult painted_slices =
        viam::io::PaintSubmapSlices(submap_slices, kPixelSize);
    auto image = cartographer::io::Image(std::move(painted_slices.surface));
    auto file = cartographer::io::StreamFileWriter(output_directory + "/map_" + appendix + ".png");
    image.WritePng(&file);
  }
}

void CreateMap(const std::string& mode,
              const std::string& data_directory,
              const std::string& output_directory,
              const std::string& configuration_directory,
              const std::string& configuration_basename,
              const std::string& map_output_name,
              int starting_scan_number,
              int picture_print_interval) {

  mapping::MapBuilder mapBuilder;

  // Add configs
  mapBuilder.SetUp(configuration_directory, configuration_basename);

  // Build MapBuilder
  mapBuilder.BuildMapBuilder();

  // Build TrajectoryBuilder
  int trajectory_id = mapBuilder.map_builder_->AddTrajectoryBuilder(
      {kRangeSensorId}, mapBuilder.trajectory_builder_options_,
      mapBuilder.GetLocalSlamResultCallback());

  LOG(INFO) << "Trajectory ID: " << trajectory_id;

  cartographer::mapping::TrajectoryBuilderInterface* trajectory_builder = mapBuilder.map_builder_->GetTrajectoryBuilder(trajectory_id);

  viam::io::ReadFile read_file;
  std::vector<std::string> file_list = read_file.listFilesInDirectory(data_directory);
  std::string initial_file = file_list[0];

  if (starting_scan_number < 0 || starting_scan_number >= int(file_list.size())) {
    std::cerr << "starting_scan_number is out of bounds: " << starting_scan_number << std::endl;
    return;
  }

  std::cout << "Beginning to add data....\n";
  
  int end_scan_number = int(file_list.size());
  for (int i = starting_scan_number; i < end_scan_number; i++ ) {
    auto measurement = mapBuilder.GetDataFromFile(data_directory, initial_file, i);

    if (measurement.ranges.size() > 0) {
        trajectory_builder->AddSensorData(kRangeSensorId.id, measurement);
        int num_nodes = mapBuilder.map_builder_->pose_graph()->GetTrajectoryNodes().size();
        if ((num_nodes >= starting_scan_number && num_nodes < starting_scan_number + 3) ||
             num_nodes % picture_print_interval == 0) {
          // std::cout << "(i, num_nodes) = (" << i << ", " << num_nodes << ")" << std::endl;
          PaintMap(mapBuilder.map_builder_, output_directory, std::to_string(num_nodes));
        }
    }
  }

  // Save the map in a pbstream file
  const std::string map_file = "./" + map_output_name;
  mapBuilder.map_builder_->pose_graph()->RunFinalOptimization();
  mapBuilder.map_builder_->SerializeStateToFile(true, map_file);

  mapBuilder.map_builder_->FinishTrajectory(trajectory_id);
  mapBuilder.map_builder_->pose_graph()->RunFinalOptimization();
  PaintMap(mapBuilder.map_builder_, output_directory, "0");
  
  return;
}

void LoadMapAndRun(const std::string& mode,
        const std::string& data_directory,
        const std::string& output_directory,
        const std::string& configuration_directory,
        const std::string& configuration_basename,
        const std::string& map_output_name,
        int starting_scan_number,
        int picture_print_interval,
        const std::string& operation) {

  bool load_frozen_trajectory = true;
  if (operation.compare("update") != 0 && operation.compare("localization") != 0) {
    std::cerr << "Operation is not valid: " << operation << std::endl;
    return;
  } else if (operation == "update") {
    load_frozen_trajectory = false;
  }
  mapping::MapBuilder mapBuilder;

  // Add configs
  mapBuilder.SetUp(configuration_directory, configuration_basename);

  // Build MapBuilder
  mapBuilder.BuildMapBuilder();

  // ASSUMPTION: Loaded trajectory has trajectory_id == 0
  const std::string map_file = "./" + map_output_name;
  std::map<int, int> mapping_of_trajectory_ids = mapBuilder.map_builder_->LoadStateFromFile(map_file, load_frozen_trajectory);
  mapBuilder.map_builder_->pose_graph()->RunFinalOptimization();
  for(std::map<int, int>::const_iterator it = mapping_of_trajectory_ids.begin(); it != mapping_of_trajectory_ids.end(); ++it)
  {
      std::cout << "Trajectory ids mapping: " << it->first << " " << it->second << "\n";
  }

  // Build TrajectoryBuilder
  int trajectory_id = mapBuilder.map_builder_->AddTrajectoryBuilder(
      {kRangeSensorId}, mapBuilder.trajectory_builder_options_,
      mapBuilder.GetLocalSlamResultCallback());

  std::cout << "Trajectory ID: " << trajectory_id << "\n";

  cartographer::mapping::TrajectoryBuilderInterface* trajectory_builder = mapBuilder.map_builder_->GetTrajectoryBuilder(trajectory_id);

  io::ReadFile read_file;
  std::vector<std::string> file_list = read_file.listFilesInDirectory(data_directory);
  std::string initial_file = file_list[0];

  if (starting_scan_number < 0 || starting_scan_number >= int(file_list.size())) {
    std::cerr << "starting_scan_number is out of bounds: " << starting_scan_number << std::endl;
    return;
  }

  std::cout << "Beginning to add data....\n";
  PaintMap(mapBuilder.map_builder_, output_directory, "before_" + operation);
  
  int end_scan_number = int(file_list.size());
  for (int i = starting_scan_number; i < end_scan_number; i++ ) {
    auto measurement = mapBuilder.GetDataFromFile(data_directory, initial_file, i);

    if (measurement.ranges.size() > 0) {
        trajectory_builder->AddSensorData(kRangeSensorId.id, measurement);
        if ((i >= starting_scan_number && i < starting_scan_number + 3) || i % picture_print_interval == 0) {
          PaintMap(mapBuilder.map_builder_, output_directory, operation + "_" + std::to_string(1 + i++));
        }
    }
  }

  // saved map after localization is finished
  const std::string map_file_2 = "./after_" + operation + "_" + map_output_name;
  mapBuilder.map_builder_->pose_graph()->RunFinalOptimization();
  mapBuilder.map_builder_->SerializeStateToFile(true, map_file_2);

  mapBuilder.map_builder_->FinishTrajectory(0);
  mapBuilder.map_builder_->FinishTrajectory(trajectory_id);
  mapBuilder.map_builder_->pose_graph()->RunFinalOptimization();
  PaintMap(mapBuilder.map_builder_, output_directory, "after_" + operation + "_optimization");

  return;
}

void DrawSavedMap(const std::string& mode,
        const std::string& output_directory,
        const std::string& configuration_directory,
        const std::string& configuration_basename,
        const std::string& map_output_name,
        const std::string& operation) {

  mapping::MapBuilder mapBuilder;

  // Add configs
  mapBuilder.SetUp(configuration_directory, configuration_basename);

  // Build MapBuilder
  mapBuilder.BuildMapBuilder();
  const std::string map_file = "./after_" + operation + "_" + map_output_name;
  std::map<int, int> mapping_of_trajectory_ids = mapBuilder.map_builder_->LoadStateFromFile(map_file, true);
  mapBuilder.map_builder_->pose_graph()->RunFinalOptimization();
  for(std::map<int, int>::const_iterator it = mapping_of_trajectory_ids.begin(); it != mapping_of_trajectory_ids.end(); ++it)
  {
      std::cout << "Trajectory ids mapping: " << it->first << " " << it->second << "\n";
  }

  PaintMap(mapBuilder.map_builder_, output_directory, map_output_name + "_saved_map_after_" + operation);

  mapBuilder.map_builder_->FinishTrajectory(0);
  mapBuilder.map_builder_->pose_graph()->RunFinalOptimization();
  PaintMap(mapBuilder.map_builder_, output_directory, "optimized_" +  map_output_name + "_saved_map_after_" + operation);

  return;
}

}  // namespace viam

// Example of how to run this file: 
// .run_cart_main.sh
int main(int argc, char** argv) {
  google::InitGoogleLogging("XXX");
  std::string mode = "DON'T USE RIGHT NOW!!!!!!!";
  google::ParseCommandLineFlags(&argc, &argv, true);
  FLAGS_logtostderr = false;

  if (FLAGS_configuration_directory.empty()) {
    std::cout << "-configuration_directory is missing.\n";
    return EXIT_FAILURE;
  } else if (FLAGS_configuration_mapping_basename.empty()) {
    std::cout << "-configuration_mapping_basename is missing.\n";
    return EXIT_FAILURE;
  } else if (FLAGS_configuration_localization_basename.empty()) {
    std::cout << "-configuration_localization_basename is missing.\n";
    return EXIT_FAILURE;
  } else if (FLAGS_configuration_update_basename.empty()) {
    std::cout << "-configuration_update_basename is missing.\n";
    return EXIT_FAILURE;
  } else if (FLAGS_mapping_data_directory.empty()) {
    std::cout << "-mapping_data_directory is missing.\n";
    return EXIT_FAILURE;
  } else if (FLAGS_localization_data_directory.empty()) {
    std::cout << "-localization_data_directory is missing.\n";
    return EXIT_FAILURE;
  } else if (FLAGS_update_data_directory.empty()) {
    std::cout << "-update_data_directory is missing.\n";
    return EXIT_FAILURE;
  } else if (FLAGS_output_directory.empty()) {
    std::cout << "-output_directory is missing.\n";
    return EXIT_FAILURE;
  } else if (FLAGS_map_output_name.empty()) {
    std::cout << "-map_output_name is missing.\n";
    return EXIT_FAILURE;
  }

  google::SetCommandLineOption("GLOG_minloglevel", "2");

  if (FLAGS_mapping == true) {
    std::cout << "Mapping!" << std::endl;
    viam::CreateMap(mode,
      FLAGS_mapping_data_directory,
      FLAGS_output_directory,
      FLAGS_configuration_directory,
      FLAGS_configuration_mapping_basename,
      FLAGS_map_output_name,
      FLAGS_mapping_starting_scan_number,
      FLAGS_picture_print_interval);
  }

  if (FLAGS_localization == true) {
    // std::cout << "Creating a quick visualization of the localization map" << std::endl;
    // cartographer::mapping::CreateMap(mode,
    //   FLAGS_localization_data_directory,
    //   "pics_localization_map_visualization",
    //   FLAGS_configuration_directory,
    //   FLAGS_configuration_mapping_basename,
    //   "map_localization_unused.pbstream",
    //   FLAGS_localization_starting_scan_number,
    //   200);

    std::cout << "Localizing!" << std::endl;
    viam::LoadMapAndRun(mode,
      FLAGS_localization_data_directory,
      FLAGS_output_directory,
      FLAGS_configuration_directory,
      FLAGS_configuration_localization_basename,
      FLAGS_map_output_name,
      FLAGS_localization_starting_scan_number,
      FLAGS_picture_print_interval,
      "localization");

    std::cout << "Drawing saved map!" << std::endl;
    viam::DrawSavedMap(mode,
      FLAGS_output_directory,
      FLAGS_configuration_directory,
      FLAGS_configuration_localization_basename,
      FLAGS_map_output_name,
      "localization");
  }

  if (FLAGS_update == true) {
    // std::cout << "Creating a quick visualization of the updating map" << std::endl;
    // cartographer::mapping::CreateMap(mode,
    //   FLAGS_update_data_directory,
    //   "pics_update_map_visualization",
    //   FLAGS_configuration_directory,
    //   FLAGS_configuration_mapping_basename,
    //   "map_update_unused.pbstream",
    //   FLAGS_update_starting_scan_number,
    //   200);

    std::cout << "Updating Map!" << std::endl;
    viam::LoadMapAndRun(mode,
      FLAGS_update_data_directory,
      FLAGS_output_directory,
      FLAGS_configuration_directory,
      FLAGS_configuration_update_basename,
      FLAGS_map_output_name,
      FLAGS_update_starting_scan_number,
      FLAGS_picture_print_interval,
      "update");

    std::cout << "Drawing saved map!" << std::endl;
    viam::DrawSavedMap(mode,
      FLAGS_output_directory,
      FLAGS_configuration_directory,
      FLAGS_configuration_update_basename,
      FLAGS_map_output_name,
      "update");
  }

  if (FLAGS_localization == false && FLAGS_mapping == false && FLAGS_update == false) {
    std::cout << "Not doing anything, mapping, localization, & updating maps are turned off." << std::endl;
  }

  return 1;
}
