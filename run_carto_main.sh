#!/bin/sh
set -o errexit
# set -o verbose


# ---- Edit based on your needs:
DATE="May03"

MAPPING="true"
LOCALIZATION="true"
UPDATE="true"

DATA_BASE_DIRECTORY="$HOME/viam/lidar_data"
MAPPING_DATA_DIRECTORY="$DATA_BASE_DIRECTORY/data_Feb_11_2022_small"
LOCALIZATION_DATA_DIRECTORY="$DATA_BASE_DIRECTORY/data_Feb_24_2022_printer_room"
UPDATE_DATA_DIRECTORY="$DATA_BASE_DIRECTORY/data_Feb_24_2022_printer_room"

DESCRIPTION="_refactor"
OUTPUT_DIRECTORY="pics${DESCRIPTION}_${DATE}"
MAP_OUTPUT_NAME="map${DESCRIPTION}_${DATE}.pbstream"

PICTURE_PRINT_INTERVAL="50"

MAPPING_STARTING_SCAN_NUMBER="0"
LOCALIZATION_STARTING_SCAN_NUMBER="0"
UPDATE_STARTING_SCAN_NUMBER="0"
# ----

CONFIGURATION_DIRECTORY="../lua_files"
CONFIGURATION_MAPPING_BASENAME="mapping_new_map.lua"
CONFIGURATION_LOCALIZATION_BASENAME="locating_in_map.lua"
CONFIGURATION_UPDATE_BASENAME="updating_a_map.lua"

mkdir -p viam/output
cd viam/output
rm -rf ${OUTPUT_DIRECTORY}
mkdir ${OUTPUT_DIRECTORY}
rm -rf pics_localization_map_visualization
mkdir pics_localization_map_visualization
rm -rf pics_update_map_visualization
mkdir pics_update_map_visualization

../build/main  \
    -configuration_directory=${CONFIGURATION_DIRECTORY}  \
    -configuration_mapping_basename=${CONFIGURATION_MAPPING_BASENAME}  \
    -configuration_localization_basename=${CONFIGURATION_LOCALIZATION_BASENAME}  \
    -configuration_update_basename=${CONFIGURATION_UPDATE_BASENAME}  \
    -mapping=${MAPPING}  \
    -localization=${LOCALIZATION}  \
    -update=${UPDATE}  \
    -mapping_data_directory="${MAPPING_DATA_DIRECTORY}" \
    -localization_data_directory="${LOCALIZATION_DATA_DIRECTORY}" \
    -update_data_directory="${UPDATE_DATA_DIRECTORY}" \
    -output_directory=${OUTPUT_DIRECTORY} \
    -map_output_name=${MAP_OUTPUT_NAME} \
    -picture_print_interval=${PICTURE_PRINT_INTERVAL} \
    -mapping_starting_scan_number=${MAPPING_STARTING_SCAN_NUMBER} \
    -localization_starting_scan_number=${LOCALIZATION_STARTING_SCAN_NUMBER} \
    -update_starting_scan_number=${UPDATE_STARTING_SCAN_NUMBER}
