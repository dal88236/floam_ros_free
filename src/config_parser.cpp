#include "floam_ros_free/config_parser.h"

#include <yaml-cpp/yaml.h>
#include <iostream>

namespace floam {

bool ConfigParser::parseConfig(const std::string& file_path, lidar::Lidar& lidar_params, double& map_resolution) {
  try {
    YAML::Node config = YAML::LoadFile(file_path);
    lidar_params.setMaxDistance(config["max_dis"].as<double>());
    lidar_params.setMinDistance(config["min_dis"].as<double>());
    lidar_params.setVerticalAngle(config["vertical_angle"].as<double>());
    lidar_params.setLines(config["scan_line"].as<int>());
    lidar_params.setScanPeriod(config["scan_period"].as<double>());
    map_resolution = config["map_resolution"].as<double>();

    return true;
  }
  catch(const YAML::BadFile& e) {
    std::cerr << e.msg << std::endl;
    return false;
  }
  catch(const YAML::ParserException& e) {
    std::cerr << e.msg << std::endl;
    return false;
  }
  
}

} // namespace floam