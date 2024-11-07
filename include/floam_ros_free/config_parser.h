#ifndef CONFIG_PARSER_H_
#define CONFIG_PARSER_H_

#include "floam_ros_free/lidar.h"

#include <string>

namespace floam {

class ConfigParser {
 public:
  ConfigParser() = default;

  bool parseConfig(const std::string& file_path, lidar::Lidar& lidar_params, double& map_resolution);
};

} // namespace floam

#endif // CONFIG_PARSER_H_