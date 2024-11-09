#ifndef SYSTEM_H_
#define SYSTEM_H_

#include "floam_ros_free/laser_mapper.h"
#include "floam_ros_free/odom_estimator.h"
#include "floam_ros_free/laser_processor.h"
#include "floam_ros_free/config_parser.h"

#include <string>

namespace floam {

class System {
 public:
  System(const std::string& config_file_path);
  System(const std::string& config_file_path, const LaserMapper::UpdateMapCallback& update_map_cb, const OdomEstimator::UpdateOdomCallback& update_odom_cb);
  ~System();

  System(const System&) = delete;
  System(System&&) = delete;

  void operator=(const System&) = delete;

  void track(const pcl::PointCloud<pcl::PointXYZI>::Ptr& pc_in);
  void track(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& pc_in);

 private:
  LaserProcessor* laser_processor_;
  LaserMapper* laser_mapper_;
  OdomEstimator* odom_estimator_;
  ConfigParser config_parser_;
  
  std::thread* laser_mapping_thd_;
  std::thread* odom_estimation_thd_;
};

} // namespace floam

#endif // SYSTEM_H_