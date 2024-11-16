#include "floam_ros_free/system.h"

#include <pcl/io/pcd_io.h>

#include <iostream>
#include <chrono>
#include <ctime>
#include <iomanip>
#include <sstream>

namespace floam {

System::System(const std::string& config_file_path) {
  laser_mapper_ = new LaserMapper();
  odom_estimator_ = new OdomEstimator(laser_mapper_);
  laser_processor_ = new LaserProcessor(odom_estimator_);
  viewer_ = new Viewer(std::bind(&System::saveMap, this));

  lidar::Lidar lidar_param;
  double map_resolution;
  config_parser_.parseConfig(config_file_path, lidar_param, map_resolution);
  laser_mapper_->init(map_resolution);
  laser_processor_->init(lidar_param);
  odom_estimator_->init(map_resolution);

  laser_mapper_->setUpdateMapCallback(std::bind(&Viewer::updateMap, viewer_, std::placeholders::_1));
  odom_estimator_->setUpdateOdomCallback(std::bind(&Viewer::addOdom, viewer_, std::placeholders::_1, std::placeholders::_2));

  laser_mapping_thd_ = new std::thread(&LaserMapper::run, laser_mapper_);
  odom_estimation_thd_ = new std::thread(&OdomEstimator::run, odom_estimator_);
  viewer_render_thd_ = new std::thread(&Viewer::run, viewer_);
}

System::System(const std::string& config_file_path, const LaserMapper::UpdateMapCallback& update_map_cb, const OdomEstimator::UpdateOdomCallback& update_odom_cb) {
  laser_mapper_ = new LaserMapper();
  odom_estimator_ = new OdomEstimator(laser_mapper_);
  laser_processor_ = new LaserProcessor(odom_estimator_);

  lidar::Lidar lidar_param;
  double map_resolution;
  config_parser_.parseConfig(config_file_path, lidar_param, map_resolution);
  laser_mapper_->init(map_resolution);
  laser_processor_->init(lidar_param);
  odom_estimator_->init(map_resolution);

  laser_mapper_->setUpdateMapCallback(update_map_cb);
  odom_estimator_->setUpdateOdomCallback(update_odom_cb);

  laser_mapping_thd_ = new std::thread(&LaserMapper::run, laser_mapper_);
  odom_estimation_thd_ = new std::thread(&OdomEstimator::run, odom_estimator_);
}

System::~System() {
  laser_mapper_->stop();
  odom_estimator_->stop();

  if (laser_mapping_thd_->joinable())
    laser_mapping_thd_->join();
  if (odom_estimation_thd_->joinable())
    odom_estimation_thd_->join();

  delete odom_estimator_;
  delete laser_processor_;
  delete laser_mapper_;

  if (viewer_) {
    viewer_->requestStop();
    if (viewer_render_thd_->joinable())
      viewer_render_thd_->join();
    delete viewer_;
  }
}

void System::track(const pcl::PointCloud<pcl::PointXYZI>::Ptr& pc_in) {
  laser_processor_->featureExtraction(pc_in);
}

void System::track(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& pc_in) {
  pcl::PointCloud<pcl::PointXYZI>::Ptr pc_in_ptr(new pcl::PointCloud<pcl::PointXYZI>(*pc_in));
  laser_processor_->featureExtraction(pc_in_ptr);
}

void System::saveMap() {
  pcl::PointCloud<pcl::PointXYZI>::Ptr map_pointcloud = laser_mapper_->getMap();

  auto now = std::chrono::system_clock::now();
  std::time_t current_time = std::chrono::system_clock::to_time_t(now);

  std::tm local_time;
  localtime_r(&current_time, &local_time);

  // Format the date and time as a string
  std::ostringstream data_time_stream;
  data_time_stream << std::put_time(&local_time, "%Y%m%d_%H%M%S");
  std::string map_filename = data_time_stream.str() + ".pcd";
  pcl::io::savePCDFileASCII(map_filename, *map_pointcloud);

  std::cout << map_filename << " has been saved to disk" << std::endl;
}

} // namespace floam