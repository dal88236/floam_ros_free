#include "floam_ros_free/system.h"

namespace floam {

System::System(const std::string& config_file_path) {
  laser_mapper_ = new LaserMapper();
  odom_estimator_ = new OdomEstimator(laser_mapper_);
  laser_processor_ = new LaserProcessor(odom_estimator_);

  lidar::Lidar lidar_param;
  double map_resolution;
  config_parser_.parseConfig(config_file_path, lidar_param, map_resolution);
  laser_mapper_->init(map_resolution);
  laser_processor_->init(lidar_param);
  odom_estimator_->init(map_resolution);

  laser_mapping_thd_ = new std::thread(&LaserMapper::run, laser_mapper_);
  odom_estimation_thd_ = new std::thread(&OdomEstimator::run, odom_estimator_);
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
}

void System::track(const pcl::PointCloud<pcl::PointXYZI>::Ptr& pc_in) {
  laser_processor_->featureExtraction(pc_in);
}

void System::track(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& pc_in) {
  pcl::PointCloud<pcl::PointXYZI>::Ptr pc_in_ptr(new pcl::PointCloud<pcl::PointXYZI>(*pc_in));
  laser_processor_->featureExtraction(pc_in_ptr);
}

} // namespace floam