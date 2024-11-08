#ifndef LASER_PROCESSOR_H_
#define LASER_PROCESSOR_H_

#include "floam_ros_free/lidar.h"

#include <pcl/filters/crop_box.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace floam {

// points covariance class
class Double2d {
 public:
  int id;
  double value;
  Double2d(int id_in, double value_in);
};
// points info class
class PointsInfo {
 public:
  int layer;
  double time;
  PointsInfo(int layer_in, double time_in);
};

class OdomEstimator;

class LaserProcessor {
 public:
  LaserProcessor(OdomEstimator* odom_estimator);

  void init(lidar::Lidar lidar_param_in);
  void featureExtraction(const pcl::PointCloud<pcl::PointXYZI>::Ptr& pc_in);
  void featureExtractionFromSector(
      const pcl::PointCloud<pcl::PointXYZI>::Ptr& pc_in,
      std::vector<Double2d>& cloudCurvature,
      pcl::PointCloud<pcl::PointXYZI>::Ptr& pc_out_edge,
      pcl::PointCloud<pcl::PointXYZI>::Ptr& pc_out_surf);

 private:
  lidar::Lidar lidar_param_;
  OdomEstimator* odom_estimator_;
  bool stop_{false};
};  // class LaserProcessor

}  // namespace floam

#endif  // LASER_PROCESSOR_H_