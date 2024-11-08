#ifndef LASER_MAPPER_H_
#define LASER_MAPPER_H_

#include "floam_ros_free/lidar.h"

// PCL lib
#include <pcl/filters/filter.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// eigen  lib
#include <Eigen/Dense>
#include <Eigen/Geometry>

// c++ lib
#include <math.h>

#include <functional>
#include <mutex>
#include <queue>
#include <string>
#include <vector>

#define LASER_CELL_WIDTH 50.0
#define LASER_CELL_HEIGHT 50.0
#define LASER_CELL_DEPTH 50.0

// separate map as many sub point clouds

#define LASER_CELL_RANGE_HORIZONTAL 2
#define LASER_CELL_RANGE_VERTICAL 2

namespace floam {

class LaserMapper {
 public:
  typedef std::function<void(pcl::PointCloud<pcl::PointXYZI>&)>
      UpdateMapCallback;

  LaserMapper();

  void run();
  void stop();
  void init(double map_resolution);
  void addPointsAndPose(const pcl::PointCloud<pcl::PointXYZI>::Ptr& pc_in,
                        const Eigen::Isometry3d& pose_current);
  pcl::PointCloud<pcl::PointXYZI>::Ptr getMap(void);
  void setUpdateMapCallback(UpdateMapCallback update_map_cb) {
    update_map_cb_ = update_map_cb;
  }

 private:
  void updateCurrentPointsToMap(
      const pcl::PointCloud<pcl::PointXYZI>::Ptr& pc_in,
      const Eigen::Isometry3d& pose_current);
  void addWidthCellNegative(void);
  void addWidthCellPositive(void);
  void addHeightCellNegative(void);
  void addHeightCellPositive(void);
  void addDepthCellNegative(void);
  void addDepthCellPositive(void);
  void checkPoints(int& x, int& y, int& z);

  int origin_in_map_x_;
  int origin_in_map_y_;
  int origin_in_map_z_;
  int map_width_;
  int map_height_;
  int map_depth_;
  std::vector<std::vector<std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>>>
      map_;
  pcl::VoxelGrid<pcl::PointXYZI> downsize_filter_;

  std::queue<pcl::PointCloud<pcl::PointXYZI>::Ptr> pointcloud_queue_;
  std::queue<Eigen::Isometry3d> pose_queue_;

  bool stop_{false};

  UpdateMapCallback update_map_cb_;

  std::mutex mtx_;
};

}  // namespace floam

#endif  // LASER_MAPPER_H_