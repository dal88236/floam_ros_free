#include "floam_ros_free/laser_mapper.h"
#include "floam_ros_free/util.h"

#include <chrono>
#include <thread>

namespace floam {

LaserMapper::LaserMapper() {}

void LaserMapper::run() {
  while (!stop_) {
    bool shall_update = false;
    pcl::PointCloud<pcl::PointXYZI>::Ptr pc_in;
    Eigen::Isometry3d pose_current;

    {
      std::lock_guard<std::mutex> lk(mtx_);
      if (!pointcloud_queue_.empty() && !pose_queue_.empty()) {
        pc_in = pointcloud_queue_.front();
        pose_current = pose_queue_.front();
        pointcloud_queue_.pop();
        pose_queue_.pop();
        shall_update = true;
      }
    }

    if (shall_update)
      updateCurrentPointsToMap(pc_in, pose_current);

    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
}

void LaserMapper::stop() {
  stop_ = true;
}

void LaserMapper::addPointsAndPose(const pcl::PointCloud<pcl::PointXYZI>::Ptr& pc_in, const Eigen::Isometry3d& pose_current) {
  std::lock_guard<std::mutex> lk(mtx_);
  pointcloud_queue_.push(pc_in);
  pose_queue_.push(pose_current);
}

void LaserMapper::init(double map_resolution) {
  // init map
  // init can have real object, but future added block does not need
  for (int i = 0; i < LASER_CELL_RANGE_HORIZONTAL * 2 + 1; i++) {
    std::vector<std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>>
        map_height_temp;
    for (int j = 0; j < LASER_CELL_RANGE_HORIZONTAL * 2 + 1; j++) {
      std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> map_depth_temp;
      for (int k = 0; k < LASER_CELL_RANGE_VERTICAL * 2 + 1; k++) {
        pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud_temp(
            new pcl::PointCloud<pcl::PointXYZI>);
        map_depth_temp.push_back(point_cloud_temp);
      }
      map_height_temp.push_back(map_depth_temp);
    }
    map_.push_back(map_height_temp);
  }

  origin_in_map_x_ = LASER_CELL_RANGE_HORIZONTAL;
  origin_in_map_y_ = LASER_CELL_RANGE_HORIZONTAL;
  origin_in_map_z_ = LASER_CELL_RANGE_VERTICAL;
  map_width_ = LASER_CELL_RANGE_HORIZONTAL * 2 + 1;
  map_height_ = LASER_CELL_RANGE_HORIZONTAL * 2 + 1;
  map_depth_ = LASER_CELL_RANGE_HORIZONTAL * 2 + 1;

  // downsampling size
  downsize_filter_.setLeafSize(map_resolution, map_resolution, map_resolution);
}

void LaserMapper::addWidthCellNegative(void) {
  std::vector<std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>>
      map_height_temp;
  for (int j = 0; j < map_height_; j++) {
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> map_depth_temp;
    for (int k = 0; k < map_depth_; k++) {
      pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud_temp;
      map_depth_temp.push_back(point_cloud_temp);
    }
    map_height_temp.push_back(map_depth_temp);
  }
  map_.insert(map_.begin(), map_height_temp);

  origin_in_map_x_++;
  map_width_++;
}

void LaserMapper::addWidthCellPositive(void) {
  std::vector<std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>>
      map_height_temp;
  for (int j = 0; j < map_height_; j++) {
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> map_depth_temp;
    for (int k = 0; k < map_depth_; k++) {
      pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud_temp;
      map_depth_temp.push_back(point_cloud_temp);
    }
    map_height_temp.push_back(map_depth_temp);
  }
  map_.push_back(map_height_temp);
  map_width_++;
}

void LaserMapper::addHeightCellNegative(void) {
  for (int i = 0; i < map_width_; i++) {
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> map_depth_temp;
    for (int k = 0; k < map_depth_; k++) {
      pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud_temp;
      map_depth_temp.push_back(point_cloud_temp);
    }
    map_[i].insert(map_[i].begin(), map_depth_temp);
  }
  origin_in_map_y_++;
  map_height_++;
}

void LaserMapper::addHeightCellPositive(void) {
  for (int i = 0; i < map_width_; i++) {
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> map_depth_temp;
    for (int k = 0; k < map_depth_; k++) {
      pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud_temp;
      map_depth_temp.push_back(point_cloud_temp);
    }
    map_[i].push_back(map_depth_temp);
  }
  map_height_++;
}

void LaserMapper::addDepthCellNegative(void) {
  for (int i = 0; i < map_width_; i++) {
    for (int j = 0; j < map_height_; j++) {
      pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud_temp;
      map_[i][j].insert(map_[i][j].begin(), point_cloud_temp);
    }
  }
  origin_in_map_z_++;
  map_depth_++;
}

void LaserMapper::addDepthCellPositive(void) {
  for (int i = 0; i < map_width_; i++) {
    for (int j = 0; j < map_height_; j++) {
      pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud_temp;
      map_[i][j].push_back(point_cloud_temp);
    }
  }
  map_depth_++;
}

// extend map is points exceed size
void LaserMapper::checkPoints(int& x, int& y, int& z) {
  while (x + LASER_CELL_RANGE_HORIZONTAL > map_width_ - 1) {
    addWidthCellPositive();
  }
  while (x - LASER_CELL_RANGE_HORIZONTAL < 0) {
    addWidthCellNegative();
    x++;
  }
  while (y + LASER_CELL_RANGE_HORIZONTAL > map_height_ - 1) {
    addHeightCellPositive();
  }
  while (y - LASER_CELL_RANGE_HORIZONTAL < 0) {
    addHeightCellNegative();
    y++;
  }
  while (z + LASER_CELL_RANGE_VERTICAL > map_depth_ - 1) {
    addDepthCellPositive();
  }
  while (z - LASER_CELL_RANGE_VERTICAL < 0) {
    addDepthCellNegative();
    z++;
  }

  // initialize
  // create object if area is null
  for (int i = x - LASER_CELL_RANGE_HORIZONTAL;
       i < x + LASER_CELL_RANGE_HORIZONTAL + 1; i++) {
    for (int j = y - LASER_CELL_RANGE_HORIZONTAL;
         j < y + LASER_CELL_RANGE_HORIZONTAL + 1; j++) {
      for (int k = z - LASER_CELL_RANGE_VERTICAL;
           k < z + LASER_CELL_RANGE_VERTICAL + 1; k++) {
        if (map_[i][j][k] == NULL) {
          pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud_temp(
              new pcl::PointCloud<pcl::PointXYZI>());
          map_[i][j][k] = point_cloud_temp;
        }
      }
    }
  }
}

// update points to map
void LaserMapper::updateCurrentPointsToMap(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& pc_in,
    const Eigen::Isometry3d& pose_current) {
  int current_pos_id_x =
      int(std::floor(pose_current.translation().x() / LASER_CELL_WIDTH + 0.5)) +
      origin_in_map_x_;
  int current_pos_id_y =
      int(std::floor(pose_current.translation().y() / LASER_CELL_HEIGHT +
                     0.5)) +
      origin_in_map_y_;
  int current_pos_id_z =
      int(std::floor(pose_current.translation().z() / LASER_CELL_DEPTH + 0.5)) +
      origin_in_map_z_;

  // check is submap is null
  checkPoints(current_pos_id_x, current_pos_id_y, current_pos_id_z);

  pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_pc(
      new pcl::PointCloud<pcl::PointXYZI>());
  transformPointCloud(*pc_in, *transformed_pc, pose_current.cast<float>());

  // save points
  for (int i = 0; i < static_cast<int>(transformed_pc->points.size()); i++) {
    pcl::PointXYZI point_temp = transformed_pc->points[i];
    // for visualization only
    point_temp.intensity =
        std::min(1.0, std::max(pc_in->points[i].z + 2.0, 0.0) / 5);
    int currentPointIdX =
        int(std::floor(point_temp.x / LASER_CELL_WIDTH + 0.5)) +
        origin_in_map_x_;
    int currentPointIdY =
        int(std::floor(point_temp.y / LASER_CELL_HEIGHT + 0.5)) +
        origin_in_map_y_;
    int currentPointIdZ =
        int(std::floor(point_temp.z / LASER_CELL_DEPTH + 0.5)) +
        origin_in_map_z_;

    map_[currentPointIdX][currentPointIdY][currentPointIdZ]->push_back(
        point_temp);
  }

  // filtering points
  for (int i = current_pos_id_x - LASER_CELL_RANGE_HORIZONTAL;
       i < current_pos_id_x + LASER_CELL_RANGE_HORIZONTAL + 1; i++) {
    for (int j = current_pos_id_y - LASER_CELL_RANGE_HORIZONTAL;
         j < current_pos_id_y + LASER_CELL_RANGE_HORIZONTAL + 1; j++) {
      for (int k = current_pos_id_z - LASER_CELL_RANGE_VERTICAL;
           k < current_pos_id_z + LASER_CELL_RANGE_VERTICAL + 1; k++) {
        downsize_filter_.setInputCloud(map_[i][j][k]);
        downsize_filter_.filter(*(map_[i][j][k]));
      }
    }
  }

  if (update_map_cb_) {
    pcl::PointCloud<pcl::PointXYZI> laser_cloud_map;
    for (int i = 0; i < map_width_; i++){
      for (int j = 0; j < map_height_; j++){
        for (int k = 0; k < map_depth_; k++){
          if(map_[i][j][k]!=NULL){
            laser_cloud_map += *(map_[i][j][k]);
          }
        }
      }
    }
    update_map_cb_(laser_cloud_map);
  }
}

pcl::PointCloud<pcl::PointXYZI>::Ptr LaserMapper::getMap(void) {
  pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudMap =
      pcl::PointCloud<pcl::PointXYZI>::Ptr(
          new pcl::PointCloud<pcl::PointXYZI>());
  for (int i = 0; i < map_width_; i++) {
    for (int j = 0; j < map_height_; j++) {
      for (int k = 0; k < map_depth_; k++) {
        if (map_[i][j][k] != NULL) {
          *laserCloudMap += *(map_[i][j][k]);
        }
      }
    }
  }
  return laserCloudMap;
}

}  // namespace floam