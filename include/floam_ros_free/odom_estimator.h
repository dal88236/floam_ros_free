#ifndef ODOM_ESTIMATOR_H_
#define ODOM_ESTIMATOR_H_

// std lib
#include <math.h>
#include <thread>
#include <string>
#include <vector>
#include <functional>
#include <queue>
#include <mutex>

// PCL
#include <pcl/filters/crop_box.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// eigen
#include <Eigen/Dense>
#include <Eigen/Geometry>

// g2o
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/linear_solver.h>
#include <g2o/solvers/dense/linear_solver_dense.h>

// LOCAL LIB
#include <ros/ros.h>

#include "floam_ros_free/g2o_type.h"

namespace floam {

class LaserMapper;

class OdomEstimator {
 public:
  typedef std::function<void(const Eigen::Quaterniond&, const Eigen::Vector3d&)> UpdateOdomCallback;
  typedef g2o::BlockSolver<g2o::BlockSolverTraits<6, 1>> BlockSolverType;
  typedef g2o::LinearSolverDense<BlockSolverType::PoseMatrixType> LinearSolverType;

  OdomEstimator(LaserMapper* laser_mapper);

  void run();
  void stop();
  void init(double map_resolution);
  void addFeaturePoints(const pcl::PointCloud<pcl::PointXYZI>::Ptr& edge_in,
                        const pcl::PointCloud<pcl::PointXYZI>::Ptr& surf_in);
  void initMapWithPoints(const pcl::PointCloud<pcl::PointXYZI>::Ptr& edge_in,
                         const pcl::PointCloud<pcl::PointXYZI>::Ptr& surf_in);
  void updatePointsToMap(const pcl::PointCloud<pcl::PointXYZI>::Ptr& edge_in,
                         const pcl::PointCloud<pcl::PointXYZI>::Ptr& surf_in);
  void getMap(pcl::PointCloud<pcl::PointXYZI>::Ptr& laserCloudMap);
  bool isInit() { return init_; }
  void setUpdateOdomCallback(UpdateOdomCallback update_odom_cb) { update_odom_cb_ = update_odom_cb; }

  Eigen::Isometry3d odom_;
  pcl::PointCloud<pcl::PointXYZI>::Ptr laser_cloud_corner_map_;
  pcl::PointCloud<pcl::PointXYZI>::Ptr laser_cloud_surf_map_;

 private:
  // function
  void addEdgeCostFactor(const pcl::PointCloud<pcl::PointXYZI>::Ptr& pc_in,
                         const pcl::PointCloud<pcl::PointXYZI>::Ptr& map_in,
                         g2o::SparseOptimizer&, g2o::VertexSE3Expmap*);
  void addSurfCostFactor(const pcl::PointCloud<pcl::PointXYZI>::Ptr& pc_in,
                         const pcl::PointCloud<pcl::PointXYZI>::Ptr& map_in,
                         g2o::SparseOptimizer&, g2o::VertexSE3Expmap*);
  void addPointsToMap(
      const pcl::PointCloud<pcl::PointXYZI>::Ptr& downsample_edge_cloud,
      const pcl::PointCloud<pcl::PointXYZI>::Ptr& downsample_surf_cloud);
  void pointAssociateToMap(pcl::PointXYZI const *const pi,
                           pcl::PointXYZI *const po);
  void downSamplingToMap(const pcl::PointCloud<pcl::PointXYZI>::Ptr& edge_pc_in,
                         pcl::PointCloud<pcl::PointXYZI>::Ptr& edge_pc_out,
                         const pcl::PointCloud<pcl::PointXYZI>::Ptr& surf_pc_in,
                         pcl::PointCloud<pcl::PointXYZI>::Ptr& surf_pc_out);

  // optimization variable
  double parameters[7] = {0, 0, 0, 1, 0, 0, 0};
  Eigen::Map<Eigen::Quaterniond> q_w_curr_ =
      Eigen::Map<Eigen::Quaterniond>(parameters);
  Eigen::Map<Eigen::Vector3d> t_w_curr_ =
      Eigen::Map<Eigen::Vector3d>(parameters + 4);

  Eigen::Isometry3d last_odom_;

  // kd-tree
  pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtree_edge_map_;
  pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtree_surf_map_;

  // points downsampling before add to map
  pcl::VoxelGrid<pcl::PointXYZI> downsize_filter_edge_;
  pcl::VoxelGrid<pcl::PointXYZI> downsize_filter_surf_;

  // local map
  pcl::CropBox<pcl::PointXYZI> crop_box_filter_;

  // optimization count
  int optimization_count_;

  bool init_{false};
  bool stop_{false};

  std::queue<pcl::PointCloud<pcl::PointXYZI>::Ptr> edge_queue_;
  std::queue<pcl::PointCloud<pcl::PointXYZI>::Ptr> surf_queue_;

  LaserMapper* laser_mapper_;

  UpdateOdomCallback update_odom_cb_;

  std::mutex mtx_;
};

}  // namespace floam

#endif  // ODOM_ESTIMATOR_H_