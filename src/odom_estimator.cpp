#include "floam_ros_free/odom_estimator.h"
#include "floam_ros_free/laser_mapper.h"

#include <g2o/core/optimization_algorithm_gauss_newton.h>

#include <chrono>
#include <iostream>

namespace floam {

OdomEstimator::OdomEstimator(LaserMapper* laser_mapper) : laser_mapper_(laser_mapper) {}

void OdomEstimator::run() {
  while (!stop_) {
    bool shall_update = false;
    pcl::PointCloud<pcl::PointXYZI>::Ptr edge_in;
    pcl::PointCloud<pcl::PointXYZI>::Ptr surf_in;

    {
      std::lock_guard<std::mutex> lk(mtx_);
      if (!edge_queue_.empty() && !surf_queue_.empty()) {
        edge_in = edge_queue_.front();
        surf_in = surf_queue_.front();
        edge_queue_.pop();
        surf_queue_.pop();
        shall_update = true;
      }
    }

    if (shall_update) {
      if (!init_)
        initMapWithPoints(edge_in, surf_in);
      else
        updatePointsToMap(edge_in, surf_in);
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
}

void OdomEstimator::stop() {
  stop_ = true;
}

void OdomEstimator::init(double map_resolution) {
  // init local map
  laser_cloud_corner_map_ = pcl::PointCloud<pcl::PointXYZI>::Ptr(
      new pcl::PointCloud<pcl::PointXYZI>());
  laser_cloud_surf_map_ = pcl::PointCloud<pcl::PointXYZI>::Ptr(
      new pcl::PointCloud<pcl::PointXYZI>());

  // downsampling size
  downsize_filter_edge_.setLeafSize(map_resolution, map_resolution,
                                 map_resolution);
  downsize_filter_surf_.setLeafSize(map_resolution * 2, map_resolution * 2,
                                 map_resolution * 2);

  // kd-tree
  kdtree_edge_map_ = pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr(
      new pcl::KdTreeFLANN<pcl::PointXYZI>());
  kdtree_surf_map_ = pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr(
      new pcl::KdTreeFLANN<pcl::PointXYZI>());

  odom_ = Eigen::Isometry3d::Identity();
  last_odom_ = Eigen::Isometry3d::Identity();
  optimization_count_ = 2;
}

void OdomEstimator::addFeaturePoints(const pcl::PointCloud<pcl::PointXYZI>::Ptr& edge_in,
                                     const pcl::PointCloud<pcl::PointXYZI>::Ptr& surf_in) {
  std::lock_guard<std::mutex> lk(mtx_);
  edge_queue_.push(edge_in);
  surf_queue_.push(surf_in);
}

void OdomEstimator::initMapWithPoints(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& edge_in,
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& surf_in) {
  if (init_)
    return;

  *laser_cloud_corner_map_ += *edge_in;
  *laser_cloud_surf_map_ += *surf_in;
  optimization_count_ = 12;
  init_ = true;
}

void OdomEstimator::updatePointsToMap(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& edge_in,
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& surf_in) {
  if (optimization_count_ > 2) optimization_count_--;

  Eigen::Isometry3d odom_prediction = odom_ * (last_odom_.inverse() * odom_);
  last_odom_ = odom_;
  odom_ = odom_prediction;

  pcl::PointCloud<pcl::PointXYZI>::Ptr downsample_edge_cloud(
      new pcl::PointCloud<pcl::PointXYZI>());
  pcl::PointCloud<pcl::PointXYZI>::Ptr downsample_surf_cloud(
      new pcl::PointCloud<pcl::PointXYZI>());
  downSamplingToMap(edge_in, downsample_edge_cloud, surf_in,
                    downsample_surf_cloud);

  q_w_curr_ = Eigen::Quaterniond(odom_.rotation());
  t_w_curr_ = odom_.translation();

  if (laser_cloud_corner_map_->points.size() > 10 &&
      laser_cloud_surf_map_->points.size() > 50) {
    kdtree_edge_map_->setInputCloud(laser_cloud_corner_map_);
    kdtree_surf_map_->setInputCloud(laser_cloud_surf_map_);

    std::unique_ptr<LinearSolverType> linear_solver = std::make_unique<LinearSolverType>();
    std::unique_ptr<BlockSolverType> block_solver = std::make_unique<BlockSolverType>(std::move(linear_solver));
    g2o::OptimizationAlgorithmGaussNewton* solver = new g2o::OptimizationAlgorithmGaussNewton(std::move(block_solver));
    g2o::SparseOptimizer opt;
    opt.setAlgorithm(solver);
    opt.setVerbose(false);

    g2o::VertexSE3Expmap* v = new g2o::VertexSE3Expmap();
    v->setEstimate(g2o::SE3Quat(q_w_curr_, t_w_curr_));
    v->setId(0);
    opt.addVertex(v);

    //  add edge here
    addEdgeCostFactor(downsample_edge_cloud, laser_cloud_corner_map_, opt, v);
    addSurfCostFactor(downsample_surf_cloud, laser_cloud_surf_map_, opt, v);

    for (int iterCount = 0; iterCount < optimization_count_; iterCount++) {
      opt.initializeOptimization();
      opt.optimize(4);
      g2o::SE3Quat SE3 = v->estimate();

      q_w_curr_ = SE3.rotation();
      t_w_curr_ = SE3.translation();
    }
  } else {
    printf("not enough points in map to associate, map error");
  }
  odom_ = Eigen::Isometry3d::Identity();
  odom_.linear() = q_w_curr_.toRotationMatrix();
  odom_.translation() = t_w_curr_;

  addPointsToMap(downsample_edge_cloud, downsample_surf_cloud);

  pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_filtered(new pcl::PointCloud<pcl::PointXYZI>());  
  *pointcloud_filtered+=*edge_in;
  *pointcloud_filtered+=*surf_in;
  laser_mapper_->addPointsAndPose(pointcloud_filtered, odom_);

  if (update_odom_cb_) {
    update_odom_cb_(q_w_curr_, t_w_curr_);
  }
}

void OdomEstimator::pointAssociateToMap(pcl::PointXYZI const *const pi,
                                        pcl::PointXYZI *const po) {
  Eigen::Vector3d point_curr(pi->x, pi->y, pi->z);
  Eigen::Vector3d point_w = q_w_curr_ * point_curr + t_w_curr_;
  po->x = point_w.x();
  po->y = point_w.y();
  po->z = point_w.z();
  po->intensity = pi->intensity;
  // po->intensity = 1.0;
}

void OdomEstimator::downSamplingToMap(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr &edge_pc_in,
    pcl::PointCloud<pcl::PointXYZI>::Ptr &edge_pc_out,
    const pcl::PointCloud<pcl::PointXYZI>::Ptr &surf_pc_in,
    pcl::PointCloud<pcl::PointXYZI>::Ptr &surf_pc_out) {
  downsize_filter_edge_.setInputCloud(edge_pc_in);
  downsize_filter_edge_.filter(*edge_pc_out);
  downsize_filter_surf_.setInputCloud(surf_pc_in);
  downsize_filter_surf_.filter(*surf_pc_out);
}

void OdomEstimator::addEdgeCostFactor(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& pc_in,
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& map_in,
    g2o::SparseOptimizer& opt, g2o::VertexSE3Expmap* v) {
  int corner_num = 0;
  for (int i = 0; i < static_cast<int>(pc_in->points.size()); i++) {
    pcl::PointXYZI point_temp;
    pointAssociateToMap(&(pc_in->points[i]), &point_temp);

    std::vector<int> point_search_ind;
    std::vector<float> point_search_sq_dis;
    kdtree_edge_map_->nearestKSearch(point_temp, 5, point_search_ind,
                                  point_search_sq_dis);
    if (point_search_sq_dis[4] < 1.0) {
      std::vector<Eigen::Vector3d> nearCorners;
      Eigen::Vector3d center(0, 0, 0);
      for (int j = 0; j < 5; j++) {
        Eigen::Vector3d tmp(map_in->points[point_search_ind[j]].x,
                            map_in->points[point_search_ind[j]].y,
                            map_in->points[point_search_ind[j]].z);
        center = center + tmp;
        nearCorners.push_back(tmp);
      }
      center = center / 5.0;

      Eigen::Matrix3d cov_mat = Eigen::Matrix3d::Zero();
      for (int j = 0; j < 5; j++) {
        Eigen::Matrix<double, 3, 1> tmp_zero_mean = nearCorners[j] - center;
        cov_mat = cov_mat + tmp_zero_mean * tmp_zero_mean.transpose();
      }

      Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> saes(cov_mat);

      Eigen::Vector3d unit_direction = saes.eigenvectors().col(2);
      Eigen::Vector3d curr_point(pc_in->points[i].x, pc_in->points[i].y,
                                 pc_in->points[i].z);
      if (saes.eigenvalues()[2] > 3 * saes.eigenvalues()[1]) {
        Eigen::Vector3d point_on_line = center;
        Eigen::Vector3d point_a, point_b;
        point_a = 0.1 * unit_direction + point_on_line;
        point_b = -0.1 * unit_direction + point_on_line;

        EdgeFeatureEdge* edge =
            new EdgeFeatureEdge(point_a, point_b, curr_point);
        edge->setId(i);
        edge->setVertex(0, v);
        // edge->setMeasurement(0);
        edge->setInformation(Eigen::Matrix<double, 1, 1>::Identity());
        opt.addEdge(edge);

        corner_num++;
      }
    }
  }
  if (corner_num < 20) {
    printf("not enough correct points\n");
  }
}

void OdomEstimator::addSurfCostFactor(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& pc_in,
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& map_in,
    g2o::SparseOptimizer& opt, g2o::VertexSE3Expmap* v) {
  int surf_num = 0;
  for (int i = 0; i < static_cast<int>(pc_in->points.size()); i++) {
    pcl::PointXYZI point_temp;
    pointAssociateToMap(&(pc_in->points[i]), &point_temp);
    std::vector<int> point_search_ind;
    std::vector<float> point_search_sq_dis;
    kdtree_surf_map_->nearestKSearch(point_temp, 5, point_search_ind,
                                  point_search_sq_dis);

    Eigen::Matrix<double, 5, 3> matA0;
    Eigen::Matrix<double, 5, 1> matB0 =
        -1 * Eigen::Matrix<double, 5, 1>::Ones();
    if (point_search_sq_dis[4] < 1.0) {
      for (int j = 0; j < 5; j++) {
        matA0(j, 0) = map_in->points[point_search_ind[j]].x;
        matA0(j, 1) = map_in->points[point_search_ind[j]].y;
        matA0(j, 2) = map_in->points[point_search_ind[j]].z;
      }
      // find the norm of plane
      Eigen::Vector3d norm = matA0.colPivHouseholderQr().solve(matB0);
      double negative_OA_dot_norm = 1 / norm.norm();
      norm.normalize();

      bool planeValid = true;
      for (int j = 0; j < 5; j++) {
        // if OX * n > 0.2, then plane is not fit well
        if (fabs(norm(0) * map_in->points[point_search_ind[j]].x +
                 norm(1) * map_in->points[point_search_ind[j]].y +
                 norm(2) * map_in->points[point_search_ind[j]].z +
                 negative_OA_dot_norm) > 0.2) {
          planeValid = false;
          break;
        }
      }
      Eigen::Vector3d curr_point(pc_in->points[i].x, pc_in->points[i].y,
                                 pc_in->points[i].z);
      if (planeValid) {
        EdgeFeatureSurface* edge = new EdgeFeatureSurface(curr_point, norm);
        edge->setId(i);
        edge->setVertex(0, v);
        edge->setMeasurement(negative_OA_dot_norm);
        edge->setInformation(Eigen::Matrix<double, 1, 1>::Identity());
        opt.addEdge(edge);

        surf_num++;
      }
    }
  }
  if (surf_num < 20) {
    printf("not enough correct points\n");
  }
}

void OdomEstimator::addPointsToMap(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& downsample_edge_cloud,
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& downsample_surf_cloud) {
  for (int i = 0; i < static_cast<int>(downsample_edge_cloud->points.size()); i++) {
    pcl::PointXYZI point_temp;
    pointAssociateToMap(&downsample_edge_cloud->points[i], &point_temp);
    laser_cloud_corner_map_->push_back(point_temp);
  }

  for (int i = 0; i < static_cast<int>(downsample_surf_cloud->points.size()); i++) {
    pcl::PointXYZI point_temp;
    pointAssociateToMap(&downsample_surf_cloud->points[i], &point_temp);
    laser_cloud_surf_map_->push_back(point_temp);
  }

  double x_min = +odom_.translation().x() - 100;
  double y_min = +odom_.translation().y() - 100;
  double z_min = +odom_.translation().z() - 100;
  double x_max = +odom_.translation().x() + 100;
  double y_max = +odom_.translation().y() + 100;
  double z_max = +odom_.translation().z() + 100;

  crop_box_filter_.setMin(Eigen::Vector4f(x_min, y_min, z_min, 1.0));
  crop_box_filter_.setMax(Eigen::Vector4f(x_max, y_max, z_max, 1.0));
  crop_box_filter_.setNegative(false);

  pcl::PointCloud<pcl::PointXYZI>::Ptr tmpCorner(
      new pcl::PointCloud<pcl::PointXYZI>());
  pcl::PointCloud<pcl::PointXYZI>::Ptr tmpSurf(
      new pcl::PointCloud<pcl::PointXYZI>());
  crop_box_filter_.setInputCloud(laser_cloud_surf_map_);
  crop_box_filter_.filter(*tmpSurf);
  crop_box_filter_.setInputCloud(laser_cloud_corner_map_);
  crop_box_filter_.filter(*tmpCorner);

  downsize_filter_surf_.setInputCloud(tmpSurf);
  downsize_filter_surf_.filter(*laser_cloud_surf_map_);
  downsize_filter_edge_.setInputCloud(tmpCorner);
  downsize_filter_edge_.filter(*laser_cloud_corner_map_);
}

void OdomEstimator::getMap(
    pcl::PointCloud<pcl::PointXYZI>::Ptr& laserCloudMap) {
  *laserCloudMap += *laser_cloud_surf_map_;
  *laserCloudMap += *laser_cloud_corner_map_;
}

}  // namespace floam