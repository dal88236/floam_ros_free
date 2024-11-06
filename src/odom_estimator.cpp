#include "floam_ros_free/odom_estimator.h"
#include "floam_ros_free/laser_mapper.h"

#include <g2o/core/optimization_algorithm_gauss_newton.h>

#include <chrono>

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

void OdomEstimator::init(lidar::Lidar lidar_param, double map_resolution) {
  // init local map
  laserCloudCornerMap = pcl::PointCloud<pcl::PointXYZI>::Ptr(
      new pcl::PointCloud<pcl::PointXYZI>());
  laserCloudSurfMap = pcl::PointCloud<pcl::PointXYZI>::Ptr(
      new pcl::PointCloud<pcl::PointXYZI>());

  // downsampling size
  downSizeFilterEdge.setLeafSize(map_resolution, map_resolution,
                                 map_resolution);
  downSizeFilterSurf.setLeafSize(map_resolution * 2, map_resolution * 2,
                                 map_resolution * 2);

  // kd-tree
  kdtreeEdgeMap = pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr(
      new pcl::KdTreeFLANN<pcl::PointXYZI>());
  kdtreeSurfMap = pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr(
      new pcl::KdTreeFLANN<pcl::PointXYZI>());

  odom = Eigen::Isometry3d::Identity();
  last_odom = Eigen::Isometry3d::Identity();
  optimization_count = 2;
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

  *laserCloudCornerMap += *edge_in;
  *laserCloudSurfMap += *surf_in;
  optimization_count = 12;
  init_ = true;
}

void OdomEstimator::updatePointsToMap(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& edge_in,
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& surf_in) {
  if (optimization_count > 2) optimization_count--;

  Eigen::Isometry3d odom_prediction = odom * (last_odom.inverse() * odom);
  last_odom = odom;
  odom = odom_prediction;

  pcl::PointCloud<pcl::PointXYZI>::Ptr downsampledEdgeCloud(
      new pcl::PointCloud<pcl::PointXYZI>());
  pcl::PointCloud<pcl::PointXYZI>::Ptr downsampledSurfCloud(
      new pcl::PointCloud<pcl::PointXYZI>());
  downSamplingToMap(edge_in, downsampledEdgeCloud, surf_in,
                    downsampledSurfCloud);

  q_w_curr = Eigen::Quaterniond(odom.rotation());
  t_w_curr = odom.translation();

  if (laserCloudCornerMap->points.size() > 10 &&
      laserCloudSurfMap->points.size() > 50) {
    kdtreeEdgeMap->setInputCloud(laserCloudCornerMap);
    kdtreeSurfMap->setInputCloud(laserCloudSurfMap);

    for (int iterCount = 0; iterCount < optimization_count; iterCount++) {
      
      std::unique_ptr<LinearSolverType> linearSolver = std::make_unique<LinearSolverType>();
      std::unique_ptr<BlockSolverType> blockSolver = std::make_unique<BlockSolverType>(std::move(linearSolver));
      g2o::OptimizationAlgorithmGaussNewton* solver = new g2o::OptimizationAlgorithmGaussNewton(std::move(blockSolver));
      g2o::SparseOptimizer opt;
      opt.setAlgorithm(solver);
      opt.setVerbose(false);

      g2o::VertexSE3Expmap* v = new g2o::VertexSE3Expmap();
      v->setEstimate(g2o::SE3Quat(q_w_curr, t_w_curr));
      v->setId(0);
      opt.addVertex(v);

      //  add edge here
      addEdgeCostFactor(downsampledEdgeCloud, laserCloudCornerMap, opt, v);
      addSurfCostFactor(downsampledSurfCloud, laserCloudSurfMap, opt, v);

      opt.initializeOptimization();
      opt.optimize(10);
      g2o::SE3Quat SE3 = v->estimate();

      q_w_curr = SE3.rotation();
      t_w_curr = SE3.translation();
    }
  } else {
    printf("not enough points in map to associate, map error");
  }
  odom = Eigen::Isometry3d::Identity();
  odom.linear() = q_w_curr.toRotationMatrix();
  odom.translation() = t_w_curr;

  addPointsToMap(downsampledEdgeCloud, downsampledSurfCloud);

  pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_filtered(new pcl::PointCloud<pcl::PointXYZI>());  
  *pointcloud_filtered+=*edge_in;
  *pointcloud_filtered+=*surf_in;
  laser_mapper_->addPointsAndPose(pointcloud_filtered, odom);

  if (update_odom_cb_) {
    update_odom_cb_(q_w_curr, t_w_curr);
  }
}

void OdomEstimator::pointAssociateToMap(pcl::PointXYZI const *const pi,
                                        pcl::PointXYZI *const po) {
  Eigen::Vector3d point_curr(pi->x, pi->y, pi->z);
  Eigen::Vector3d point_w = q_w_curr * point_curr + t_w_curr;
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
  downSizeFilterEdge.setInputCloud(edge_pc_in);
  downSizeFilterEdge.filter(*edge_pc_out);
  downSizeFilterSurf.setInputCloud(surf_pc_in);
  downSizeFilterSurf.filter(*surf_pc_out);
}

void OdomEstimator::addEdgeCostFactor(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& pc_in,
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& map_in,
    g2o::SparseOptimizer& opt, g2o::VertexSE3Expmap* v) {
  int corner_num = 0;
  for (int i = 0; i < static_cast<int>(pc_in->points.size()); i++) {
    pcl::PointXYZI point_temp;
    pointAssociateToMap(&(pc_in->points[i]), &point_temp);

    std::vector<int> pointSearchInd;
    std::vector<float> pointSearchSqDis;
    kdtreeEdgeMap->nearestKSearch(point_temp, 5, pointSearchInd,
                                  pointSearchSqDis);
    if (pointSearchSqDis[4] < 1.0) {
      std::vector<Eigen::Vector3d> nearCorners;
      Eigen::Vector3d center(0, 0, 0);
      for (int j = 0; j < 5; j++) {
        Eigen::Vector3d tmp(map_in->points[pointSearchInd[j]].x,
                            map_in->points[pointSearchInd[j]].y,
                            map_in->points[pointSearchInd[j]].z);
        center = center + tmp;
        nearCorners.push_back(tmp);
      }
      center = center / 5.0;

      Eigen::Matrix3d covMat = Eigen::Matrix3d::Zero();
      for (int j = 0; j < 5; j++) {
        Eigen::Matrix<double, 3, 1> tmpZeroMean = nearCorners[j] - center;
        covMat = covMat + tmpZeroMean * tmpZeroMean.transpose();
      }

      Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> saes(covMat);

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
    printf("not enough correct points");
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
    std::vector<int> pointSearchInd;
    std::vector<float> pointSearchSqDis;
    kdtreeSurfMap->nearestKSearch(point_temp, 5, pointSearchInd,
                                  pointSearchSqDis);

    Eigen::Matrix<double, 5, 3> matA0;
    Eigen::Matrix<double, 5, 1> matB0 =
        -1 * Eigen::Matrix<double, 5, 1>::Ones();
    if (pointSearchSqDis[4] < 1.0) {
      for (int j = 0; j < 5; j++) {
        matA0(j, 0) = map_in->points[pointSearchInd[j]].x;
        matA0(j, 1) = map_in->points[pointSearchInd[j]].y;
        matA0(j, 2) = map_in->points[pointSearchInd[j]].z;
      }
      // find the norm of plane
      Eigen::Vector3d norm = matA0.colPivHouseholderQr().solve(matB0);
      double negative_OA_dot_norm = 1 / norm.norm();
      norm.normalize();

      bool planeValid = true;
      for (int j = 0; j < 5; j++) {
        // if OX * n > 0.2, then plane is not fit well
        if (fabs(norm(0) * map_in->points[pointSearchInd[j]].x +
                 norm(1) * map_in->points[pointSearchInd[j]].y +
                 norm(2) * map_in->points[pointSearchInd[j]].z +
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
    printf("not enough correct points");
  }
}

void OdomEstimator::addPointsToMap(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr &downsampledEdgeCloud,
    const pcl::PointCloud<pcl::PointXYZI>::Ptr &downsampledSurfCloud) {
  for (int i = 0; i < static_cast<int>(downsampledEdgeCloud->points.size()); i++) {
    pcl::PointXYZI point_temp;
    pointAssociateToMap(&downsampledEdgeCloud->points[i], &point_temp);
    laserCloudCornerMap->push_back(point_temp);
  }

  for (int i = 0; i < static_cast<int>(downsampledSurfCloud->points.size()); i++) {
    pcl::PointXYZI point_temp;
    pointAssociateToMap(&downsampledSurfCloud->points[i], &point_temp);
    laserCloudSurfMap->push_back(point_temp);
  }

  double x_min = +odom.translation().x() - 100;
  double y_min = +odom.translation().y() - 100;
  double z_min = +odom.translation().z() - 100;
  double x_max = +odom.translation().x() + 100;
  double y_max = +odom.translation().y() + 100;
  double z_max = +odom.translation().z() + 100;

  cropBoxFilter.setMin(Eigen::Vector4f(x_min, y_min, z_min, 1.0));
  cropBoxFilter.setMax(Eigen::Vector4f(x_max, y_max, z_max, 1.0));
  cropBoxFilter.setNegative(false);

  pcl::PointCloud<pcl::PointXYZI>::Ptr tmpCorner(
      new pcl::PointCloud<pcl::PointXYZI>());
  pcl::PointCloud<pcl::PointXYZI>::Ptr tmpSurf(
      new pcl::PointCloud<pcl::PointXYZI>());
  cropBoxFilter.setInputCloud(laserCloudSurfMap);
  cropBoxFilter.filter(*tmpSurf);
  cropBoxFilter.setInputCloud(laserCloudCornerMap);
  cropBoxFilter.filter(*tmpCorner);

  downSizeFilterSurf.setInputCloud(tmpSurf);
  downSizeFilterSurf.filter(*laserCloudSurfMap);
  downSizeFilterEdge.setInputCloud(tmpCorner);
  downSizeFilterEdge.filter(*laserCloudCornerMap);
}

void OdomEstimator::getMap(
    pcl::PointCloud<pcl::PointXYZI>::Ptr& laserCloudMap) {
  *laserCloudMap += *laserCloudSurfMap;
  *laserCloudMap += *laserCloudCornerMap;
}

}  // namespace floam