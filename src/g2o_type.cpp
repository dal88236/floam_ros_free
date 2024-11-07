#include "floam_ros_free/g2o_type.h"

#include <sophus/se3.hpp>

namespace floam {

void EdgeFeatureEdge::computeError() {
  g2o::VertexSE3Expmap* v = static_cast<g2o::VertexSE3Expmap*>(_vertices[0]);
  Eigen::Matrix3d R = v->estimate().rotation().toRotationMatrix();
  Eigen::Vector3d t = v->estimate().translation();
  Eigen::Vector3d lp = R * c_p + t;
  Eigen::Vector3d nu = (lp - lp_a).cross(lp - lp_b);
  Eigen::Vector3d de = lp_a - lp_b;
  double de_norm = de.norm();
  double nu_norm = nu.norm();

  _error(0, 0) = nu_norm / de_norm;
}

void EdgeFeatureEdge::linearizeOplus() {
  g2o::VertexSE3Expmap* v = static_cast<g2o::VertexSE3Expmap*>(_vertices[0]);
  Eigen::Matrix3d R = v->estimate().rotation().toRotationMatrix();
  Eigen::Vector3d t = v->estimate().translation();
  Eigen::Matrix3d skew_lp = Sophus::SO3d::hat(R * c_p + t);
  Eigen::Vector3d lp = R * c_p + t;
  Eigen::Vector3d nu = (lp - lp_a).cross(lp - lp_b);
  Eigen::Vector3d de = lp_a - lp_b;
  double de_norm = de.norm();
  Eigen::Matrix<double, 3, 6> dp_by_se3;
  dp_by_se3.block<3, 3>(0, 3).setIdentity();
  dp_by_se3.block<3, 3>(0, 0) = -skew_lp;
  Eigen::Matrix3d skew_de = Sophus::SO3d::hat(lp_a - lp_b);
  _jacobianOplusXi.block<1, 6>(0, 0) = -nu.transpose() / nu.norm() * skew_de * dp_by_se3 / de_norm;
}

void EdgeFeatureSurface::computeError() {
  g2o::VertexSE3Expmap* v = static_cast<g2o::VertexSE3Expmap*>(_vertices[0]);
  Eigen::Matrix3d R = v->estimate().rotation().toRotationMatrix();
  Eigen::Vector3d t = v->estimate().translation();
  Eigen::Vector3d point_w = R * c_p + t;

  _error(0, 0) = p_n.dot(point_w) + _measurement;
}

void EdgeFeatureSurface::linearizeOplus() {
  g2o::VertexSE3Expmap* v = static_cast<g2o::VertexSE3Expmap*>(_vertices[0]);
  Eigen::Matrix3d R = v->estimate().rotation().toRotationMatrix();
  Eigen::Vector3d t = v->estimate().translation();
  Eigen::Matrix3d skew_point_w = Sophus::SO3d::hat(R * c_p + t); 
  Eigen::Matrix<double, 3, 6> dp_by_se3;
  dp_by_se3.block<3, 3>(0, 3).setIdentity();
  dp_by_se3.block<3, 3>(0, 0) = -skew_point_w;
  _jacobianOplusXi.block<1, 6>(0, 0) = p_n.transpose() * dp_by_se3;
}


} // namespace floam