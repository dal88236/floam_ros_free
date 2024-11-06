#ifndef FLOAM_G2O_TYPE_H_
#define FLOAM_G2O_TYPE_H_

#include <g2o/core/base_unary_edge.h>
#include <g2o/types/sba/vertex_se3_expmap.h>

namespace floam {

class EdgeFeatureEdge : public g2o::BaseUnaryEdge<1, double, g2o::VertexSE3Expmap> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  EdgeFeatureEdge(Eigen::Vector3d pa, Eigen::Vector3d pb, Eigen::Vector3d cp) : lp_a(pa), lp_b(pb), c_p(cp) {}

  bool read(std::istream &in) { return true; }
  bool write(std::ostream &out) const { return true; }

  void computeError() override;
  void linearizeOplus() override;

 private:
  Eigen::Vector3d lp_a, lp_b, c_p;
}; // class EdgeFeatureEdge

class EdgeFeatureSurface : public g2o::BaseUnaryEdge<1, double, g2o::VertexSE3Expmap> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  EdgeFeatureSurface(Eigen::Vector3d cur_p, Eigen::Vector3d p_nor) : c_p(cur_p), p_n(p_nor) {}

  bool read(std::istream &in) { return true; }
  bool write(std::ostream &out) const { return true; }

  void computeError() override;
  void linearizeOplus() override;

 private:
  Eigen::Vector3d c_p, p_n;
}; // class EdgeFeatureSurface

} // namespace floam

#endif // FLOAM_G2O_TYPE_H_
