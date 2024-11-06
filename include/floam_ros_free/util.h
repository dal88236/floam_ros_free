#ifndef UTIL_H_
#define UTIL_H_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace floam {

void transformPointCloud(const pcl::PointCloud<pcl::PointXYZI>& pc_in, pcl::PointCloud<pcl::PointXYZI>& pc_out, const Eigen::Isometry3f& pose_current) {
  for (int i = 0; i < static_cast<int>(pc_in.size()); ++i) {
    Eigen::Vector3f pt_xyz(pc_in[i].x, pc_in[i].y, pc_in[i].z);
    Eigen::Vector3f transformed_pt_xyz = pose_current * pt_xyz;
    pcl::PointXYZI transformed_pt;
    transformed_pt.x = transformed_pt_xyz.x();
    transformed_pt.y = transformed_pt_xyz.y();
    transformed_pt.z = transformed_pt_xyz.z();
    transformed_pt.intensity = pc_in[i].intensity;
    pc_out.emplace_back(transformed_pt);
  }
}

} // namespace lfoam

#endif // UTIL_H_