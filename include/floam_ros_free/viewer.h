#ifndef VIEWER_H_
#define VIEWER_H_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pangolin/pangolin.h>

#include <Eigen/Core>

#include <list>
#include <functional>
#include <atomic>
#include <mutex>

#define ODOM_DISPLAY_NUM 1000

namespace floam {

class Viewer {
 public:
  typedef std::function<void()> SaveMapCallback;

  Viewer(SaveMapCallback save_map_cb);
  ~Viewer();

  void run();
  void requestStop();
  void addOdom(const Eigen::Quaterniond& q_wb, const Eigen::Vector3d& t_wb);
  void updateMap(pcl::PointCloud<pcl::PointXYZI>& map_pointcloud);

 private:
  void buildColorTable();
  void drawTrajectory();
  void drawMap();
  void updatePose();

  std::vector<Eigen::Vector4f> color_table_;
  pcl::PointCloud<pcl::PointXYZI> map_pointcloud_;
  std::list<pangolin::OpenGlMatrix> odom_list_;
  pangolin::OpenGlMatrix pose_;
  pangolin::OpenGlMatrix current_pose_;
  SaveMapCallback save_map_cb_;
  std::atomic_bool stop_;
  std::mutex map_mtx_;
  std::mutex odom_mtx_;
};

} // namespace floam

#endif // VIEWER_H_