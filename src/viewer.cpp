#include "floam_ros_free/viewer.h"

namespace floam {

Viewer::Viewer(SaveMapCallback save_map_cb) : save_map_cb_(save_map_cb), stop_(false) {
  current_pose_.SetIdentity();
  buildColorTable();
}

Viewer::~Viewer() {}

void Viewer::run() {
  pangolin::CreateWindowAndBind("FLOAM: Map Viewer", 1024, 768);

  glEnable(GL_DEPTH_TEST);

  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

  pangolin::CreatePanel("menu").SetBounds(0.0, 1.0, 0.0, pangolin::Attach::Pix(175));
  pangolin::Var<bool> menu_follow_robot("menu.Robot View", false, false);
  pangolin::Var<bool> menu_top_view("menu.Top View", false, false);
  pangolin::Var<bool> menu_stop("menu.Stop", false, false);

  pangolin::OpenGlRenderState s_cam(
    pangolin::ProjectionMatrix(1024, 768, 500.0, 500.0, 512, 389, 0.1, 1000),
    pangolin::ModelViewLookAt(-0.05, 0.0, 0, 0, 0, 0, 0.0, 0.0, 1.0));

  pangolin::View& d_cam = pangolin::CreateDisplay()
    .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f/768.0f)
    .SetHandler(new pangolin::Handler3D(s_cam));

  pangolin::OpenGlMatrix current_pose;

  while (!stop_) {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    updatePose();

    if (menu_follow_robot) {
      menu_follow_robot = false;
      s_cam.SetProjectionMatrix(pangolin::ProjectionMatrix(1024, 768, 500.0, 500.0, 512, 389, 0.1, 1000));
      s_cam.SetModelViewMatrix(pangolin::ModelViewLookAt(-0.05, 0.0, 0, 0, 0, 0, 0.0, 0.0, 1.0));
    }

    if (menu_top_view) {
      menu_top_view = false;
      s_cam.SetProjectionMatrix(pangolin::ProjectionMatrix(1024, 768, 3000.0, 3000.0, 512, 389, 0.1, 10000));
      s_cam.SetModelViewMatrix(pangolin::ModelViewLookAt(0, 0.01, 50, 0, 0, 0, pangolin::AxisY));
    }
      
    s_cam.Follow(current_pose_);
    d_cam.Activate(s_cam);
    glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

    drawMap();
    drawTrajectory();

    pangolin::FinishFrame();
  }
}

void Viewer::requestStop() {
  stop_.store(true);
}

void Viewer::addOdom(const Eigen::Quaterniond& q_wb, const Eigen::Vector3d& t_wb) {
  pangolin::OpenGlMatrix T_wb;
  T_wb.SetIdentity();

  Eigen::Matrix3d rot_wb = q_wb.toRotationMatrix();

  for (int i = 0; i < 3; ++i) {
    T_wb.m[4 * i] = rot_wb(0, i);
    T_wb.m[4 * i + 1] = rot_wb(1, i);
    T_wb.m[4 * i + 2] = rot_wb(2, i);
  }
  T_wb.m[12] = t_wb.x();
  T_wb.m[13] = t_wb.y();
  T_wb.m[14] = t_wb.z();

  std::lock_guard<std::mutex> lk(odom_mtx_);
  pose_ = T_wb;
}

void Viewer::updateMap(pcl::PointCloud<pcl::PointXYZI>& map_pointcloud) {
  std::lock_guard<std::mutex> lk(map_mtx_);
  map_pointcloud_ = std::move(map_pointcloud);
}

void Viewer::buildColorTable() {
  color_table_.reserve(255 * 6);
    auto make_color = [](int r, int g, int b) -> Eigen::Vector4f { return Eigen::Vector4f(r / 255.0f, g / 255.0f, b / 255.0f, 0.2f); };
    for (int i = 0; i < 256; i++) {
        color_table_.emplace_back(make_color(255, i, 0));
    }
    for (int i = 0; i < 256; i++) {
        color_table_.emplace_back(make_color(255 - i, 0, 255));
    }
    for (int i = 0; i < 256; i++) {
        color_table_.emplace_back(make_color(0, 255, i));
    }
    for (int i = 0; i < 256; i++) {
        color_table_.emplace_back(make_color(255, 255 - i, 0));
    }
    for (int i = 0; i < 256; i++) {
        color_table_.emplace_back(make_color(i, 0, 255));
    }
    for (int i = 0; i < 256; i++) {
        color_table_.emplace_back(make_color(0, 255, 255 - i));
    }
}

void Viewer::drawTrajectory() {
  const float w = 0.08;
  const float h = w * 0.75;
  const float z = w * 0.6;

  for (std::list<pangolin::OpenGlMatrix>::iterator it = odom_list_.begin(); it != odom_list_.end(); it++) {
    glPushMatrix();
    glMultMatrixd(it->m);

    glLineWidth(3.0);
    glColor3f(0.0f, 0.0f, 1.0f);
    glBegin(GL_LINES);
    glVertex3f(0, 0, 0);
    glVertex3f(z, -w, -h);
    glVertex3f(0, 0, 0);
    glVertex3f(z, w, -h);
    glVertex3f(0, 0, 0);
    glVertex3f(z, w, h);
    glVertex3f(0, 0, 0);
    glVertex3f(z, -w, h);

    glVertex3f(z, -w, -h);
    glVertex3f(z, w, -h);

    glVertex3f(z, -w, h);
    glVertex3f(z, w, h);

    glVertex3f(z, -w, h);
    glVertex3f(z, -w, -h);

    glVertex3f(z, w, h);
    glVertex3f(z, w, -h);
    glEnd();

    glPopMatrix();
  }

  // Draw current pose
  glPushMatrix();
  glMultMatrixd(current_pose_.m);

  glLineWidth(3.0);
  glColor3f(0.0f, 1.0f, 0.0f);
  glBegin(GL_LINES);
  glVertex3f(0, 0, 0);
  glVertex3f(z, -w, -h);
  glVertex3f(0, 0, 0);
  glVertex3f(z, w, -h);
  glVertex3f(0, 0, 0);
  glVertex3f(z, w, h);
  glVertex3f(0, 0, 0);
  glVertex3f(z, -w, h);

  glVertex3f(z, -w, -h);
  glVertex3f(z, w, -h);

  glVertex3f(z, -w, h);
  glVertex3f(z, w, h);

  glVertex3f(z, -w, h);
  glVertex3f(z, -w, -h);

  glVertex3f(z, w, h);
  glVertex3f(z, w, -h);
  glEnd();

  glPopMatrix();

  while (odom_list_.size() > ODOM_DISPLAY_NUM)
    odom_list_.pop_front();
  odom_list_.push_back(current_pose_);
}

void Viewer::drawMap() {
  std::lock_guard<std::mutex> lk(map_mtx_);

  if (map_pointcloud_.empty())
    return;

  glPointSize(2.0);
  glBegin(GL_POINTS);
  
  for(size_t i = 0; i < map_pointcloud_.size(); ++i) {
    int idx = map_pointcloud_[i].z * 60;
    idx = idx % color_table_.size();
    glColor4f(color_table_[idx](0), color_table_[idx](1), color_table_[idx](2), color_table_[idx](3));
    glVertex3f(map_pointcloud_[i].x, map_pointcloud_[i].y, map_pointcloud_[i].z);
  }
  glEnd();
}

void Viewer::updatePose() {
  std::lock_guard<std::mutex> lk(odom_mtx_);
  current_pose_ = pose_;
}

} // namespace floam