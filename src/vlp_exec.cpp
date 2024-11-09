#include "floam_ros_free/system.h"

#include <pcl/io/vlp_grabber.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <boost/function.hpp>
#include <boost/bind.hpp>
#include <boost/lambda/lambda.hpp>

#include <yaml-cpp/yaml.h>

#include <csignal>

volatile bool stop = false;

void signalHandler(int signum) {
  std::cout << "\nInterrupt signal (" << signum << ") received. Stopping grabber..." << std::endl;
  stop = true;
}

int main(int argc, char** argv) {
  if (argc != 2) {
    std::cerr << "Usage shall be: floam_ros {config_file_path}" << std::endl;
    return 1;
  }
  std::signal(SIGINT, signalHandler);

  std::string config_path(argv[1]);
  floam::System system(config_path);

  // TODO pub odom and visualize map

  YAML::Node config = YAML::LoadFile(config_path);
  std::string ip_address_str = config["ip_address"].as<std::string>();
  int port = config["port"].as<int>();
  boost::asio::ip::address ip_address = boost::asio::ip::make_address(ip_address_str);

  std::cout << "Initializing VLP grabber..." << std::endl;

  pcl::VLPGrabber grabber(ip_address, port);
  std::function<void(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& pointcloud)> callback = 
    [&system](const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& pointcloud) {
        system.track(pointcloud);
    };
  grabber.registerCallback(callback);

  grabber.start();

  while (!stop) {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  grabber.stop();

  std::cout << "VLP grabber stopped" << std::endl;

  return 0;
}