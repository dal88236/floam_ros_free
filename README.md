# floam_ros_free
This repository is a modified LiDAR odometry system, which is developed based on the open-source odometry framework [**FLOAM**](https://github.com/wh200720041/floam) and g2o version [**FLOAM_G2O**](https://github.com/chengwei0427/floam_g2o)

## 1. Demo Highlights
Watch our demo at [Video Link](https://youtu.be/PzZly1SQtng)
<p align='center'>
<a href="https://youtu.be/PzZly1SQtng">
<img width="65%" src="/img/floam_kitti.gif"/>
</a>
</p>

## 2. Evaluation
### 2.1. Computational efficiency evaluation
Computational efficiency evaluation (based on KITTI dataset):
Platform: Intel® Core™ i7-8700 CPU @ 3.20GHz 
| Dataset                                      | ALOAM                      | FLOAM                  |
|----------------------------------------------|----------------------------|------------------------|
| `KITTI`                                      | 151ms                      | 59ms                   |

Localization error:
| Dataset                                      | ALOAM                      | FLOAM                  |
|----------------------------------------------|----------------------------|------------------------|
| `KITTI sequence 00`                          | 0.55%                      | 0.51%                  |
| `KITTI sequence 02`                          | 3.93%                      | 1.25%                  |
| `KITTI sequence 05`                          | 1.28%                      | 0.93%                  |

### 2.2. localization result
<p align='center'>
<img width="65%" src="/img/kitti_example.gif"/>
</p>

### 2.3. mapping result
<p align='center'>
<a href="https://youtu.be/w_R0JAymOSs">
<img width="65%" src="/img/floam_mapping.gif"/>
</a>
</p>

## 3. Prerequisites
### 3.1. **PCL**
Follow [PCL Installation](http://www.pointclouds.org/downloads/linux.html).

### 3.2. **G2O**
https://github.com/RainerKuemmerle/g2o

### 3.3. **Sophus**
https://github.com/strasdat/Sophus

### 3.4. **YAML-CPP**
https://github.com/jbeder/yaml-cpp

### 3.5. **ROS2(Optional)**
Follow [ROS2 Installation](https://docs.ros.org/en/dashing/Installation/Ubuntu-Install-Binary.html)

## 4. Build
### 4.1 Clone repository:
```
    git clone https://github.com/dal88236/floam_ros_free.git
    . build.sh
    . build_ros.sh (optional)
```

## 5. Test on other sequence
To generate rosbag file of kitti dataset, you may use the tools provided by 
[kitti_to_rosbag](https://github.com/ethz-asl/kitti_to_rosbag) or [kitti2bag](https://github.com/tomas789/kitti2bag) 

## 6. Test on Velodyne VLP-16 or HDL-32
You may wish to test FLOAM on your own platform and sensor such as VLP-16
You can install the velodyne sensor driver by 
```
sudo apt-get install ros-<your_version>-velodyne-pointcloud
```
launch floam for your own velodyne sensor
```
build/vlp_exec vlp-16.yaml
```
or using ros wrapper
```
ros_wrapper/build/floam_ros vlp-16.yaml
```
If you are using HDL-32 or other sensor, please change the scan_line in the launch file 

## 7.Acknowledgements
Thanks for [A-LOAM](https://github.com/HKUST-Aerial-Robotics/A-LOAM) and LOAM(J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time) and [LOAM_NOTED](https://github.com/cuitaixiang/LOAM_NOTED).


## 8. Citation
If you use this work for your research, you may want to cite
```
@inproceedings{wang2021,
  author={H. {Wang} and C. {Wang} and C. {Chen} and L. {Xie}},
  booktitle={2021 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)}, 
  title={F-LOAM : Fast LiDAR Odometry and Mapping}, 
  year={2020},
  volume={},
  number={}
}
```