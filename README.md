# YOLO based localization with ROS

## Table of Contents

- [About](#about)
- [Getting Started](#getting_started)
- [Usage](#usage)
- [Contributing](../CONTRIBUTING.md)

## About <a name = "about"></a>

The project is based on the job of https://github.com/leggedrobotics/darknet_ros.git. By combining the output of YOLO bounding box with the depth data, the coordinate_pub node publish the 3D coordinate of specific target.

## Getting Started <a name = "getting_started"></a>
```
roslaunch darknet_ros yolo_v3
rosrun coordinate_target coordinate_pub
```
### Prerequisites

1. Realsense depth camera, like D415
2. A pretrain yolo weight file which is trained under the darknet framwork.

### Installing


```
git clone https://github.com/Adiming/Yolo_ros_coordinate.git
```


## Usage <a name = "usage"></a>

1. place the file under darknet_ros/darknet_ros/yolo_network_config/cfg
2. generate a yaml file under folder darknet_ros/darknet_ros/config for threshold and class setting
