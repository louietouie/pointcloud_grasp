# Pointcloud Processing for Manipulation with Realsense D435

This repo contains ROS2 packages that act on the ROS2 topics published by RealSense cameras (depth[Image][1] and [PointCloud2][2]). are useful for common pointcloud manipulation tasks.

## Getting Started

Before running these packages, both ROS2 and a few [realsense libraries][3] need to be installed. The realsense installs include the base software, that gives you commands like `realsense-viewer`, to view the cameras outputs in a GUI. Seperate realsense ROS2 wrappers also need to be installed, to allow for realsense to publish to ROS2 topics with commands like...

1. `ros2 launch realsense2_camera rs_launch.py`
2. `ros2 launch realsense2_camera rs_launch.py pointcloud.enable:=true`
3. `ros2 launch realsense2_camera rs_pointcloud_launch.py`

The packages here have launch files to start everything necessary at once. To launch the nodes to calculate and publish normals based on the depth images of a single realsense camera, call the following commands from the top-level `pointcloud_grasp` directory.

1. `colcon build`
2. `ros2 launch pointcloud_package master.launch.py`

## Packages

#### pc_processor_node.py

This calculates the normals using the depth image (not the pointcloud), from a single RealSense camera. When using the depth image instead of a pointcloud, finding the nearest-neighboring points is faster, because the assumption is made that nearby points in 3D space will also be nearby pixels in the 2D image. This assumption fails around edges where the depth quickly changes, but in general, works well. The algorithm I used to calculate normals is described in [Chapter 5][4] of the MIT Robot Manipulation course.

[1]: http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/Image.html
[2]: http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/PointCloud2.html
[3]: https://github.com/IntelRealSense/realsense-ros?tab=readme-ov-file#installation-on-ubuntu
[4]: https://manipulation.csail.mit.edu/clutter.html