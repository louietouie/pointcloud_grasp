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

## ROS2 Nodes

#### pc_processor_node

This calculates the normals using the depth image (not the pointcloud), from a single RealSense camera. When using the depth image instead of a pointcloud, finding the nearest-neighboring points is faster, because the assumption is made that nearby points in 3D space will also be nearby pixels in the 2D image. This assumption fails around edges where the depth quickly changes, but in general, works well. The algorithm I used to calculate normals is described in [Chapter 5][4] of the MIT Robot Manipulation course.

#### camera_calibrator_node

Backs out the original camera position from an image, given a known 3D point representation of the calibrator object (chessboard or chess cube). Right now it just publishes these positions to a topic as markers in a MarkerArray for Rviz, but I hope to also add a Pose message to publish just the camera pose. Because this alignment is a linear least-squares problem (meaning the objective function is linear), the solver uses OpenCVs solvePnP, which uses Levenberg-Marquardt to iteratively minimize the distance between the 3D points and the 2D pixel projection. I believe solvePnP is a correspondence-based registration, which means that the 2D to 3D point matches need to be given in advance (it doesn't find the best matches between points, just the best pose). This is in constrast to simulateous pose and correspondence algorithms like ICP, CPD, and Bundle Adjustment. For our problem, that we use solvePnP with Levenberg-Marquardt for, I believe we could also just use the Moore Penrose PseudoInverse to solve for *a* in *Ma = b* where *M* is our ?, *a* is the 8 values of our projective homography (assuming *h_22 = 1* since homogeneous coordinates all scales are the same), and *b* is ?. I've used this to find homographys between two 2D images (for panorama fitting), and imagine there is a similar reframed problem for this case, where it would work (as long as the true *h_22 != 0*). So right now, while I do use Levenberg-Marquardt, I am unsure why not to just use MP pseudoinverse.

[1]: http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/Image.html
[2]: http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/PointCloud2.html
[3]: https://github.com/IntelRealSense/realsense-ros?tab=readme-ov-file#installation-on-ubuntu
[4]: https://manipulation.csail.mit.edu/clutter.html