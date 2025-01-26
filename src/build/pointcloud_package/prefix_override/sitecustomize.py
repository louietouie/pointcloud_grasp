import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/louis/Documents/ros2_projects/pointcloud_grasp/src/install/pointcloud_package'
