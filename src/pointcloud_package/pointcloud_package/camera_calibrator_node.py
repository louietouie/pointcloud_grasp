import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose # Transform
from visualization_msgs.msg import Marker

class CameraCalibrator(Node):
    
    def __init__(self, chessboard_length):
        super().__init__('camera_calibrator')

        self.subscription = self.create_subscription(Image, '/camera/camera/depth/image_rect_raw', self.listener_callback, 10)
        self.publisher_marker = self.create_publisher(Marker, 'camera_icon', 10)
        self.publisher_pose = self.create_publisher(Pose, 'camera_pose', 10)

        chessboard = ChessboardRepresentation(chessboard_length)
        self.solver = PointToPixelSolver(chessboard)

    def listener_callback(self, msg):
        pose = self.solver.solve(msg)
        marker = self.pose_to_marker(pose)
        self.publisher_marker.publish(marker)
        self.publisher_pose.publish(pose)

    def pose_to_marker():
        marker = Marker()
        return marker

class ChessboardRepresentation():

    def __init__(self, full_length):

        self.full_length = full_length


class PointToPixelSolver():
    
    def __init__(self, chessboard):

        self.chessboard = chessboard

    def solve(self, image):
        
        pose = Pose()
        return pose
    
        # image to grayscale
        # grayscale to corner detection (harris corner or opencv implementation)
        # solver for 3D chessboard points to 2D (Levenberg Marquardt)
            # what about noise of image points? Should Levenberg be done multiple times with RANSAC and use the best one?
        # invert solver homography to get camera position wrt world instead of world position wrt camera?
        # split homography into a ROS2 pose (a Point position and Quaternion quaternion)
