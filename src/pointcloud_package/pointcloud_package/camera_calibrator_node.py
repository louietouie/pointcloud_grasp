import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose # Transform
from visualization_msgs.msg import Marker, MarkerArray

from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

import cv2
import numpy as np

class CameraCalibrator(Node):
    
    def __init__(self, chessboard_length):
        super().__init__('camera_calibrator')

        self.subscription = self.create_subscription(Image, '/camera/camera/depth/image_rect_raw', self.listener_callback, 10)
        self.publisher_marker = self.create_publisher(MarkerArray, 'camera_icon', 10)
        self.publisher_pose = self.create_publisher(Pose, 'camera_pose', 10)

        # chessboard = ChessboardRepresentation(chessboard_length)
        # self.solver = PointToPixelSolver(chessboard)

        row = 6
        col = 7
        
        image = cv2.imread('chessboard_high2.png')
        w = image.shape[1]
        h = image.shape[0]
        mask = np.zeros(image.shape[:2], dtype="uint8")
        cv2.rectangle(mask, (700, 700), (1200, 1000), 255, -1)
        image = cv2.bitwise_and(image, image, mask=mask)

        # cv2.namedWindow("window_o", cv2.WINDOW_NORMAL) 
        # cv2.imshow('window_o', image)
        # cv2.waitKey(100)

        # ret, corners = cv2.findChessboardCorners(image, (row, col), None)
        # ret, corners = cv2.findChessboardCorners(image, (row, col), flags=cv2.CALIB_CB_FILTER_QUADS)
        ret, corners = cv2.findChessboardCornersSB(image, (row, col), None)
        corners = corners.reshape((-1,2))
        object_corners = self.object_corners(chessboard_length, row, col).astype('float32')
        
        camera_matrix = np.float64([[382.77, 0, w/2],
                                    [0, 382.77, h/2],
                                    [0, 0, 1]])
        distorition_matrix = np.array([0,0,0,0,0])
        distorition_matrix2 = np.array([0.,0.,0.,0.,0.])

        ret, rotation, translation = cv2.solvePnP(object_corners, corners, camera_matrix, distorition_matrix)

        print("found rot and translation")
        rot_array, _ = cv2.Rodrigues(rotation)
        print(rot_array)
        print(translation)

        print("inverted rotation and translation")
        rotation_i, translation_i = self.invert_rot_and_pose(rotation, translation)
        rot_array_i, _ = cv2.Rodrigues(rotation_i)
        print(rot_array_i)
        print(translation_i)

        print("flipped rotation and translation")
        # translation_x = translation_i * np.array([[1],[1],[-1]])
        translation_x = translation_i * np.array([1,1,-1])
        rot_array_x, _ = cv2.Rodrigues(rotation_i)

        rot_array_x = np.array([
                            [-1,0,0],
                            [0,-1,0],
                            [0,0,1]
        ]) @ rot_array_x

        rotation_x, _ = cv2.Rodrigues(rot_array_x)

        rotation_x_i, translation_x_i = self.invert_rot_and_pose(rotation_x, translation_x)

        print(rot_array_x)
        print(translation_x)

        markers = MarkerArray()

        marker = self.create_marker(rot_array, translation, 9000, [1,1,1])
        markers.markers.append(marker)

        marker = self.create_marker(rot_array_i, translation_i, 9001, [.5,0,0])
        markers.markers.append(marker)
        
        marker = self.create_marker(rot_array_x, translation_x, 9002, [0,0,0])
        markers.markers.append(marker)
        
        for idx, point in enumerate(corners):
            marker = self.create_marker2([0,0,0], point, idx+3000, [0,1,1])
            markers.markers.append(marker)

        for idx, point in enumerate(object_corners):
            marker = self.create_marker2([0,0,0], point, idx+1, [0,0,1])
            markers.markers.append(marker)

        more = cv2.projectPoints(object_corners, rotation_x_i, translation_x_i, camera_matrix, distorition_matrix2)[0].reshape(-1,2)
        # more = cv2.projectPoints(object_corners, rotation_x_i, translation_x_i, camera_matrix, distorition_matrix2)[0].reshape(-1,2)
        for idx, point in enumerate(more):
            marker = self.create_marker2([0,0,0], point, idx+1000, [1,0,0])
            markers.markers.append(marker)

        marker = self.create_marker2([0,0,0], [w,h,0], idx+2000, [1,1,0])
        markers.markers.append(marker)

        print("publshing")
        self.publisher_marker.publish(markers)
        print("DONE")

        cv2.drawChessboardCorners(image, (row, col), corners, ret)
        cv2.namedWindow("window_z", cv2.WINDOW_NORMAL) 
        cv2.imshow('window_z', image)
        cv2.waitKey(10)


    def invert_rot_and_pose(self, rotation, translation):
        rot_array, _ = cv2.Rodrigues(rotation)
        pose_a = np.hstack((rot_array, translation.reshape((-1,1))))
        pose_b = np.vstack((pose_a, [[0, 0, 0, 1]]))
        pose_inv = np.linalg.inv(pose_b)
        rot_t_array = pose_inv[:3,:3]
        rot_t, _ = cv2.Rodrigues(rot_t_array)
        trans_t = pose_inv[:3,3]
        return (rot_t, trans_t)

    def listener_callback(self, msg):
        return 0
        # pose = self.solver.solve(msg)
        # marker = self.pose_to_marker(pose)
        # self.publisher_marker.publish(marker)
        # self.publisher_pose.publish(pose)

    def object_corners(self, size, row, col):
        x = np.arange(row)
        y = np.arange(col)
        arr = np.meshgrid(x,y)
        z = arr[0] * 0
        corners = np.stack((arr[0], arr[1], z), axis=2) * size
        return corners.reshape((-1,3))

    def pose_to_marker():
        marker = Marker()
        return marker
    
    def create_marker(self, vector, point, id, color):
        marker = Marker()
        marker.header.frame_id = '/camera_depth_optical_frame'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.type = marker.ARROW
        marker.id = id
        marker.action = marker.ADD
        
        marker.scale.x = .02
        marker.scale.y = 0.002
        marker.scale.z = 0.002
        marker.color.r = float(color[0])
        marker.color.g = float(color[1])
        marker.color.b = float(color[2])
        marker.color.a = 1.0

        marker.pose.position.x = float(point[0] / 1000)
        marker.pose.position.y = float(point[1] / 1000)
        marker.pose.position.z = float(point[2] / 1000)

        quaternion = Rotation.from_matrix(vector).as_quat()      
        # quaternion = Rotation.align_vectors([[0,0,-1]],[vector])[0].as_quat()  
        # quaternion = Rotation.align_vectors([vector], [[1,0,0]])[0].as_quat()  

        # print("HERE")
        # print(vector)
        # print(quaternion)

        marker.pose.orientation.x = float(quaternion[0])
        marker.pose.orientation.y = float(quaternion[1])
        marker.pose.orientation.z = float(quaternion[2])
        marker.pose.orientation.w = float(quaternion[3])

        return marker
    
    def create_marker2(self, vector, point, id, color):
        marker = Marker()
        marker.header.frame_id = '/camera_depth_optical_frame'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.type = marker.SPHERE
        marker.id = id
        marker.action = marker.ADD
        
        marker.scale.x = .005
        marker.scale.y = 0.005
        marker.scale.z = 0.005
        marker.color.r = float(color[0])
        marker.color.g = float(color[1])
        marker.color.b = float(color[2])
        marker.color.a = 1.0

        marker.pose.position.x = float(point[0] / 1000)
        marker.pose.position.y = float(point[1] / 1000)
        marker.pose.position.z = float(0)

        # quaternion = Rotation.from_matrix(vector).as_quat()      
        quaternion = Rotation.align_vectors([[0,0,1]],[vector])[0].as_quat()  

        # print("HERE")
        # print(vector)
        # print(quaternion)

        # marker.pose.orientation.x = float(quaternion[0])
        # marker.pose.orientation.y = float(quaternion[1])
        # marker.pose.orientation.z = float(quaternion[2])
        # marker.pose.orientation.w = float(quaternion[3])

        return marker

# class ChessboardRepresentation():

#     def __init__(self, full_length):

#         self.full_length = full_length

#     def create_points(self):

#         return 0


# class PointToPixelSolver():
    
#     def __init__(self, chessboard):

#         self.chessboard = chessboard

#     def solve(self, image, initial_guess):

#         # Ma = b
#         def function(x):
#             return 0

#         best_homography = 0
#         max_inliers = 0
#         for _ in range(ransac_rounds):
#             matches = ransac_sample(image)
#             homography = least_squares(function(matches), initial_guess, method="lm")
#             inliers = 0
#             if inliers > max_inliers:
#                 max_inliers = inliers
#                 best_homography = homography

#         pose = Pose(best_homography)
#         return pose
    
#         # image to grayscale
#         # grayscale to corner detection (harris corner or opencv implementation)
#         # solver for 3D chessboard points to 2D (Levenberg Marquardt)
#             # what about noise of image points? Should Levenberg be done multiple times with RANSAC and use the best one?
#         # invert solver homography to get camera position wrt world instead of world position wrt camera?
#         # split homography into a ROS2 pose (a Point position and Quaternion quaternion)
    
#     def ransac_sample(self, points_3d, points_2d, num_samples):
#         return 0
    
#     def ransac_sample_2(self, matches, num_samples):
#         return 0

def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = CameraCalibrator(10)
    rclpy.spin(minimal_subscriber)
    minimal_subscriber.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()

if __name__ == '__main__':
    main()