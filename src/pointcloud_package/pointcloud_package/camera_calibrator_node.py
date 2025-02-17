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
    
    def __init__(self, camera, calibrator):
        super().__init__('camera_calibrator')

        self.subscription = self.create_subscription(Image, '/camera/camera/depth/image_rect_raw', self.listener_callback, 10)
        self.publisher_marker = self.create_publisher(MarkerArray, 'camera_icon', 10)
        self.publisher_pose = self.create_publisher(Pose, 'camera_pose', 10)

        markers = MarkerArray()

        object_points = calibrator.get_points()

        for idx, point in enumerate(object_points):
            print(point)
            marker = self.create_marker2(point, idx+1, [0,0,1])
            markers.markers.append(marker)

        print(len(markers.markers))
        for idx, i in enumerate(markers.markers):
            print(idx)
            print(i)
            print("______________________")

        rows, cols = calibrator.grid_size()
        camera_points = camera.find_image_points(rows, cols)
        masks = []
        masks.append(cv2.fillPoly(camera.image, pts=[], color=(255,255,255)))
        masks.append(cv2.fillPoly(camera.image, pts=[], color=(255,255,255)))
        masks.append(cv2.fillPoly(camera.image, pts=[], color=(255,255,255)))
        camera_points = camera.find_cube_points(rows, cols, masks)

        for idx, point in enumerate(camera_points):
            marker = self.create_marker2(point, idx+3000, [0,1,1])
            markers.markers.append(marker)

        marker = self.create_marker2([camera.width,camera.height,0], idx+2000, [1,1,0])
        markers.markers.append(marker)

        cv2.drawChessboardCorners(camera.image, (rows, cols), camera_points, True) # last arg should be ret
        cv2.namedWindow("window_z", cv2.WINDOW_NORMAL) 
        cv2.imshow('window_z', camera.image)
        cv2.waitKey(10)

        ret, r_c_w, T_c_w = cv2.solvePnP(object_points, camera_points, camera.intrinsics, camera.distortion)
        R_c_w, _ = cv2.Rodrigues(r_c_w)

        R_w_c, T_w_c = self.invert_rot_and_pose(R_c_w, T_c_w)

        # if (T_w_c[2] < 0):
            # translation_x = translation_i * np.array([[1],[1],[-1]])
        T_w_c_f = T_w_c * np.array([1,1,-1])
        R_w_c_f = np.array([
                    [-1,0,0],
                    [0,-1,0],
                    [0,0,1]
                ]) @ R_w_c
        
        R_c_w_f, T_c_w_f = self.invert_rot_and_pose(R_w_c_f, T_w_c_f)

        r_c_w_f, _ = cv2.Rodrigues(R_c_w_f)

        marker = self.create_marker(R_c_w, T_c_w, 9000, [1,1,1])
        markers.markers.append(marker)

        marker = self.create_marker(R_w_c, T_w_c, 9001, [.5,0,0])
        markers.markers.append(marker)

        reprojection_points = cv2.projectPoints(object_points, r_c_w_f, T_c_w_f, camera.intrinsics, camera.distortion)[0].reshape(-1,2)
        for idx, point in enumerate(reprojection_points):
            marker = self.create_marker2(point, idx+1000, [1,0,0])
            markers.markers.append(marker)

        print("publshing")
        self.publisher_marker.publish(markers)
        print("DONE")


    def listener_callback(self, msg):
        return 0
    
    def invert_rot_and_pose(self, rot_array, translation):
        pose_a = np.hstack((rot_array, translation.reshape((-1,1))))
        pose_b = np.vstack((pose_a, [[0, 0, 0, 1]]))
        pose_inv = np.linalg.inv(pose_b)
        rot_t_array = pose_inv[:3,:3]
        trans_t = pose_inv[:3,3]
        return (rot_t_array, trans_t)
    
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

        marker.pose.orientation.x = float(quaternion[0])
        marker.pose.orientation.y = float(quaternion[1])
        marker.pose.orientation.z = float(quaternion[2])
        marker.pose.orientation.w = float(quaternion[3])

        return marker
    
    def create_marker2(self, point, id, color):
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

        print('m' + str(id))
        print(point)

        marker.pose.position.x = float(point[0] / 1000)
        marker.pose.position.y = float(point[1] / 1000)
        z = float(point[2]/1000) if point[2] else 0.
        marker.pose.position.z = z

        return marker

class CalibratorObject():

    def __init__(self):
        self.points = self.construct_points_at_origin()
        self.check_points()

    def construct_points_at_origin(self):
        raise NotImplementedError()
    
    def transform_points(self, X_w_p):
        raise NotImplementedError()
    
    def get_points(self):
        return self.points
    
    def check_points(self):
        return True
    
    def grid_size(self):
        return self.rows, self.cols
    
    def grid_xy(self, rows, cols, spacing, margin = 0):
        x = np.arange(rows)
        y = np.arange(cols)
        arr = np.meshgrid(x,y)
        z = arr[0] * 0
        points = np.stack((arr[0], arr[1], z), axis=2) * spacing + [margin, margin, 0]
        return points.reshape((-1,3)).astype('float32')

class Chessboard(CalibratorObject):

    def __init__(self, side_length, rows, cols):
        self.side_length = side_length
        self.rows = rows
        self.cols = cols
        # self.mirror = 'xy'
        super().__init__()

    def construct_points_at_origin(self):
        return self.grid_xy(self.rows, self.cols, self.side_length)

class Chesscube(CalibratorObject):

    def __init__(self, side_length, margin, rows, cols):
        self.side_length = side_length
        self.margin = margin
        self.rows = rows
        self.cols = cols
        super().__init__()

    def construct_points_at_origin(self):
        grid = self.grid_xy(self.rows, self.cols, self.side_length, self.margin)
        R_xz = np.array([[1,0,0],[0,0,1],[0,1,0]])
        R_yz = np.array([[0,1,0],[0,0,1],[1,0,0]])
        return np.concatenate((grid, grid @ R_xz, grid @ R_yz))

class Camera():

    def __init__(self, focal_length, image_path):
        self.load_image(image_path)
        self.distortion = np.array([0.,0.,0.,0.,0.])
        self.intrinsics = np.float64([[focal_length, 0, self.width/2],
                                      [0, focal_length, self.height/2],
                                      [0, 0, 1]])
        
    def load_image(self, image_path):
        self.image = cv2.imread(image_path)
        self.width = self.image.shape[1]
        self.height = self.image.shape[0]

    def find_image_points(self, row, col):
        ret, corners = cv2.findChessboardCorners(self.image, (row, col), None)
        if (ret): return corners.reshape((-1,2))
        ret, corners = cv2.findChessboardCornersSB(self.image, (row, col), None)
        if (ret): return corners.reshape((-1,2))
        raise Exception("Neither method could find the chessboard corners")
    
    def find_cube_points(self, row, col, masks):
        all_points = []
        for mask in masks:
            partial = cv2.bitwise_and(self.image, self.image, mask)
            points = self.find_image_points(partial, row, col)
            all_points.append(points)
        return all_points

    def find_object_in_image(self, object, mask = None):
        return 0

    def project_object_to_image(self, object, camera_translation, camera_rotation):
        return 0
    
    def check_valid_object(self, object):
        return True
    

def main(args=None):
    rclpy.init(args=args)

    camera = Camera(382.77, 'chessboard_top.png')
    # calibrator = Chessboard(10, 6, 7)
    calibrator = Chesscube(20, 30, 6, 6)

    minimal_subscriber = CameraCalibrator(camera, calibrator)
    rclpy.spin(minimal_subscriber)
    minimal_subscriber.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()

if __name__ == '__main__':
    main()