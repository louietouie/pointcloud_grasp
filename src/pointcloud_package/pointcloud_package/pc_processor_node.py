import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image # PointCloud2
from visualization_msgs.msg import Marker, MarkerArray
from scipy.spatial.transform import Rotation
import numpy as np

TYPE_MAP = {
    1: 'i1', # np.int8
    2: 'u1', # np.uint8
    3: 'i2', # np.int16
    4: 'u2', # np.uint16
    5: 'i4', # np.int32
    6: 'u4', # np.uint32
    7: 'f4', # np.float32
    8: 'f8', # np.float64
}

# NOTE: https://calib.io/blogs/knowledge-base/camera-models?srsltid=AfmBOoricX292iNdbQf7ZCepJyz20adlV-7n84QJZAOcHu1iRIO3lVou
# NOTE: https://dev.intelrealsense.com/docs/projection-texture-mapping-and-occlusion-with-intel-realsense-depth-cameras

class PointcloudToNormalsConverter(Node):

    def __init__(self):
        super().__init__('pointcloud_to_normals_converter')

        self.subscription = self.create_subscription(Image, '/camera/camera/depth/image_rect_raw', self.listener_callback, 10)
        self.publisher = self.create_publisher(MarkerArray, 'normals', 10)

    def listener_callback(self, msg):
        np_image = self.image_to_numpy(msg)
        markers = self.create_normal_markers(np_image)
        self.publisher.publish(markers)

    def image_to_numpy(self, image):
        structured_dtype = self.image_to_structured_dtype()
        array_1d = np.array(image.data, np.uint8, copy=False).view(structured_dtype)
        return array_1d.reshape(image.height, image.width)

    def image_to_structured_dtype(self):
        return np.dtype(np.uint16) # I believe each uint16 represents a distance in mm

    def create_normal_markers(self, image):
        normals = self.estimate_normals_by_nearest_pixels(image, 4, 10)
        markers = MarkerArray()
        for idx, (vector, point) in enumerate(normals):
            marker = self.create_marker(vector, point, idx)
            markers.markers.append(marker)
        return markers
    
    # Inspired by MIT Robot Manipulation Chapter 5
    def estimate_normals_by_nearest_pixels(self, image, window_size, stride):

        def deproject_image(depth_image):
            
            num_rows = depth_image.shape[0] # 480
            num_cols = depth_image.shape[1] # 640

            row_idx, col_idx = np.meshgrid(np.arange(num_cols),np.arange(num_rows))
            position_image = np.stack((row_idx, col_idx, image), axis = 2)

            principal_point = np.array([640/2, 480/2, 0])
            focal_length = np.array([382.77, 382.77, 1])
            depth = position_image[:, :, 2:]
            ones = np.full((position_image.shape[0], position_image.shape[1], 1), 1)
            depth_scale = np.concatenate((depth, depth,ones), axis=2)
            # rs-enumerate-devices -c... uses Brown Conrady but k1-k5 are all 0, so no distortion matrix is needed
            deprojected_unscaled_image = (position_image - principal_point) / focal_length
            deprojected_image = np.multiply(deprojected_unscaled_image, depth_scale)

            return deprojected_image

        normals = []
        min_row, max_row, min_col, max_col  = self.bbox(image)
        deprojected_image = deproject_image(image)
        integral_image = deprojected_image.cumsum(axis=0).cumsum(axis=1)

        # TODO: replace with numpy.lib.stride_tricks.sliding_window_view?
        for col in range(min_col, max_col, stride):
            for row in range(min_row, max_row, stride):

                x, y, z = deprojected_image[row][col]

                wndw_min_c = max(col - window_size, 0)
                wndw_max_c = min(col + window_size + 1, max_col - 1)
                wndw_min_r = max(row - window_size, 0)
                wndw_max_r = min(row + window_size + 1, max_row - 1)

                col_window_range = np.arange(wndw_min_c, wndw_max_c)
                row_window_range = np.arange(wndw_min_r, wndw_max_r)

                if (col_window_range.size == 0 or row_window_range.size == 0): continue

                window_actual_size = (wndw_max_c - wndw_min_c) * (wndw_max_r - wndw_min_r)
                avg_xyd = (integral_image[wndw_min_r][wndw_min_c] + integral_image[wndw_max_r][wndw_max_c] - integral_image[wndw_min_r][wndw_max_c] - integral_image[wndw_max_r][wndw_min_c]) / window_actual_size
                
                points_in_window = deprojected_image[wndw_min_r:wndw_max_r, wndw_min_c:wndw_max_c].reshape((-1,3))
                diff = points_in_window - avg_xyd
                W = diff.T @ diff

                eigen_info = np.linalg.eigh(W)
                normal = eigen_info.eigenvectors[:, 0]
                normal_f = self.flip_vector_towards_camera(normal)
                center_point = (x, y, z)
                normals.append((normal_f, center_point))

        return normals
        
    def bbox(self, np_image):
        mask = np.where(np_image != 0)
        top_right = np.min(mask, 1)
        bottom_left = np.max(mask, 1)
        return (top_right[0], bottom_left[0], top_right[1], bottom_left[1])

    def flip_vector_towards_camera(self, vector):
        camera_dir = np.array([0,0,1])
        vector_dir = np.array(vector)
        if camera_dir.dot(vector_dir) > 0:
            return -vector
        return vector

    def create_marker(self, vector, point, id):
        marker = Marker()
        marker.header.frame_id = '/camera_depth_optical_frame'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.type = marker.ARROW
        marker.id = id
        marker.action = marker.ADD
        
        marker.scale.x = .02
        marker.scale.y = 0.002
        marker.scale.z = 0.002
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        marker.pose.position.x = float(point[0] / 1000)
        marker.pose.position.y = float(point[1] / 1000)
        marker.pose.position.z = float(point[2] / 1000)
        quaternion = Rotation.align_vectors([vector], [[1,0,0]])[0].as_quat()

        marker.pose.orientation.x = float(quaternion[0])
        marker.pose.orientation.y = float(quaternion[1])
        marker.pose.orientation.z = float(quaternion[2])
        marker.pose.orientation.w = float(quaternion[3])

        return marker


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = PointcloudToNormalsConverter()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()