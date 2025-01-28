import rclpy
from rclpy.node import Node
from matplotlib import pyplot as plt

from sensor_msgs.msg import PointCloud2, Image
from visualization_msgs.msg import Marker, MarkerArray
from scipy.spatial.transform import Rotation
# from tf2.transformations import quaternion_from_euler
import numpy as np
import math

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

class PointcloudToNormalsConverter(Node):

    def __init__(self):
        super().__init__('pointcloud_to_normals_converter')

        # self.subscription = self.create_subscription(PointCloud2, '/camera/camera/depth/color/points', self.listener_callback, 10)
        self.subscription = self.create_subscription(Image, '/camera/camera/depth/image_rect_raw', self.listener_callback, 10)

        self.publisher = self.create_publisher(MarkerArray, 'normals', 10)

        # self.window_size = 
        # self.stride = 
        # self.row_size = 


    def listener_callback(self, msg):
        # pointcloud = self.pc2ToNumpy(msg)
        # self.publish_message(pointcloud)
        # self.get_logger().info('I heard: "%s"' % pointcloud[0])
        self.get_logger().info("ROW")
        # plt.imshow(self.image_to_numpy(msg), interpolation='nearest')
        # plt.show()

        np_image = self.image_to_numpy(msg)
        markers = self.create_normal_markers(np_image)
        self.publisher.publish(markers)


    # I believe each uint16 represents a distance in mm
    def image_to_numpy(self, image):
        structured_dtype = self.image_to_structured_dtype()
        array_1d = np.array(image.data, np.uint8, copy=False).view(structured_dtype)
        return array_1d.reshape(image.height, image.width)

    def image_to_structured_dtype(self):
        return np.dtype(np.uint16)
    

    # def pc2_to_numpy(self, pointcloud2):
    #     structured_dtype = self.pc2ToStructuredDtype(pointcloud2)
    #     return np.array(pointcloud2.data, np.uint8, copy=False).view(structured_dtype)
    #     # numpy_pc = np.frombuffer(bytes(my_bytes), structured_dtype)

    # def pc2_to_structured_dtype(self, pointcloud2):
    #     fields = pointcloud2.fields
    #     big_endian = ">" if pointcloud2.is_bigendian else "<"
    #     return np.dtype({
    #         'names': [f.name for f in fields],
    #         'formats': [big_endian + TYPE_MAP.get(f.datatype) for f in fields],
    #         'offsets': [f.offset for f in fields],
    #         'itemsize':  pointcloud2.point_step
    #     })


    # def publish_message(self, pointcloud):
    #     msg = self.calculate_normals(pointcloud)
    #     self.publisher_.publish(msg)

    # assumes plumb bob distortion (aka no distortion) other than scaling and translating
    # https://calib.io/blogs/knowledge-base/camera-models?srsltid=AfmBOoricX292iNdbQf7ZCepJyz20adlV-7n84QJZAOcHu1iRIO3lVou
    def projectionToReal(self, x, y, depth):
        camera_projection = ((x-320)/640*depth, (y-240)/480*depth, 1*depth)
        return camera_projection
    
    # Inspired by MIT Robot Manipulation Chapter 5
    def estimate_normals_by_nearest_pixels(self, image, window_size, stride):
        
        num_rows = image.shape[0] # 480
        num_cols = image.shape[1] # 640

        # marker_array = MarkerArray()
        normals = []
        min_row, max_row, min_col, max_col  = self.bbox(image)

        for col in range(min_col, max_col, stride):
            for row in range(min_row, max_row, stride):

                x, y, z = self.projectionToReal(col, row, image[row][col])

                col_window_range = np.arange(max(col - window_size, 0), min(col + window_size + 1, num_cols - 1))
                row_window_range = np.arange(max(row - window_size, 0), min(row + window_size + 1, num_rows - 1))

                if (col_window_range.size == 0 or row_window_range.size == 0): continue

                total_depth, total_x, total_y = (0, 0, 0)
                total = 0
                for wcol in col_window_range:
                    for wrow in row_window_range:
                        xw, yw, zw = self.projectionToReal(wcol, wrow, image[wrow][wcol])
                        total_depth += zw
                        total_x += xw
                        total_y += yw
                        total += 1
                avg_depth = total_depth / total
                avg_x = total_x / total
                avg_y = total_y / total

                W = np.zeros((3,3))
                for wcol in col_window_range:
                    for wrow in row_window_range:
                        xw, yw, zw = self.projectionToReal(wcol, wrow, image[wrow][wcol])
                        diff = np.array([xw-avg_x, yw-avg_y, zw-avg_depth])
                        W += np.outer(diff, diff)

                eigen_info = np.linalg.eigh(W)
                # print(eigen_info)
                # print(eigen_info.eigenvectors[0])
                normal = eigen_info.eigenvectors[:, 0]
                center_point = (x, y, z)
                normals.append((normal, center_point))

        # # pointcloud.reshape
        # for pixel in image:
        #     # mark = self.create_marker()
        #     normals.append(normal)
        #     # marker_array.markers.append(mark)

        return normals
        
    def bbox(self, np_image):
        mask = np.where(np_image != 0)
        top_right = np.min(mask, 1)
        bottom_left = np.max(mask, 1)
        return (top_right[0], bottom_left[0], top_right[1], bottom_left[1])

    def create_normal_markers(self, image):
        normals = self.estimate_normals_by_nearest_pixels(image, 2, 10)
        markers = MarkerArray()
        id = 0
        for vector, point in normals:
            marker = self.create_marker(point, vector, id)
            markers.markers.append(marker)
            id+=1
        return markers

    def create_marker(self, point, vector, id):
        marker = Marker()
        marker.header.frame_id = '/camera_depth_optical_frame'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.type = marker.ARROW
        marker.id = id
        marker.action = marker.ADD
        marker.scale.x = .1
        marker.scale.y = 0.005
        marker.scale.z = 0.005
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 1.0

        # marker.pose.position.x = float(vector[0])
        # marker.pose.position.y = float(vector[1])
        # marker.pose.position.z = float(vector[2])

        marker.pose.position.x = float(point[0] / 400)
        marker.pose.position.y = float(point[1] / 400)
        marker.pose.position.z = float(point[2] / 400)
        quaternion = Rotation.align_vectors([vector], [[1,0,0]])[0].as_quat()

        # print("_")
        # print(point)
        # print(vector)
        # print(quaternion)

        # # quaternion = Rotation.align_vectors([[1,0,0]], [vector])[0].as_quat()
        # # quaternion = Rotation.align_vectors([[1,0,0]], [[0,0,1]])[0].as_quat()
        # # quaternion = Rotation.from_euler('xyz', vector, degrees = False).as_quat()
        # # quaternion = Rotation.from_euler('xyz', [0,1.51,0], degrees = False).as_quat()

        # euler = Rotation.align_vectors([[1,0,0]], [vector])[0].as_euler('xyz', degrees=False)

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

# from tf2_geometry_msgs import quaternion_from_euler




if __name__ == '__main__':
    main()

    # # print(Rotation.align_vectors([[1,0,0]], [[0,1,0]])[0].as_euler('xyz', degrees=True).as_quat())
    # a = Rotation.align_vectors([[0,0,1]], [[1,0,0]])[0]
    # e = a.as_euler('xyz', degrees=True)
    # q = a.as_quat()
    # m = a.as_matrix()
    # q_to_e = Rotation.from_euler('xyz', e, degrees=True).as_quat()
    # m_to_e = Rotation.from_matrix(m).as_quat()
    # print(a)
    # print(e)
    # print(q)
    # print(q_to_e)
    # print(m_to_e)

    # print(a.apply([1,0,0]))
    # # print(Rotation.align_vectors([[1,.21, .123]], [[.34, .23, .12]])[0].as_quat())
    # # print(Rotation.from_euler('xyz', [1,.21, .123], degrees = False).as_quat())

