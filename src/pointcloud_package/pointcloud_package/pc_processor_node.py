import rclpy
from rclpy.node import Node

from sensor_msgs.msg import PointCloud2, Image
from visualization_msgs.msg import Marker, MarkerArray
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

class PointcloudToNormalsConverter(Node):

    def __init__(self):
        super().__init__('pointcloud_to_normals_converter')

        # self.subscription = self.create_subscription(PointCloud2, '/camera/camera/depth/color/points', self.listener_callback, 10)
        self.subscription = self.create_subscription(Image, '/camera/camera/depth/image_rect_raw', self.listener_callback, 10)

        # self.publisher = self.create_publisher(MarkerArray, 'pointcloud_normals', 10)

        # self.window_size = 
        # self.stride = 
        # self.row_size = 


    def listener_callback(self, msg):
        # pointcloud = self.pc2ToNumpy(msg)
        # self.publish_message(pointcloud)
        # self.get_logger().info('I heard: "%s"' % pointcloud[0])
        self.get_logger().info("ROW")
        # self.get_logger().info("%s" % msg.height)
        # self.get_logger().info("%s" % msg.width)
        # self.get_logger().info("%s" % msg.row_step)
        # self.get_logger().info("%s" % len(msg.data))

        self.get_logger().info("%s" % msg.encoding)
        self.get_logger().info("%s" % msg.height)
        self.get_logger().info("%s" % msg.width)
        self.get_logger().info("%s" % msg.step)
        self.get_logger().info("%s" % len(msg.data))
        self.get_logger().info("%s" % self.image_to_numpy(msg)[0])

        # self.get_logger().info('I heard: "%s"' % pointcloud[0])

    def image_to_numpy(self, image):
        structured_dtype = self.image_to_structured_dtype()
        array_1d = np.array(image.data, np.uint8, copy=False).view(structured_dtype)
        return array_1d.reshape(image.width, image.height)

    def image_to_structured_dtype(self):
        return np.dtype(np.uint16)
    
    def pc2_to_numpy(self, pointcloud2):
        structured_dtype = self.pc2ToStructuredDtype(pointcloud2)
        return np.array(pointcloud2.data, np.uint8, copy=False).view(structured_dtype)
        # numpy_pc = np.frombuffer(bytes(my_bytes), structured_dtype)

    def pc2_to_structured_dtype(self, pointcloud2):
        fields = pointcloud2.fields
        big_endian = ">" if pointcloud2.is_bigendian else "<"
        return np.dtype({
            'names': [f.name for f in fields],
            'formats': [big_endian + TYPE_MAP.get(f.datatype) for f in fields],
            'offsets': [f.offset for f in fields],
            'itemsize':  pointcloud2.point_step
        })


    def publish_message(self, pointcloud):
        msg = self.calculate_normals(pointcloud)
        self.publisher_.publish(msg)

    
    # Inspired by MIT Robot Manipulation Chapter 5
    def estimate_normals_by_nearest_pixels(self, pointcloud):
        marker_array = MarkerArray()
        # pointcloud.reshape
        for point in pointcloud:
            mark = self.create_marker()
            marker_array.markers.append(mark)
        return marker_array
        
    def create_marker():
        marker = Marker()
        marker.header.frame_id = '/map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.type = marker.SPHERE
        marker.id = 0
        marker.action = marker.ADD
        marker.scale.x = 0.5
        marker.scale.y = 0.5
        marker.scale.z = 0.5
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 2.0
        return marker


def pcToNormals(pointcloud):

    return 0

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