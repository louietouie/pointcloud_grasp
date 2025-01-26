import rclpy
from rclpy.node import Node

from sensor_msgs.msg import PointCloud2
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

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            PointCloud2,
            '/camera/camera/depth/color/points',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        pointcloud = pc2ToNumpy(msg)
        self.get_logger().info('I heard: "%s"' % pointcloud[0])

def pc2ToNumpy(pointcloud2):
    structured_dtype = pc2ToStructuredDtype(pointcloud2)
    return np.array(pointcloud2.data, np.uint8, copy=False).view(structured_dtype)
    # numpy_pc = np.frombuffer(bytes(my_bytes), structured_dtype)

def pc2ToStructuredDtype(pointcloud2):
    fields = pointcloud2.fields
    big_endian = ">" if pointcloud2.is_bigendian else "<"
    return np.dtype({
        'names': [f.name for f in fields],
        'formats': [big_endian + TYPE_MAP.get(f.datatype) for f in fields],
        'offsets': [f.offset for f in fields],
        'itemsize':  pointcloud2.point_step
    })

def pcToNormals(pointcloud):

    return 0

def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()