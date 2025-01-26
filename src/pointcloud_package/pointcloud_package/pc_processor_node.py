import rclpy
from rclpy.node import Node

from sensor_msgs.msg import PointCloud2
import numpy as np

TYPE_MAP = {
    1: np.int8,
    2: np.uint8,
    3: np.int16,
    4: np.uint16,
    5: np.int32,
    6: np.uint32,
    7: np.float32,
    8: np.float64
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
        print(msg.fields)
        print(msg.point_step)
        pc2 = pcToNumpy(msg)
        self.get_logger().info('I heard: "%s"' % msg.data[0:20])
        self.get_logger().info('I heard: "%s"' % msg.fields)


def pcToNumpy(pointcloud2):
    structured_dtype = pc2FieldsToStructuredDtype(pointcloud2.fields)
    # numpy_pc = np.array(pointcloud2.data, np.uint8, copy=False).view(structured_dtype)
    # numpy_pc = np.frombuffer(bytes(my_bytes), structured_dtype)
    # print(structured_dtype)
    return 0


def pc2FieldsToStructuredDtype(fields):
    numpy_dtypes = []

    # type2 = np.dtype({
    #     'names':
    #     'formats':
    #     'offsets':
    #     'itemsize':
    # })

    print(fields)
    for field in fields:
        dtype = (field.name, TYPE_MAP.get(field.datatype))
        numpy_dtypes.append(dtype)
    print(numpy_dtypes)
    return np.dtype(numpy_dtypes)

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
    # my_bytes = [179, 60, 10, 190, 238, 24, 36, 190, 115, 104, 209, 62, 0, 0, 0, 0, 0, 0, 10, 0]
    # dtypes = [('x', '<f4'), ('y', '<f4'), ('z', '<f4'), ('i', '<f4'), ('rgb', '<f4')]
    # # first = np.array(my_bytes)
    # # print(np.array(my_bytes, dtypes))
    # # print(first.view(dtypes))
    # print(np.frombuffer(bytes(my_bytes), dtypes))
    # print(np.fromiter(my_bytes, dtypes))
    # print(np.asarray(my_bytes, dtypes, 'K'))
    # print(np.array(my_bytes, dtype=np.uint8,copy=False).view(dtypes))