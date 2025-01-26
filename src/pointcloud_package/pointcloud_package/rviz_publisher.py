import rclpy
from rclpy.node import Node

from visualization_msgs.msg import Marker, MarkerArray
import numpy as np

class RVizArrowPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(MarkerArray, 'pointcloud_normals', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = self.sampleMarkers()
        # msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg)
        self.i += 1

    def sampleMarkers(self):
        marker_array = MarkerArray()
        mark = self.create_marker()
        marker_array.markers.append(mark)
        return marker_array

    def create_marker(self):
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



def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = RVizArrowPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()