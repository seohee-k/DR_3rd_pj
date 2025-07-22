import rclpy
import DR_init
from rclpy.node import Node
from std_msgs.msg import Header
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from time import sleep



ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
VELOCITY, ACC = 100, 100

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL


def main(args=None):
    rclpy.init(args=args)
    node = PeriodicVisualizer()
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)

    node_ros = rclpy.create_node("doosan_pick_and_place", namespace=ROBOT_ID)
    DR_init.__dsr__node = node_ros

    from DSR_ROBOT2 import (
    move_periodic, DR_TOOL, set_tcp, set_tool
    )
    
    def doosan_motion():
        set_tcp("GripperDA_v1_test")
        set_tool("ToolWeight_test")
        move_periodic(
            amp=[10, 5, 0, 0, 0, 0],
            period=[2.0]*6,
            atime=0.1,
            repeat=5,
            ref=DR_TOOL
        )

    node.set_move_periodic_func(doosan_motion)

    executor.spin()
    node.destroy_node()
    rclpy.shutdown()

class PeriodicVisualizer(Node):
    def __init__(self):
        super().__init__('periodic_marker_node')

        self.marker_pub = self.create_publisher(Marker, '/skeleton_path_marker', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.marker = Marker()
        self.marker.header.frame_id = 'base_link'
        self.marker.ns = 'periodic_path'
        self.marker.id = 0
        self.marker.type = Marker.LINE_STRIP
        self.marker.action = Marker.ADD
        self.marker.scale.x = 0.003
        self.marker.color.a = 1.0
        self.marker.color.r = 0.0
        self.marker.color.g = 1.0
        self.marker.color.b = 0.0
        self.marker.pose.orientation.w = 1.0
        self.marker.points = []

        self.index = 0
        self.executed = False
        self._move_periodic_func = None

    def set_move_periodic_func(self, func):
        self._move_periodic_func = func

    def timer_callback(self):
        if not self.executed and self._move_periodic_func:
            self.get_logger().info("[INFO] move_periodic 시작")
            self._move_periodic_func()
            self.executed = True
            self.get_logger().info("[INFO] move_periodic 실행 완료")

        if self.index < 20:
            t = self.index * 0.1
            px = 0.01 * 10 * t
            py = 0.005 * 10 * t * (1 - t / 2)

            p = Point()
            p.x = px
            p.y = py
            p.z = 0.0
            self.marker.points.append(p)
            self.marker.header.stamp = self.get_clock().now().to_msg()
            self.marker_pub.publish(self.marker)
            self.index += 1

if __name__ == '__main__':
    main()
