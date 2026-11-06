#!/usr/bin/env python3
"""Conveyor node: publishes belt state/speed and a scrolling stripe pattern so the
belt visibly moves in RViz. (Box motion itself is analytic: x = x0 + v*(t - t0).)"""

import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float64, ColorRGBA
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray


class Conveyor(Node):
    def __init__(self):
        super().__init__('conveyor')
        self.declare_parameter('belt_speed', 0.15)
        self.declare_parameter('belt_length', 3.0)
        self.declare_parameter('belt_width', 0.4)
        self.declare_parameter('belt_ground_clearance', 0.5)
        self.declare_parameter('update_rate', 20.0)
        self.speed = float(self.get_parameter('belt_speed').value)
        self.length = float(self.get_parameter('belt_length').value)
        self.width = float(self.get_parameter('belt_width').value)
        self.top_z = float(self.get_parameter('belt_ground_clearance').value) + 0.006
        rate = float(self.get_parameter('update_rate').value)

        self.state_pub = self.create_publisher(String, '/conveyor/state', 10)
        self.speed_pub = self.create_publisher(Float64, '/conveyor/belt_speed', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/conveyor/belt_markers', 10)

        self.pitch = 0.3
        self.t_start = self.now()
        self.create_timer(1.0 / rate, self.update)
        self.create_timer(1.0, self.publish_state)
        self.get_logger().info(f'Conveyor started: {self.speed} m/s, {self.length} m long')

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def publish_state(self):
        s = String()
        s.data = 'RUNNING'
        self.state_pub.publish(s)
        f = Float64()
        f.data = self.speed
        self.speed_pub.publish(f)

    def update(self):
        offset = (self.speed * (self.now() - self.t_start)) % self.pitch
        arr = MarkerArray()
        m = Marker()
        m.header.frame_id = 'world'
        m.ns = 'belt_stripes'
        m.id = 0
        m.type = Marker.CUBE_LIST
        m.action = Marker.ADD
        m.pose.orientation.w = 1.0
        m.scale.x, m.scale.y, m.scale.z = 0.03, self.width - 0.06, 0.004
        m.color = ColorRGBA(r=0.95, g=0.8, b=0.1, a=1.0)
        n = int(math.ceil(self.length / self.pitch)) + 1
        for i in range(n):
            x = offset + i * self.pitch
            if 0.0 <= x <= self.length:
                m.points.append(Point(x=x, y=0.0, z=self.top_z))
        arr.markers.append(m)
        self.marker_pub.publish(arr)


def main(args=None):
    rclpy.init(args=args)
    node = Conveyor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info('Conveyor shutting down')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
