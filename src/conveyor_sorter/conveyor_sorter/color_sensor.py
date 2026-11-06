#!/usr/bin/env python3
"""Virtual colour sensors. Box positions are analytic (feed-forward), so the sensor
simply fires when x(t) enters a sensor window."""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, ColorRGBA
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray

from conveyor_sorter.kinematics import belt_x


class ColorSensor(Node):
    def __init__(self):
        super().__init__('color_sensor')
        self.declare_parameter('sensor1_x', 0.45)
        self.declare_parameter('sensor2_x', 1.45)
        self.declare_parameter('sensor_width', 0.1)
        self.declare_parameter('belt_surface_z', 0.505)
        self.declare_parameter('belt_speed', 0.15)
        self.declare_parameter('update_rate', 50.0)
        g = lambda n: float(self.get_parameter(n).value)  # noqa: E731
        self.sx = {1: g('sensor1_x'), 2: g('sensor2_x')}
        self.width = g('sensor_width')
        self.surface_z = g('belt_surface_z')
        self.speed = g('belt_speed')
        rate = g('update_rate')

        self.det_pub = self.create_publisher(String, '/conveyor/color_detection', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/conveyor/sensor_markers', 10)
        self.create_subscription(String, '/conveyor/box_spawned', self.on_spawn, 10)

        self.boxes = {}
        self.create_timer(1.0 / rate, self.update)
        self.create_timer(0.2, self.publish_markers)
        self.get_logger().info(f'Colour sensors at x={self.sx[1]:.2f} and x={self.sx[2]:.2f}')

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_spawn(self, msg: String):
        try:
            box_id, color, x, y, z, t0 = msg.data.split(',')
            self.boxes[box_id] = dict(color=color, x0=float(x), y=float(y), z=float(z),
                                      t0=float(t0), seen={1: False, 2: False})
        except ValueError as e:
            self.get_logger().warn(f'bad spawn msg: {e}')

    def update(self):
        t = self.now()
        done = []
        for box_id, b in self.boxes.items():
            x = belt_x(b['x0'], b['t0'], self.speed, t)
            for n, sx in self.sx.items():
                if not b['seen'][n] and x >= sx - self.width / 2:
                    b['seen'][n] = True
                    out = String()
                    out.data = f"{box_id},{b['color']},sensor{n},{x:.4f},{b['y']:.4f},{b['z']:.4f}"
                    self.det_pub.publish(out)
            if x > self.sx[2] + 1.0:
                done.append(box_id)
        for box_id in done:
            del self.boxes[box_id]

    def publish_markers(self):
        arr = MarkerArray()
        for n, sx in self.sx.items():
            m = Marker()
            m.header.frame_id = 'world'
            m.ns = 'sensor_zones'
            m.id = n
            m.type = Marker.CUBE
            m.action = Marker.ADD
            m.pose.position = Point(x=sx, y=0.0, z=self.surface_z + 0.1)
            m.pose.orientation.w = 1.0
            m.scale.x, m.scale.y, m.scale.z = self.width, 0.4, 0.2
            m.color = ColorRGBA(r=0.1, g=1.0, b=0.2, a=0.25)
            arr.markers.append(m)
        self.marker_pub.publish(arr)


def main(args=None):
    rclpy.init(args=args)
    node = ColorSensor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
