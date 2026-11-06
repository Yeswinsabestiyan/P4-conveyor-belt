#!/usr/bin/env python3
"""Box spawner + box physics-lite.

Owns every box's pose and publishes them as RViz markers.
Boxes are in one of: BELT (analytic x = x0 + v*(t-t0)), CARRIED (pose comes from the
arm gripper via /conveyor/carry), FALLING (drops into a bin) or BINNED.
"""

import random
import re
import time
from enum import Enum

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, ColorRGBA
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray

from conveyor_sorter.kinematics import belt_x

LOG_RE = re.compile(r'Box (\S+) \((\w+)\) - (\w+) - (.*)')


class BoxColor(Enum):
    RED = 'red'
    BLUE = 'blue'
    BLACK = 'black'


COLORS = {
    'red': ColorRGBA(r=0.9, g=0.05, b=0.05, a=1.0),
    'blue': ColorRGBA(r=0.1, g=0.2, b=0.95, a=1.0),
    'black': ColorRGBA(r=0.04, g=0.04, b=0.04, a=1.0),
}


class Box:
    def __init__(self, num, box_id, color, x0, y, z, t0):
        self.num = num              # integer marker id (unique, never reused)
        self.id = box_id
        self.color = color
        self.x0, self.t0 = x0, t0
        self.x, self.y, self.z = x0, y, z
        self.state = 'BELT'
        self.vz = 0.0
        self.target = None          # (x, y, z) when FALLING
        self.bin = None             # 'red' | 'blue' | 'scrap'
        self.last_carry = 0.0


class BoxSpawner(Node):
    def __init__(self):
        super().__init__('box_spawner')
        d = self.declare_parameter
        d('spawn_rate', 0.5)
        d('min_spawn_interval', 1.2)
        d('max_spawn_interval', 3.0)
        d('red_prob', 0.35)
        d('blue_prob', 0.35)
        d('black_prob', 0.30)
        d('box_size', 0.1)
        d('spawn_x', 0.0)
        d('spawn_y', 0.0)
        d('spawn_z', 0.555)
        d('belt_speed', 0.15)
        d('belt_length', 3.0)
        d('red_bin_x', 0.8)
        d('red_bin_y', 0.75)
        d('blue_bin_x', 1.8)
        d('blue_bin_y', -0.75)
        d('scrap_bin_x', 3.25)
        d('scrap_bin_y', 0.0)
        d('bin_floor_z', 0.04)
        d('max_boxes_per_bin', 18)
        p = lambda n: self.get_parameter(n).value  # noqa: E731
        self.rate = float(p('spawn_rate'))
        self.min_iv = float(p('min_spawn_interval'))
        self.max_iv = float(p('max_spawn_interval'))
        self.box_size = float(p('box_size'))
        self.spawn_x, self.spawn_y, self.spawn_z = (float(p('spawn_x')), float(p('spawn_y')),
                                                    float(p('spawn_z')))
        self.speed = float(p('belt_speed'))
        self.belt_length = float(p('belt_length'))
        self.bins = {
            'red': (float(p('red_bin_x')), float(p('red_bin_y'))),
            'blue': (float(p('blue_bin_x')), float(p('blue_bin_y'))),
            'scrap': (float(p('scrap_bin_x')), float(p('scrap_bin_y'))),
        }
        self.bin_floor_z = float(p('bin_floor_z'))
        self.max_per_bin = int(p('max_boxes_per_bin'))

        # Never let boxes overlap on the belt
        min_gap_iv = (self.box_size * 1.8) / max(self.speed, 1e-6)
        self.min_iv = max(self.min_iv, min_gap_iv)
        self.max_iv = max(self.max_iv, self.min_iv + 0.1)

        probs = [float(p('red_prob')), float(p('blue_prob')), float(p('black_prob'))]
        total = sum(probs) or 1.0
        self.probs = [v / total for v in probs]

        self.marker_pub = self.create_publisher(MarkerArray, '/conveyor/boxes_markers', 10)
        self.info_pub = self.create_publisher(String, '/conveyor/box_spawned', 10)
        self.log_pub = self.create_publisher(String, '/conveyor/pick_log', 10)
        self.create_subscription(String, '/conveyor/pick_log', self.on_pick_log, 10)
        self.create_subscription(String, '/conveyor/carry', self.on_carry, 10)

        self.boxes = {}
        self.bin_counts = {'red': 0, 'blue': 0, 'scrap': 0}
        self.bin_members = {'red': [], 'blue': [], 'scrap': []}
        self.pending_delete = []
        self.counter = 0
        self.total_spawned = 0
        self.first_publish = True

        self.last_tick = self.now()
        self.next_spawn = self.now() + 1.0
        self.create_timer(1.0 / 30.0, self.update)
        self.get_logger().info(
            f'Box spawner ready: belt {self.speed} m/s, spawn every '
            f'{self.min_iv:.1f}-{self.max_iv:.1f}s')

    # ------------------------------------------------------------------ utils
    def now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def log(self, box, result, details):
        msg = String()
        msg.data = (f'[{time.strftime("%H:%M:%S")}] conveyor: Box {box.id} '
                    f'({box.color}) - {result} - {details}')
        self.log_pub.publish(msg)
        self.get_logger().info(msg.data)

    def pick_color(self) -> str:
        r = random.random()
        if r < self.probs[0]:
            return 'red'
        if r < self.probs[0] + self.probs[1]:
            return 'blue'
        return 'black'

    # ------------------------------------------------------------- callbacks
    def on_carry(self, msg: String):
        try:
            box_id, x, y, z = msg.data.split(',')
            box = self.boxes.get(box_id)
            if box and box.state in ('BELT', 'CARRIED'):
                box.state = 'CARRIED'
                box.x, box.y, box.z = float(x), float(y), float(z)
                box.last_carry = self.now()
        except ValueError:
            pass

    def on_pick_log(self, msg: String):
        m = LOG_RE.search(msg.data)
        if not m:
            return
        box_id, _color, result, details = m.groups()
        box = self.boxes.get(box_id)
        if box is None:
            return
        if result == 'SUCCESS':
            which = 'red' if 'red bin' in details else 'blue' if 'blue bin' in details else None
            if which:
                self.drop_into_bin(box, which)
        elif result == 'ABORTED' and box.state == 'CARRIED':
            # arm gave up while holding it: let it fall to scrap
            self.drop_into_bin(box, 'scrap')

    # ------------------------------------------------------------------ logic
    def slot_position(self, which):
        n = self.bin_counts[which]
        self.bin_counts[which] += 1
        gx, gy = n % 3, (n // 3) % 3
        layer = n // 9
        bx, by = self.bins[which]
        return (bx + (gx - 1) * 0.12, by + (gy - 1) * 0.12,
                self.bin_floor_z + self.box_size / 2 + layer * (self.box_size + 0.005))

    def drop_into_bin(self, box, which):
        box.state = 'FALLING'
        box.bin = which
        box.vz = 0.0
        box.target = self.slot_position(which)

    def spawn_box(self):
        color = self.pick_color()
        t = self.now()
        box_id = f'b{self.counter:04d}'
        box = Box(self.counter, box_id, color, self.spawn_x, self.spawn_y, self.spawn_z, t)
        self.counter += 1
        self.total_spawned += 1
        self.boxes[box_id] = box
        info = String()
        info.data = (f'{box_id},{color},{self.spawn_x:.4f},{self.spawn_y:.4f},'
                     f'{self.spawn_z:.4f},{t:.6f}')
        self.info_pub.publish(info)
        self.get_logger().info(f'Spawned {color} box {box_id}')

    def update(self):
        t = self.now()
        dt = max(0.0, min(0.1, t - self.last_tick))
        self.last_tick = t

        if t >= self.next_spawn:
            self.spawn_box()
            iv = random.uniform(0.6, 1.4) / max(self.rate, 1e-6)
            self.next_spawn = t + max(self.min_iv, min(self.max_iv, iv))

        for box in list(self.boxes.values()):
            if box.state == 'BELT':
                box.x = belt_x(box.x0, box.t0, self.speed, t)
                if box.x >= self.belt_length:
                    self.log(box, 'SCRAP', 'Reached end of belt')
                    self.drop_into_bin(box, 'scrap')
            elif box.state == 'CARRIED':
                if t - box.last_carry > 1.0:     # arm vanished; resume on belt
                    box.state = 'BELT'
            elif box.state == 'FALLING':
                tx, ty, tz = box.target
                step = 0.6 * dt
                box.x += max(-step, min(step, tx - box.x))
                box.y += max(-step, min(step, ty - box.y))
                box.vz -= 9.81 * dt
                box.z += box.vz * dt
                if box.z <= tz:
                    box.z = tz
                    box.x, box.y = tx, ty
                    box.state = 'BINNED'
                    self.bin_members[box.bin].append(box.id)
                    if len(self.bin_members[box.bin]) > self.max_per_bin:
                        old = self.bin_members[box.bin].pop(0)
                        self.remove_box(old)
        self.publish_markers()

    def remove_box(self, box_id):
        box = self.boxes.pop(box_id, None)
        if box:
            self.pending_delete.append(box.num)

    def publish_markers(self):
        arr = MarkerArray()
        if self.first_publish:
            clear = Marker()
            clear.action = Marker.DELETEALL
            clear.ns = 'boxes'
            arr.markers.append(clear)
            self.first_publish = False
        for num in self.pending_delete:
            m = Marker()
            m.header.frame_id = 'world'
            m.ns = 'boxes'
            m.id = num
            m.action = Marker.DELETE
            arr.markers.append(m)
        self.pending_delete = []
        for box in self.boxes.values():
            m = Marker()
            m.header.frame_id = 'world'          # stamp 0 -> always "latest"
            m.ns = 'boxes'
            m.id = box.num
            m.type = Marker.CUBE
            m.action = Marker.ADD
            m.pose.position = Point(x=box.x, y=box.y, z=box.z)
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = self.box_size
            m.color = COLORS[box.color]
            arr.markers.append(m)
        self.marker_pub.publish(arr)


def main(args=None):
    rclpy.init(args=args)
    node = BoxSpawner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info(f'Box spawner shutting down. Total spawned: {node.total_spawned}')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
