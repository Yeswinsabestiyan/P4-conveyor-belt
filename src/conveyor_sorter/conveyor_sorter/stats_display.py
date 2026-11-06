#!/usr/bin/env python3
"""Telemetry HUD: live statistics + arm states as RViz text markers, plus a full
summary printed on Ctrl+C."""

import re
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray

LOG_RE = re.compile(r'(\w+): Box (\S+) \((\w+)\) - (\w+) - (.*)')


class StatsDisplay(Node):
    def __init__(self):
        super().__init__('stats_display')
        self.declare_parameter('update_rate', 10.0)
        self.declare_parameter('text_scale', 0.07)
        rate = float(self.get_parameter('update_rate').value)
        self.scale = float(self.get_parameter('text_scale').value)

        self.stats = dict(spawned=0, red_bin=0, blue_bin=0, scrap_total=0,
                          black_scrap=0, missed=0, wrong_bin=0,
                          arm1_picks=0, arm1_missed=0, arm2_picks=0, arm2_missed=0)
        self.arm_states = {'arm1': 'IDLE', 'arm2': 'IDLE'}

        self.create_subscription(String, '/conveyor/box_spawned', self.on_spawn, 10)
        self.create_subscription(String, '/conveyor/pick_log', self.on_log, 10)
        self.create_subscription(String, '/arm1/state', lambda m: self.arm_states.update(arm1=m.data), 10)
        self.create_subscription(String, '/arm2/state', lambda m: self.arm_states.update(arm2=m.data), 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/conveyor/stats_markers', 10)

        self.start = time.time()
        self.create_timer(1.0 / rate, self.publish_markers)
        self.get_logger().info('Stats display ready')

    # ---------------------------------------------------------------- inputs
    def on_spawn(self, _msg):
        self.stats['spawned'] += 1

    def on_log(self, msg: String):
        m = LOG_RE.search(msg.data)
        if not m:
            return
        who, _box, color, result, details = m.groups()
        s = self.stats
        if result == 'SUCCESS':
            if who == 'arm1':
                s['arm1_picks'] += 1
            elif who == 'arm2':
                s['arm2_picks'] += 1
            if color == 'red' and 'red bin' in details:
                s['red_bin'] += 1
            elif color == 'blue' and 'blue bin' in details:
                s['blue_bin'] += 1
            else:
                s['wrong_bin'] += 1          # must always stay 0
        elif result == 'SCRAP':
            s['scrap_total'] += 1
            if color == 'black':
                s['black_scrap'] += 1        # unhandled by design
            else:
                s['missed'] += 1             # red/blue that no arm managed to sort
                s['arm1_missed' if color == 'red' else 'arm2_missed'] += 1

    # --------------------------------------------------------------- outputs
    def percentages(self):
        n = self.stats['spawned']
        sorted_n = self.stats['red_bin'] + self.stats['blue_bin']
        if n == 0:
            return 0.0, 0.0
        return 100.0 * sorted_n / n, 100.0 * self.stats['missed'] / n

    def text(self):
        s = self.stats
        sp, mp = self.percentages()
        return (f"=== CONVEYOR SORTER ===\n"
                f"Runtime: {time.time() - self.start:.0f}s   Spawned: {s['spawned']}\n"
                f"Red bin: {s['red_bin']}   Blue bin: {s['blue_bin']}   Scrap: {s['scrap_total']}\n"
                f"Sorted: {sp:.1f}%   Missed: {s['missed']} ({mp:.1f}%)\n"
                f"Black (by design): {s['black_scrap']}   Wrong bin: {s['wrong_bin']}\n"
                f"ARM1 (red):  {self.arm_states['arm1']}\n"
                f"ARM2 (blue): {self.arm_states['arm2']}")

    def label(self, arr, mid, text, x, y, z, scale=0.08):
        m = Marker()
        m.header.frame_id = 'world'
        m.ns = 'labels'
        m.id = mid
        m.type = Marker.TEXT_VIEW_FACING
        m.action = Marker.ADD
        m.pose.position = Point(x=x, y=y, z=z)
        m.pose.orientation.w = 1.0
        m.scale.z = scale
        m.color.r = m.color.g = m.color.b = m.color.a = 1.0
        m.text = text
        arr.markers.append(m)

    def publish_markers(self):
        arr = MarkerArray()
        self.label(arr, 0, self.text(), 1.5, 0.0, 1.75, self.scale)
        self.label(arr, 1, 'RED BIN', 0.8, 0.75, 0.42)
        self.label(arr, 2, 'BLUE BIN', 1.8, -0.75, 0.42)
        self.label(arr, 3, 'SCRAP BIN', 3.25, 0.0, 0.42)
        self.label(arr, 4, 'SPAWN', 0.0, 0.0, 0.75)
        self.marker_pub.publish(arr)

    def summary(self):
        s = self.stats
        sp, mp = self.percentages()
        line = '=' * 60
        return '\n'.join([
            '', line, 'CONVEYOR SORTER - SIMULATION SUMMARY', line,
            f"Runtime:             {time.time() - self.start:.1f} s",
            f"Boxes spawned:       {s['spawned']}",
            f"Red bin:             {s['red_bin']}",
            f"Blue bin:            {s['blue_bin']}",
            f"Scrap bin:           {s['scrap_total']} (black by design: {s['black_scrap']}, missed: {s['missed']})",
            f"Sorted:              {sp:.1f}%",
            f"Missed:              {s['missed']} ({mp:.1f}%)",
            f"Wrong-bin sorts:     {s['wrong_bin']} (must be 0)",
            f"Arm1 (red):          picks={s['arm1_picks']}, missed={s['arm1_missed']}",
            f"Arm2 (blue):         picks={s['arm2_picks']}, missed={s['arm2_missed']}",
            line])


def main(args=None):
    rclpy.init(args=args)
    node = StatsDisplay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        print(node.summary(), flush=True)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
