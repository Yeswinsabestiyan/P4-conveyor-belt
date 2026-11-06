#!/usr/bin/env python3
"""Arm controller: IDLE -> DETECT -> REACH -> GRASP -> LIFT -> MOVE_TO_BIN -> RELEASE -> RETURN

* Moving-target interception: the belt never stops, so REACH aims at where the box WILL
  be at t_now + reach_time (feed-forward), and GRASP then tracks the live box position.
* Exclusive reservation: a box is CLAIMED (published on /conveyor/pick_log) before the arm
  moves; other arms ignore claimed ids.
* Only the arm's own colour is ever picked, so a wrong-bin sort cannot happen.
* While holding a box the arm publishes its tool-tip pose on /conveyor/carry; the spawner
  attaches the box marker to it.
"""

import time
from enum import Enum

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Point
from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker, MarkerArray

from conveyor_sorter.kinematics import ArmGeometry, fk, ik, belt_x, smoothstep

import re

LOG_RE = re.compile(r'(\w+): Box (\S+) \((\w+)\) - (\w+) - (.*)')
GRIPPER_OPEN = 0.03
GRIPPER_CLOSED = 0.0


class ArmState(Enum):
    IDLE = 'IDLE'
    DETECT = 'DETECT'
    REACH = 'REACH'
    GRASP = 'GRASP'
    LIFT = 'LIFT'
    MOVE_TO_BIN = 'MOVE_TO_BIN'
    RELEASE = 'RELEASE'
    RETURN = 'RETURN'


class ArmController(Node):
    def __init__(self):
        super().__init__('arm_controller')
        d = self.declare_parameter
        d('arm_name', 'arm1')
        d('target_color', 'red')
        d('sensor_num', 1)
        d('base_x', 0.8)
        d('base_y', 0.35)
        d('base_z', 0.5)
        d('shoulder_h', 0.25)
        d('link2_length', 0.45)
        d('link3_length', 0.42)
        d('bin_x', 0.8)
        d('bin_y', 0.75)
        d('drop_z', 0.50)
        d('box_z', 0.555)
        d('hover', 0.12)
        d('reach_time', 1.0)
        d('descend_time', 0.5)
        d('place_time', 1.3)
        d('return_time', 1.5)
        d('home_joints', [0.0, -0.9, 1.5])
        d('belt_speed', 0.15)
        d('update_rate', 50.0)
        p = lambda n: self.get_parameter(n).value  # noqa: E731
        self.name = str(p('arm_name'))
        self.color = str(p('target_color'))
        self.sensor_num = int(p('sensor_num'))
        self.geo = ArmGeometry(float(p('base_x')), float(p('base_y')), float(p('base_z')),
                               float(p('shoulder_h')), float(p('link2_length')),
                               float(p('link3_length')))
        self.bin_xy = (float(p('bin_x')), float(p('bin_y')))
        self.drop_z = float(p('drop_z'))
        self.box_z = float(p('box_z'))
        self.hover = float(p('hover'))
        self.reach_time = float(p('reach_time'))
        self.descend_time = float(p('descend_time'))
        self.place_time = float(p('place_time'))
        self.return_time = float(p('return_time'))
        self.home = [float(v) for v in p('home_joints')]
        self.speed = float(p('belt_speed'))
        rate = float(p('update_rate'))

        self.js_pub = self.create_publisher(JointState, f'/{self.name}/joint_states', 10)
        self.state_pub = self.create_publisher(String, f'/{self.name}/state', 10)
        self.marker_pub = self.create_publisher(MarkerArray, f'/{self.name}/markers', 10)
        self.log_pub = self.create_publisher(String, '/conveyor/pick_log', 10)
        self.carry_pub = self.create_publisher(String, '/conveyor/carry', 10)
        self.create_subscription(String, '/conveyor/box_spawned', self.on_spawn, 10)
        self.create_subscription(String, '/conveyor/color_detection', self.on_detect, 10)
        self.create_subscription(String, '/conveyor/pick_log', self.on_pick_log, 10)

        self.boxes = {}          # id -> dict(color, x0, y, z, t0)
        self.detected = {}       # id -> sensor number
        self.taken = set()       # ids claimed by another arm or already handled
        self.target_id = None
        self.carrying = False

        self.state = ArmState.IDLE
        self.state_t0 = self.now()
        self.q = list(self.home)
        self.traj = None         # (t0, T, q_start, q_goal)
        self.gripper = GRIPPER_OPEN
        self.gripper_goal = GRIPPER_OPEN
        self.last_tick = self.now()
        self.released = False

        self.stats = dict(attempted=0, successful=0, aborted=0)

        self.create_timer(1.0 / rate, self.update)
        self.create_timer(0.1, self.publish_state)
        self.get_logger().info(
            f'{self.name} ready: picks {self.color} boxes from sensor {self.sensor_num}, '
            f'base=({self.geo.base_x},{self.geo.base_y}), reach={self.geo.max_reach:.2f} m')

    # ------------------------------------------------------------------ utils
    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def box_xyz(self, box, t=None):
        t = self.now() if t is None else t
        return belt_x(box['x0'], box['t0'], self.speed, t), box['y'], self.box_z

    def log_pick(self, box_id, result, details):
        msg = String()
        msg.data = (f'[{time.strftime("%H:%M:%S")}] {self.name}: Box {box_id} '
                    f'({self.color}) - {result} - {details}')
        self.log_pub.publish(msg)
        self.get_logger().info(msg.data)

    def go(self, state: ArmState):
        self.state = state
        self.state_t0 = self.now()
        self.publish_state()

    def start_traj(self, goal, duration):
        self.traj = (self.now(), max(duration, 1e-3), list(self.q), list(goal))

    # ------------------------------------------------------------- callbacks
    def on_spawn(self, msg: String):
        try:
            box_id, color, x, y, z, t0 = msg.data.split(',')
            self.boxes[box_id] = dict(color=color, x0=float(x), y=float(y), z=float(z),
                                      t0=float(t0))
        except ValueError as e:
            self.get_logger().warn(f'bad spawn msg: {e}')

    def on_detect(self, msg: String):
        try:
            box_id, color, sensor, *_ = msg.data.split(',')
            n = int(sensor.replace('sensor', ''))
            if box_id not in self.boxes:      # detection arrived before spawn msg (unlikely)
                return
            self.detected[box_id] = n
        except ValueError as e:
            self.get_logger().warn(f'bad detect msg: {e}')

    def on_pick_log(self, msg: String):
        m = LOG_RE.search(msg.data)
        if m and m.group(4) == 'CLAIMED' and m.group(1) != self.name:
            self.taken.add(m.group(2))

    # ------------------------------------------------------------- main loop
    def update(self):
        t = self.now()
        dt = max(0.0, t - self.last_tick)
        self.last_tick = t

        self.cleanup(t)
        self.run_state(t)
        self.step_joints(t)

        # gripper animation (0.1 m/s)
        step = 0.1 * dt
        self.gripper += max(-step, min(step, self.gripper_goal - self.gripper))

        if self.carrying and self.target_id:
            x, y, z = fk(self.q, self.geo)
            out = String()
            out.data = f'{self.target_id},{x:.4f},{y:.4f},{z:.4f}'
            self.carry_pub.publish(out)

        self.publish_joint_states()
        if int(t * 20) != int((t - dt) * 20):
            self.publish_markers()

    def cleanup(self, t):
        for box_id in list(self.boxes):
            x, _, _ = self.box_xyz(self.boxes[box_id], t)
            if x > 4.0 and box_id != self.target_id:
                self.boxes.pop(box_id, None)
                self.detected.pop(box_id, None)
                self.taken.discard(box_id)

    def step_joints(self, t):
        if self.traj is None:
            return
        t0, T, qs, qg = self.traj
        s = smoothstep((t - t0) / T)
        self.q = [a + (b - a) * s for a, b in zip(qs, qg)]
        if t - t0 >= T:
            self.q = list(qg)
            self.traj = None

    def run_state(self, t):
        e = t - self.state_t0
        st = self.state

        if st == ArmState.IDLE:
            self.pick_target(t)

        elif st == ArmState.DETECT:
            box = self.boxes.get(self.target_id)
            if box is None:
                self.abort('box vanished before reach')
                return
            tx, ty, tz = self.box_xyz(box, t + self.reach_time)
            goal = ik(tx, ty, tz + self.hover, self.geo)
            if goal is None:
                self.abort('IK failed for intercept point')
                return
            self.start_traj(goal, self.reach_time)
            self.go(ArmState.REACH)

        elif st == ArmState.REACH:
            if self.traj is None:
                self.go(ArmState.GRASP)

        elif st == ArmState.GRASP:
            box = self.boxes.get(self.target_id)
            if box is None:
                self.abort('box vanished during grasp')
                return
            bx, by, bz = self.box_xyz(box, t)
            s = smoothstep(e / self.descend_time)
            goal = ik(bx, by, bz + self.hover * (1.0 - s), self.geo)
            if goal is None:
                self.abort('box left the reachable workspace')
                return
            self.q = goal                       # live tracking of the moving box
            if e >= self.descend_time:
                self.gripper_goal = GRIPPER_CLOSED
            if e >= self.descend_time + 0.4:
                tip = fk(self.q, self.geo)
                err = ((tip[0] - bx) ** 2 + (tip[1] - by) ** 2 + (tip[2] - bz) ** 2) ** 0.5
                if err > 0.05:
                    self.abort(f'grasp error {err:.3f} m')
                    return
                self.carrying = True
                tip = fk(self.q, self.geo)
                goal = ik(tip[0], tip[1], tip[2] + 0.2, self.geo)
                if goal is None:
                    self.abort('IK failed for lift')
                    return
                self.start_traj(goal, 0.5)
                self.go(ArmState.LIFT)

        elif st == ArmState.LIFT:
            if self.traj is None:
                goal = ik(self.bin_xy[0], self.bin_xy[1], self.drop_z, self.geo)
                if goal is None:
                    self.abort('IK failed for bin')
                    return
                self.start_traj(goal, self.place_time)
                self.go(ArmState.MOVE_TO_BIN)

        elif st == ArmState.MOVE_TO_BIN:
            if self.traj is None:
                self.gripper_goal = GRIPPER_OPEN
                self.released = False
                self.go(ArmState.RELEASE)

        elif st == ArmState.RELEASE:
            if not self.released and e >= 0.35:
                self.released = True
                self.carrying = False
                self.stats['successful'] += 1
                self.log_pick(self.target_id, 'SUCCESS', f'Placed in {self.color} bin')
                self.taken.add(self.target_id)
                self.detected.pop(self.target_id, None)
                self.target_id = None
            if e >= 0.5:
                self.start_traj(self.home, self.return_time)
                self.go(ArmState.RETURN)

        elif st == ArmState.RETURN:
            if self.traj is None:
                self.go(ArmState.IDLE)

    def pick_target(self, t):
        best, best_x = None, -1e9
        for box_id, sensor in list(self.detected.items()):
            box = self.boxes.get(box_id)
            if box is None or box['color'] != self.color or sensor != self.sensor_num:
                continue
            if box_id in self.taken:
                continue
            x_now, y, z = self.box_xyz(box, t)
            if x_now > self.geo.base_x + 0.75:            # already past the pick window
                self.taken.add(box_id)
                continue
            ix, iy, iz = self.box_xyz(box, t + self.reach_time)
            # box must stay reachable through reach + descend + gripper-close time
            t_end = t + self.reach_time + self.descend_time + 0.5
            gx, gy, gz = self.box_xyz(box, t_end)
            if ik(ix, iy, iz + self.hover, self.geo) is None or ik(gx, gy, gz, self.geo) is None:
                continue
            if x_now > best_x:                             # oldest reachable box first
                best, best_x = box_id, x_now
        if best:
            self.target_id = best
            self.taken.add(best)
            self.stats['attempted'] += 1
            self.log_pick(best, 'CLAIMED', f'Claimed by {self.name}')
            self.go(ArmState.DETECT)

    def abort(self, reason):
        self.get_logger().warn(f'{self.name}: abort - {reason}')
        self.stats['aborted'] += 1
        if self.carrying and self.target_id:
            self.log_pick(self.target_id, 'ABORTED', reason)
        self.carrying = False
        self.gripper_goal = GRIPPER_OPEN
        self.target_id = None
        self.start_traj(self.home, self.return_time)
        self.go(ArmState.RETURN)

    # ------------------------------------------------------------- publishers
    def publish_joint_states(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = [f'{self.name}_joint1', f'{self.name}_joint2', f'{self.name}_joint3',
                    f'{self.name}_gripper_finger1_joint', f'{self.name}_gripper_finger2_joint']
        msg.position = [self.q[0], self.q[1], self.q[2], self.gripper, self.gripper]
        self.js_pub.publish(msg)

    def publish_state(self):
        s = String()
        s.data = self.state.value
        self.state_pub.publish(s)

    def publish_markers(self):
        arr = MarkerArray()
        g = self.geo

        txt = Marker()
        txt.header.frame_id = 'world'
        txt.ns = f'{self.name}_state_text'
        txt.id = 0
        txt.type = Marker.TEXT_VIEW_FACING
        txt.action = Marker.ADD
        txt.pose.position = Point(x=g.base_x, y=g.base_y, z=g.shoulder_z + 0.9)
        txt.pose.orientation.w = 1.0
        txt.scale.z = 0.07
        txt.color.r = txt.color.g = txt.color.b = txt.color.a = 1.0
        txt.text = (f'{self.name.upper()} ({self.color})\n{self.state.value}\n'
                    f'picked {self.stats["successful"]}')
        arr.markers.append(txt)

        tgt = Marker()
        tgt.header.frame_id = 'world'
        tgt.ns = f'{self.name}_target'
        tgt.id = 1
        box = self.boxes.get(self.target_id) if self.target_id else None
        if box is not None and self.state in (ArmState.DETECT, ArmState.REACH, ArmState.GRASP):
            x, y, z = self.box_xyz(box)
            tgt.type = Marker.SPHERE
            tgt.action = Marker.ADD
            tgt.pose.position = Point(x=x, y=y, z=z + 0.12)
            tgt.pose.orientation.w = 1.0
            tgt.scale.x = tgt.scale.y = tgt.scale.z = 0.06
            tgt.color.r, tgt.color.g, tgt.color.b, tgt.color.a = 1.0, 1.0, 0.0, 0.9
        else:
            tgt.action = Marker.DELETE
        arr.markers.append(tgt)
        self.marker_pub.publish(arr)

    def get_stats(self):
        return dict(arm=self.name, color=self.color, state=self.state.value, **self.stats)


def main(args=None):
    rclpy.init(args=args)
    node = ArmController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info(f'{node.name} shutting down. Stats: {node.get_stats()}')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
