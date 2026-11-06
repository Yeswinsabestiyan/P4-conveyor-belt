"""Unit tests: kinematics, URDF<->kinematics consistency, RViz config format, belt maths."""
import math
import os
import sys

import pytest
import yaml

PKG = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
sys.path.insert(0, PKG)

from conveyor_sorter.kinematics import ArmGeometry, belt_x, fk, ik  # noqa: E402

ARM1 = ArmGeometry(0.8, 0.35)
ARM2 = ArmGeometry(1.8, -0.35)


def test_color_routing():
    targets = {'arm1': 'red', 'arm2': 'blue'}
    assert 'black' not in targets.values()
    assert len(set(targets.values())) == 2


@pytest.mark.parametrize('g', [ARM1, ARM2])
def test_ik_fk_roundtrip(g):
    for dx in (-0.5, -0.2, 0.0, 0.3, 0.6):
        for z in (0.555, 0.675):
            target = (g.base_x + dx, 0.0, z)
            q = ik(*target, g)
            assert q is not None, target
            assert math.dist(fk(q, g), target) < 1e-6


def test_bins_reachable():
    assert ik(0.8, 0.75, 0.50, ARM1) is not None
    assert ik(1.8, -0.75, 0.50, ARM2) is not None


def test_unreachable_rejected():
    assert ik(3.0, 0.0, 0.6, ARM1) is None


def test_home_pose_is_valid():
    x, y, z = fk([0.0, -0.9, 1.5], ARM1)
    assert z > 0.6


def test_belt_feedforward():
    assert belt_x(0.0, 10.0, 0.15, 12.0) == pytest.approx(0.3)


def test_rviz_config_is_rviz2_format():
    cfg = yaml.safe_load(open(os.path.join(PKG, 'rviz', 'sorter.rviz')))
    disp = cfg['Visualization Manager']['Displays']
    for d in disp:
        if d['Class'].endswith('MarkerArray'):
            assert d['Topic']['Value'].startswith('/')     # ROS1-style 'Marker Topic' is ignored
        if d['Class'].endswith('RobotModel'):
            assert d['Description Topic']['Value'] == '/robot_description'
            assert d['Description Topic']['Durability Policy'] == 'Transient Local'
    classes = [p['Class'] for p in cfg['Panels']]
    assert 'rviz_common/Toolbar' not in classes


def test_urdf_matches_kinematics():
    xacro = pytest.importorskip('xacro')
    urdf_py = pytest.importorskip('urdf_parser_py.urdf')
    import numpy as np
    doc = xacro.process_file(os.path.join(PKG, 'urdf', 'main.urdf.xacro'))
    robot = urdf_py.URDF.from_xml_string(doc.toxml())
    parents = {j.child: j for j in robot.joints}
    assert len(parents) == len(robot.joints) == len(robot.links) - 1

    def rpy(r, p, y):
        rx = np.array([[1, 0, 0], [0, math.cos(r), -math.sin(r)], [0, math.sin(r), math.cos(r)]])
        ry = np.array([[math.cos(p), 0, math.sin(p)], [0, 1, 0], [-math.sin(p), 0, math.cos(p)]])
        rz = np.array([[math.cos(y), -math.sin(y), 0], [math.sin(y), math.cos(y), 0], [0, 0, 1]])
        return rz @ ry @ rx

    def rodrigues(axis, th):
        a = np.array(axis, float)
        a /= np.linalg.norm(a)
        k = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
        return np.eye(3) + math.sin(th) * k + (1 - math.cos(th)) * k @ k

    def tip(link, q):
        t = np.eye(4)
        while link in parents:
            j = parents[link]
            a = np.eye(4)
            if j.origin:
                a[:3, :3] = rpy(*j.origin.rpy)
                a[:3, 3] = j.origin.xyz
            b = np.eye(4)
            if j.type == 'revolute':
                b[:3, :3] = rodrigues(j.axis, q.get(j.name, 0.0))
            t = a @ b @ t
            link = j.parent
        return t[:3, 3]

    for q in ([0.0, -0.9, 1.5], [-1.57, -0.5, 2.19], [1.2, -0.4, 1.0]):
        urdf_tip = tip('arm1_tool_tip', dict(zip(['arm1_joint1', 'arm1_joint2', 'arm1_joint3'], q)))
        assert math.dist(urdf_tip, fk(q, ARM1)) < 1e-4
