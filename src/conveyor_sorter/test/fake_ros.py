"""Minimal fake rclpy so the node logic can be simulated headlessly in pytest."""
import sys, types, random, os
random.seed(3)
CLOCK=[1000.0]
BUS={}   # topic -> list of subscriber callbacks
TIMERS=[]
class Msg:
    def __init__(self,**kw):
        for k,v in kw.items(): setattr(self,k,v)
def mk(name, **defaults):
    class C:
        def __init__(self,**kw):
            for k,v in defaults.items():
                setattr(self,k, v() if callable(v) else v)
            for k,v in kw.items(): setattr(self,k,v)
    C.__name__=name; return C
class Hdr:
    def __init__(self): self.frame_id=''; self.stamp=None
class Pt:
    def __init__(self,x=0.,y=0.,z=0.): self.x,self.y,self.z=x,y,z
class Quat:
    def __init__(self): self.x=self.y=self.z=0.; self.w=1.
class Pose:
    def __init__(self): self.position=Pt(); self.orientation=Quat()
class Vec:
    def __init__(self): self.x=self.y=self.z=0.
class RGBA:
    def __init__(self,r=0.,g=0.,b=0.,a=1.): self.r,self.g,self.b,self.a=r,g,b,a
class Marker:
    ADD=0;DELETE=2;DELETEALL=3;CUBE=1;SPHERE=2;TEXT_VIEW_FACING=9;CUBE_LIST=6
    def __init__(self):
        self.header=Hdr();self.ns='';self.id=0;self.type=0;self.action=0;self.pose=Pose();self.scale=Vec();self.color=RGBA();self.text='';self.points=[]
class MarkerArray:
    def __init__(self): self.markers=[]
class String:
    def __init__(self): self.data=''
class Float64(String): pass
class JointState:
    def __init__(self): self.header=Hdr();self.name=[];self.position=[]
class Time:
    def __init__(self,ns): self.nanoseconds=ns
    def to_msg(self): return None
class Clock:
    def now(self): return Time(int(CLOCK[0]*1e9))
class Logger:
    def info(self,m): LOGS.append(('I',m))
    def warn(self,m): LOGS.append(('W',m))
LOGS=[]
class Param:
    def __init__(s,v): s.value=v
class Node:
    def __init__(self,name,**kw):
        self.name_=name; self.params={}
        self.overrides=NODE_PARAMS.get(name_override[0],{})
    def declare_parameter(self,n,v=None):
        self.params[n]=self.overrides.get(n,v)
    def get_parameter(self,n): return Param(self.params[n])
    def create_publisher(self,typ,topic,qos=10):
        class P:
            def publish(s,m):
                for cb in list(BUS.get(topic,[])): cb(m)
        return P()
    def create_subscription(self,typ,topic,cb,qos=10): BUS.setdefault(topic,[]).append(cb)
    def create_timer(self,period,cb): TIMERS.append([period,cb,CLOCK[0]+period])
    def get_clock(self): return Clock()
    def get_logger(self): return Logger()
    def destroy_node(self): pass
for m in ['rclpy','rclpy.node','rclpy.qos','std_msgs','std_msgs.msg','geometry_msgs','geometry_msgs.msg','visualization_msgs','visualization_msgs.msg','sensor_msgs','sensor_msgs.msg']:
    sys.modules[m]=types.ModuleType(m)
sys.modules['rclpy.node'].Node=Node
sys.modules['std_msgs.msg'].String=String; sys.modules['std_msgs.msg'].Float64=Float64; sys.modules['std_msgs.msg'].ColorRGBA=RGBA
sys.modules['geometry_msgs.msg'].Point=Pt
sys.modules['visualization_msgs.msg'].Marker=Marker; sys.modules['visualization_msgs.msg'].MarkerArray=MarkerArray
sys.modules['sensor_msgs.msg'].JointState=JointState
PKG=os.path.abspath(os.path.join(os.path.dirname(__file__),'..'))
NODE_PARAMS={}
name_override=['']
import yaml
cfg=yaml.safe_load(open(os.path.join(PKG,'config','sorter_params.yaml')))
sys.path.insert(0,PKG)
import importlib
def build(modname, clsname, yamlkey):
    NODE_PARAMS[yamlkey]=cfg[yamlkey]['ros__parameters']; name_override[0]=yamlkey
    mod=importlib.import_module('conveyor_sorter.'+modname)
    return getattr(mod,clsname)()


def simulate(seconds=120.0, dt=0.01):
    """Build all nodes on the fake bus, run, return (spawner, arm1, arm2, stats)."""
    spawner = build('box_spawner', 'BoxSpawner', 'box_spawner')
    build('conveyor', 'Conveyor', 'conveyor')
    build('color_sensor', 'ColorSensor', 'color_sensor')
    a1 = build('arm_controller', 'ArmController', 'arm1')
    a2 = build('arm_controller', 'ArmController', 'arm2')
    stats = build('stats_display', 'StatsDisplay', 'stats_display')
    t = 0.0
    while t < seconds:
        CLOCK[0] += dt
        t += dt
        for tm in TIMERS:
            while CLOCK[0] >= tm[2]:
                tm[1]()
                tm[2] += tm[0]
    return spawner, a1, a2, stats, LOGS
