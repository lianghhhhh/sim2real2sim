"""最小的 ROS 假模組, 用來在沒有 ROS 的機器上跑節點的邏輯。"""
import sys, types, math

def mod(name, **attrs):
    m = types.ModuleType(name)
    for k, v in attrs.items(): setattr(m, k, v)
    sys.modules[name] = m
    return m

class _P:
    def __init__(self, v): self.value = v

class Node:
    def __init__(self, name):
        self._p = {}
        self.timers = []
        self.subs = {}
    def declare_parameter(self, n, v=None): self._p[n] = _P(v)
    def get_parameter(self, n): return self._p[n]
    def create_publisher(self, *a, **k): return types.SimpleNamespace(publish=lambda m: None)
    def create_subscription(self, t, topic, cb, qos): self.subs[topic] = cb
    def create_service(self, *a, **k): return None
    def create_timer(self, p, cb): self.timers.append(cb); return None
    def get_logger(self):
        return types.SimpleNamespace(
            info=lambda m: print('[info]', m), warn=lambda m: print('[WARN]', m),
            error=lambda m: print('[ERR]', m))
    def get_clock(self):
        return types.SimpleNamespace(now=lambda: types.SimpleNamespace(
            nanoseconds=0, to_msg=lambda: types.SimpleNamespace(sec=0, nanosec=0)))
    def destroy_node(self): pass

mod('rclpy', init=lambda **k: None, spin=lambda n: None, ok=lambda: False,
    shutdown=lambda: None)
mod('rclpy.node', Node=Node)
mod('rclpy.qos', QoSProfile=lambda **k: None,
    HistoryPolicy=types.SimpleNamespace(KEEP_LAST=1),
    ReliabilityPolicy=types.SimpleNamespace(BEST_EFFORT=2))
mod('tf2_ros', TransformBroadcaster=lambda n: types.SimpleNamespace(
    sendTransform=lambda t: None))

def _struct(**d):
    def f(): return types.SimpleNamespace(**{k: (v() if callable(v) else v)
                                             for k, v in d.items()})
    return f
def _vec(): return types.SimpleNamespace(x=0.0, y=0.0, z=0.0)
def _quat(): return types.SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0)
def _hdr(): return types.SimpleNamespace(stamp=types.SimpleNamespace(sec=0, nanosec=0),
                                         frame_id='')
def _pose(): return types.SimpleNamespace(position=_vec(), orientation=_quat())
mod('geometry_msgs.msg',
    PoseWithCovarianceStamped=_struct(header=_hdr,
        pose=lambda: types.SimpleNamespace(pose=_pose(), covariance=[0.0]*36)),
    TransformStamped=_struct(header=_hdr, child_frame_id='',
        transform=lambda: types.SimpleNamespace(translation=_vec(), rotation=_quat())))
mod('nav_msgs.msg', Odometry=_struct(header=_hdr, child_frame_id='',
    pose=lambda: types.SimpleNamespace(pose=_pose(), covariance=[0.0]*36),
    twist=lambda: types.SimpleNamespace(
        twist=types.SimpleNamespace(linear=_vec(), angular=_vec()),
        covariance=[0.0]*36)))
mod('sensor_msgs.msg', Imu=_struct(header=_hdr, orientation=_quat,
    angular_velocity=_vec, linear_acceleration=_vec),
    JointState=_struct(header=_hdr, name=list, velocity=list, effort=list))
mod('std_srvs.srv', Trigger=object)
