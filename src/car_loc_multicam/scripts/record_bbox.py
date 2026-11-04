#!/usr/bin/env python3
"""錄 /env_cam_i/car_bbox + ground truth /odom + /multicam_loc/odom -> jsonl。

給 scripts/fit_car_model.py 用 (量 edge_bias_px、比較車型)。偵測節點要開著:

    python3 src/car_loc_multicam/scripts/record_bbox.py /tmp/rec.jsonl 120 --ros-args -p use_sim_time:=true

第二個參數是錄幾秒 (牆上時鐘)。錄的時候車子要動, 而且要開過房間的各個角落 ——
只在中間繞的話, 遠處 / 斜角的框都沒有量到。
"""
import sys, json, math, rclpy
from rclpy.node import Node
from geometry_msgs.msg import PolygonStamped
from nav_msgs.msg import Odometry
out = open(sys.argv[1], 'w'); dur = float(sys.argv[2])
rclpy.init(args=sys.argv); n = Node('rec_bbox'); t0 = [None]
def st(s): return s.sec + s.nanosec * 1e-9
def yaw(q): return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
def od(m, k):
    p = m.pose.pose.position
    out.write(json.dumps(dict(k=k, t=st(m.header.stamp), x=p.x, y=p.y, yaw=yaw(m.pose.pose.orientation))) + '\n')
def bb(m):
    pts = m.polygon.points
    out.write(json.dumps(dict(k='bbox', t=st(m.header.stamp), cam=m.header.frame_id,
        boxes=[[pts[i].x, pts[i].y, pts[i+1].x, pts[i+1].y, pts[i].z] for i in range(0, len(pts) - 1, 2)])) + '\n')
n.create_subscription(Odometry, '/odom', lambda m: od(m, 'gt'), 50)
n.create_subscription(Odometry, '/multicam_loc/odom', lambda m: od(m, 'est'), 50)
for i in range(8):
    n.create_subscription(PolygonStamped, f'/env_cam_{i}/car_bbox', bb, 50)
import time; t = time.time()
while time.time() - t < dur: rclpy.spin_once(n, timeout_sec=0.1)
out.close()
