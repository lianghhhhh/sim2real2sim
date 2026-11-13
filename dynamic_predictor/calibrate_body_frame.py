"""
Run this against your training data to determine which of the 4
possible (forward, lateral) sign/axis conventions actually matches your
car_linear_velocity_x / car_linear_velocity_y channels.

kros_car.usd (faces +x, Isaac /odom twist in the body frame) should come out
as 'A' with vx = forward, vy = lateral (left); it did on 2026-10-21:
forward corr 1.000, lateral corr 0.994, d(theta) vs wz*dt 0.997.

Usage:
    python calibrate_body_frame.py

Reads data_dir + data_source straight out of config.json, so run it from
the same directory as main.py/config.json. Uses the same loader as training
(measure rows only, resampled to the uniform DT grid, split at gaps).
"""
import json
import numpy as np
from utils import STATE_COLS, DT, load_segments_from_files, wrap_angle

IDX_X = STATE_COLS.index('car_position_x')
IDX_Y = STATE_COLS.index('car_position_y')
IDX_TH = STATE_COLS.index('theta')
IDX_VX = STATE_COLS.index('car_linear_velocity_x')
IDX_VY = STATE_COLS.index('car_linear_velocity_y')
IDX_WZ = STATE_COLS.index('car_angular_velocity_z')

# The 4 candidate proper-rotation conventions (all det=+1). They only differ
# in which world axis is treated as the theta=0 forward direction and which
# rotation direction is "left".
CANDIDATES = {
    'A: fwd=cos*dx+sin*dy, lat=-sin*dx+cos*dy': lambda dx, dy, s, c: (c*dx + s*dy, -s*dx + c*dy),
    'B: fwd=sin*dx+cos*dy, lat=-cos*dx+sin*dy': lambda dx, dy, s, c: (s*dx + c*dy, -c*dx + s*dy),
    'C: fwd=-cos*dx-sin*dy, lat=sin*dx-cos*dy': lambda dx, dy, s, c: (-c*dx - s*dy, s*dx - c*dy),
    'D: fwd=-sin*dx-cos*dy, lat=cos*dx-sin*dy': lambda dx, dy, s, c: (-s*dx - c*dy, c*dx - s*dy),
}

with open('config.json') as f:
    config = json.load(f)

segments = load_segments_from_files(config['data_dir'], config['data_source'])
if not segments:
    raise SystemExit('no data')

all_dx, all_dy, all_s, all_c, all_vx, all_vy, all_dth, all_wz = [], [], [], [], [], [], [], []
for seg in segments:
    state = seg['state']
    th = state[:-1, IDX_TH]
    all_dx.append(np.diff(state[:, IDX_X]))
    all_dy.append(np.diff(state[:, IDX_Y]))
    all_s.append(np.sin(th))
    all_c.append(np.cos(th))
    # trapezoidal averages, same as kinematic_physics_loss
    all_vx.append((state[:-1, IDX_VX] + state[1:, IDX_VX]) / 2)
    all_vy.append((state[:-1, IDX_VY] + state[1:, IDX_VY]) / 2)
    all_wz.append((state[:-1, IDX_WZ] + state[1:, IDX_WZ]) / 2)
    all_dth.append(wrap_angle(np.diff(state[:, IDX_TH])))

dx = np.concatenate(all_dx); dy = np.concatenate(all_dy)
s = np.concatenate(all_s); c = np.concatenate(all_c)
vx = np.concatenate(all_vx); vy = np.concatenate(all_vy)
wz = np.concatenate(all_wz); dth = np.concatenate(all_dth)

expect_forward = vx * DT   # kros_car.usd: vx = forward
expect_lateral = vy * DT   #               vy = lateral (left)

print(f"{'candidate':<45} {'fwd_corr':>9} {'fwd_rmse':>10} {'lat_corr':>9} {'lat_rmse':>10}  score")
best_name, best_score = None, -np.inf
for name, fn in CANDIDATES.items():
    fwd, lat = fn(dx, dy, s, c)
    fwd_corr = np.corrcoef(fwd, expect_forward)[0, 1]
    lat_corr = np.corrcoef(lat, expect_lateral)[0, 1]
    fwd_rmse = np.sqrt(np.mean((fwd - expect_forward) ** 2))
    lat_rmse = np.sqrt(np.mean((lat - expect_lateral) ** 2))
    score = fwd_corr + lat_corr
    print(f"{name:<45} {fwd_corr:>9.4f} {fwd_rmse:>10.5f} {lat_corr:>9.4f} {lat_rmse:>10.5f}  {score:.4f}")
    if score > best_score:
        best_score, best_name = score, name

print(f"\nd(theta) vs +wz*dt corr: {np.corrcoef(dth, wz * DT)[0, 1]:.4f}  (should be ~ +1: no yaw sign flip)")
print(f"Best match: {best_name}")
print("Use this letter for BODY_FRAME_CONVENTION in utils.py.")
