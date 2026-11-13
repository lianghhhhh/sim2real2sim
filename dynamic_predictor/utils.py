import os
import numpy as np
import pandas as pd

# ---------------------------------------------------------------------------
# Column definitions
# ---------------------------------------------------------------------------
# CONTROL_COLS: truly exogenous quantities -- the wheel-speed commands (rad/s)
# sent on /joint_command. The car is velocity-controlled: the motor's own
# speed loop (in sim: actuator_model's delay + 30 rad/s^2 ramp in front of
# Isaac's velocity drive) tracks them. Front and rear wheels on one side
# always get the same command, so the left/right pair is enough.
CONTROL_COLS = [
    'velocity_command_front_left', 'velocity_command_front_right',
]

# STATE_COLS: the dynamic state of the car, including the wheels. With
# velocity control the commanded speed is NOT the wheel speed -- between them
# sit the actuator delay and acceleration ramp -- so the measured wheel
# speeds are the actuator's state and the model has to predict them just like
# position/velocity/heading. This is what the model predicts (as a delta) and
# it is exactly what gets fed back in as "current state" for the next rollout
# step, so input-state and output-state dimensions match.
#
# Heading is a single angle `theta` (its delta is wrapped to [-pi, pi)).
# Heading is not a model feature (see NON_FEATURE_COLS), so sin/cos would only
# matter for rollout bookkeeping -- and their deltas depend on the absolute
# heading, while d(theta) does not.
STATE_COLS = [
    'car_position_x', 'car_position_y',
    'theta',
    'car_linear_velocity_x', 'car_linear_velocity_y', 'car_angular_velocity_z',
    'front_left_velocity', 'front_right_velocity',
    'rear_left_velocity', 'rear_right_velocity',
]

# Wheel speeds come from /joint_states (their own stamp); everything else in
# STATE_COLS comes from /odom.
WHEEL_VEL_COLS = ['front_left_velocity', 'front_right_velocity',
                  'rear_left_velocity', 'rear_right_velocity']

# Absolute x/y position and heading are meaningless as *features* (on flat
# ground the dynamics don't depend on where the car is or which way it
# points -- feeding them in just invites overfitting to the ranges seen in
# training). They still live in STATE_COLS because the model needs to predict
# their deltas and rollout needs to track the absolute pose for
# comparison/plotting -- they're just excluded from the "absolute state" block
# of the model's input features. Their deltas are still fed in.
POSITION_COLS = ['car_position_x', 'car_position_y']
HEADING_COL = 'theta'
NON_FEATURE_COLS = POSITION_COLS + [HEADING_COL]
ABS_STATE_MASK = np.array([col not in NON_FEATURE_COLS for col in STATE_COLS])
N_ABS_STATE = int(ABS_STATE_MASK.sum())

DT = 0.05            # fixed model step (s) = the command rate (20 Hz)
LOOKAHEAD = 1
SEQ_LENGTH = 10
RESET_JUMP_THRESHOLD = 0.14  # position jump bigger than this => treat as episode reset
GAP_THRESHOLD = 0.1          # gap in the Isaac stamps bigger than this (s) => new segment
MEASURE_PHASE = 'measure'    # collect_car_pkg / control_car_node: only these rows are driving data

# Indices of the position/heading channels within STATE_COLS, used to
# convert position deltas between world frame and the car's body frame
# (forward/lateral). car_position_x/y themselves stay world-frame absolute
# values -- only their *deltas* (model input history + prediction target)
# get expressed in body frame, since forward/lateral motion is what the
# actuators actually control and generalizes much better than world dx/dy.
POS_X_IDX = STATE_COLS.index('car_position_x')
POS_Y_IDX = STATE_COLS.index('car_position_y')
THETA_IDX = STATE_COLS.index(HEADING_COL)
VEL_IDX = [STATE_COLS.index('car_linear_velocity_x'), STATE_COLS.index('car_linear_velocity_y')]
WZ_IDX = STATE_COLS.index('car_angular_velocity_z')

# Which world axis is "forward" at heading==0, and which rotation sense is
# "left" -- run calibrate_body_frame.py against your real training data to
# determine this empirically (it checks correlation against
# car_linear_velocity_x/y, which is the ground truth for what your sim
# actually calls forward/lateral). Getting this wrong doesn't break the
# training math (it's still a proper, invertible rotation either way) but
# it DOES make the kinematic_physics_loss (weight_xy) fight the model
# instead of helping it, since that loss compares against vx/vy directly.
#   'A' -> forward = cos*dx + sin*dy,  lateral = -sin*dx + cos*dy
#   'B' -> forward = sin*dx + cos*dy,  lateral = -cos*dx + sin*dy
#   'C' -> forward = -cos*dx - sin*dy, lateral = sin*dx - cos*dy
#   'D' -> forward = -sin*dx - cos*dy, lateral = cos*dx - sin*dy
# kros_car.usd faces +x (REP-103): 'A'. Checked 2026-10-21 on kros_car.usd
# data: forward vs vx*dt corr 1.000, lateral vs vy*dt 0.994 ('B' was -0.13).
# The old car faced -y and used 'B' with flipped signs.
BODY_FRAME_CONVENTION = 'A'


def wrap_angle(a):
    """Wrap angle(s) to [-pi, pi)."""
    return (np.asarray(a) + np.pi) % (2 * np.pi) - np.pi


def _rotate_world_to_body(dx, dy, sin_h, cos_h):
    if BODY_FRAME_CONVENTION == 'A':
        return cos_h * dx + sin_h * dy, -sin_h * dx + cos_h * dy
    elif BODY_FRAME_CONVENTION == 'B':
        return sin_h * dx + cos_h * dy, -cos_h * dx + sin_h * dy
    elif BODY_FRAME_CONVENTION == 'C':
        return -cos_h * dx - sin_h * dy, sin_h * dx - cos_h * dy
    elif BODY_FRAME_CONVENTION == 'D':
        return -sin_h * dx - cos_h * dy, cos_h * dx - sin_h * dy
    else:
        raise ValueError(f"Unknown BODY_FRAME_CONVENTION: {BODY_FRAME_CONVENTION}")


def _rotate_body_to_world(forward, lateral, sin_h, cos_h):
    if BODY_FRAME_CONVENTION == 'A':
        return cos_h * forward - sin_h * lateral, sin_h * forward + cos_h * lateral
    elif BODY_FRAME_CONVENTION == 'B':
        return sin_h * forward - cos_h * lateral, cos_h * forward + sin_h * lateral
    elif BODY_FRAME_CONVENTION == 'C':
        return -cos_h * forward + sin_h * lateral, -sin_h * forward - cos_h * lateral
    elif BODY_FRAME_CONVENTION == 'D':
        return -sin_h * forward + cos_h * lateral, -cos_h * forward - sin_h * lateral
    else:
        raise ValueError(f"Unknown BODY_FRAME_CONVENTION: {BODY_FRAME_CONVENTION}")


def state_delta_body_frame(state_from, state_to):
    """Delta between two absolute (world-frame) state rows, with the
    position component rotated into the body frame of `state_from`
    (forward = along heading, lateral = to the left of heading) instead of
    raw world-frame dx/dy, and the heading delta wrapped to [-pi, pi). Every
    other channel (velocities, wheel speeds) is a plain difference.

    Works on a single row (1D, shape (N_STATE,)) or a batch of rows (shape
    (..., N_STATE)) as long as the last axis is the STATE_COLS axis.
    """
    state_from = np.asarray(state_from)
    delta = np.asarray(state_to) - state_from
    theta = state_from[..., THETA_IDX]
    forward, lateral = _rotate_world_to_body(delta[..., POS_X_IDX], delta[..., POS_Y_IDX],
                                             np.sin(theta), np.cos(theta))
    delta = delta.copy()
    delta[..., POS_X_IDX] = forward
    delta[..., POS_Y_IDX] = lateral
    delta[..., THETA_IDX] = wrap_angle(delta[..., THETA_IDX])
    return delta


def apply_body_frame_delta(state_from, delta):
    """Inverse of state_delta_body_frame: given an absolute (world-frame)
    state row and a delta whose position component is (forward, lateral)
    expressed in the body frame of `state_from`, return the resulting
    absolute world-frame state (heading wrapped). Other channels are added
    directly.
    """
    state_from = np.asarray(state_from)
    theta = state_from[..., THETA_IDX]
    dx, dy = _rotate_body_to_world(delta[..., POS_X_IDX], delta[..., POS_Y_IDX],
                                   np.sin(theta), np.cos(theta))

    next_state = state_from + delta  # correct for velocity / wheel channels
    next_state[..., POS_X_IDX] = state_from[..., POS_X_IDX] + dx
    next_state[..., POS_Y_IDX] = state_from[..., POS_Y_IDX] + dy
    next_state[..., THETA_IDX] = wrap_angle(next_state[..., THETA_IDX])
    return next_state


def compute_angle(qx, qy, qz, qw):
    """Yaw angle from quaternion (planar car, so only yaw matters)."""
    siny_cosp = 2 * (qw * qz + qx * qy)
    cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
    return np.arctan2(siny_cosp, cosy_cosp)


def _add_derived_columns(data: pd.DataFrame) -> pd.DataFrame:
    # No sign flips: kros_car.usd faces +x, and Isaac's /odom twist is in the
    # body frame (vx forward, vy left, wz counter-clockwise), consistent with
    # the GT yaw (d(theta) vs wz*dt corr 0.997, 2026-10-21). The old car faced
    # -y and needed theta, vx, vy, wz negated.
    data = data.copy()
    data['theta'] = compute_angle(data['car_orientation_x'], data['car_orientation_y'],
                                  data['car_orientation_z'], data['car_orientation_w'])
    return data


# ---------------------------------------------------------------------------
# Loading: CSV (calibrate_env_pkg collect_data_node) -> uniform DT segments
# ---------------------------------------------------------------------------
# The collector logs the *latest* value of every topic at 50 Hz of sim time.
# Under Isaac's 60 Hz clock that means rows 1/60 or 2/60 s apart, and some rows
# repeat the previous message (the car state only updates every frame or
# two). The model assumes one row = DT seconds, so every file is resampled
# onto a uniform DT grid, using each source's own stamp:
#   * /odom columns (pose, twist)  on `odom_stamp`   -- linear interpolation
#                                                       (theta unwrapped first)
#   * wheel speeds                 on `wheel_stamp`  -- linear interpolation
#   * commands                     on `timestamp`    -- zero-order hold (they are
#                                                       steps, not samples)
# `timestamp` is the collector's ROS clock, which runs a frame or so behind
# Isaac's stamps; it is shifted onto the Isaac timeline by the median
# (timestamp - odom_stamp) of the segment (same idea as plot_wheel_bench.py).
# Files without the stamp columns (older logs) fall back to `timestamp`.

def _stamp_col(data, col):
    want = 'wheel_stamp' if col in WHEEL_VEL_COLS else 'odom_stamp'
    return want if want in data.columns else 'timestamp'


def _interp_unique(t, y, grid, angle=False):
    """Interpolate y(t) on grid, using only the first row of each repeated stamp."""
    t = np.asarray(t, dtype=float)
    y = np.asarray(y, dtype=float)
    keep = np.r_[True, np.diff(t) > 0]
    t, y = t[keep], y[keep]
    if angle:
        return wrap_angle(np.interp(grid, t, np.unwrap(y)))
    return np.interp(grid, t, y)


def _resample_chunk(chunk, dt):
    """One gap-free chunk of rows -> (controls, state) on a uniform dt grid."""
    stamps = {_stamp_col(chunk, c) for c in STATE_COLS}
    t0 = max(chunk[s].iloc[0] for s in stamps)
    t1 = min(chunk[s].iloc[-1] for s in stamps)
    if t1 - t0 < dt:
        return None
    grid = np.arange(t0, t1 + 1e-9, dt)

    state = np.column_stack([
        _interp_unique(chunk[_stamp_col(chunk, c)], chunk[c], grid, angle=(c == HEADING_COL))
        for c in STATE_COLS])

    t_cmd = chunk['timestamp'].to_numpy(dtype=float)
    if 'odom_stamp' in chunk.columns:
        t_cmd = t_cmd - np.median(t_cmd - chunk['odom_stamp'].to_numpy(dtype=float))
    idx = np.clip(np.searchsorted(t_cmd, grid, side='right') - 1, 0, len(t_cmd) - 1)
    controls = chunk[CONTROL_COLS].to_numpy(dtype=float)[idx]
    return controls, state


def load_segments(path, dt=DT, min_len=SEQ_LENGTH + 2):
    """Load one CSV and return its continuous driving segments, each resampled
    to a uniform `dt` grid: list of dicts with
        'controls': (T, len(CONTROL_COLS))
        'state':    (T, len(STATE_COLS))
        'source':   file name
    Only phase == 'measure' rows are used (when the file has a phase column).
    A segment ends wherever the Isaac stamps jump by more than GAP_THRESHOLD
    (or go backwards: Isaac Stop/Play) or the position jumps by more than
    RESET_JUMP_THRESHOLD between samples.
    """
    name = os.path.basename(path)
    data = pd.read_csv(path)
    if 'phase' in data.columns:
        data = data[data['phase'] == MEASURE_PHASE]
    data = _add_derived_columns(data)

    stamps = sorted({_stamp_col(data, c) for c in STATE_COLS} | {'timestamp'})
    need = CONTROL_COLS + STATE_COLS + stamps
    missing = [c for c in need if c not in data.columns]
    if missing:
        raise KeyError(f"{name}: missing columns {missing} (velocity-control collector CSV expected)")
    data = data.dropna(subset=need).reset_index(drop=True)
    if len(data) < 2:
        print(f"{name}: no usable rows")
        return []

    primary = _stamp_col(data, 'car_position_x')
    t = data[primary].to_numpy(dtype=float)
    pos = data[POSITION_COLS].to_numpy(dtype=float)
    dt_rows = np.diff(t)
    jump = np.linalg.norm(np.diff(pos, axis=0), axis=1)
    cuts = np.flatnonzero((dt_rows > GAP_THRESHOLD) | (dt_rows < 0) | (jump > RESET_JUMP_THRESHOLD)) + 1
    bounds = np.r_[0, cuts, len(data)]

    segments = []
    for a, b in zip(bounds[:-1], bounds[1:]):
        out = _resample_chunk(data.iloc[a:b], dt)
        if out is None or len(out[1]) < min_len:
            continue
        segments.append({'controls': out[0], 'state': out[1], 'source': name})
    total = sum(len(s['state']) for s in segments) * dt
    print(f"{name}: {len(segments)} segment(s), {total:.0f} s at dt={dt}")
    return segments


def load_segments_from_files(data_dir, files, dt=DT, min_len=SEQ_LENGTH + 2):
    segments = []
    for f in files:
        path = os.path.join(data_dir, f)
        if not os.path.exists(path):
            raise FileNotFoundError(f"data file not found: {path} (check data_dir / file names in config.json)")
        segments.extend(load_segments(path, dt, min_len))
    return segments


def split_segments(segments, val_fraction, min_len=SEQ_LENGTH + 2):
    """Time-block split: the last `val_fraction` of every segment becomes
    validation data, the rest training data. Random row/window splits leak
    (neighbouring windows overlap by seq_length-1 rows), blocks don't."""
    train, val = [], []
    for seg in segments:
        n = len(seg['state'])
        cut = int(round(n * (1.0 - val_fraction)))
        for part, (a, b) in ((train, (0, cut)), (val, (cut, n))):
            if b - a >= min_len:
                part.append({k: (v[a:b] if isinstance(v, np.ndarray) else v) for k, v in seg.items()})
    return train, val


def windows_from_segments(segments, seq_length=SEQ_LENGTH, lookahead=LOOKAHEAD):
    """Slide a window over every segment and emit (input_sequence,
    output_delta) pairs. Window rows are [controls, absolute state features,
    delta from the previous row]; the target is the delta to the next row."""
    inputs, outputs = [], []
    for seg in segments:
        control_vals, state_vals = seg['controls'], seg['state']
        count = len(state_vals) - lookahead
        deltas = make_delta_buffer(state_vals)   # row 0 of a segment: no previous sample -> 0
        for i in range(count - seq_length):
            window = np.concatenate([control_vals[i:i + seq_length],
                                     state_vals[i:i + seq_length][:, ABS_STATE_MASK],
                                     deltas[i:i + seq_length]], axis=1)
            inputs.append(window)
            outputs.append(state_delta_body_frame(state_vals[i + seq_length - 1],
                                                  state_vals[i + seq_length]))
    return inputs, outputs


def assemble_window(controls, abs_state, delta_state, t, seq_length):
    """Build the seq_length x (N_CONTROL+N_ABS_STATE+N_STATE) raw input
    window ending at index t (inclusive) of the given per-timestep arrays.
    Absolute x/y position and heading are excluded from the absolute-state
    block (ABS_STATE_MASK) -- only their deltas are fed in. Shared by training
    (multi-step rollout loss) and evaluation, so the two stay consistent."""
    rows = []
    for j in range(t - seq_length + 1, t + 1):
        rows.append(np.concatenate([controls[j], abs_state[j][ABS_STATE_MASK], delta_state[j]]))
    return np.array(rows)


def make_delta_buffer(abs_state_window):
    """Given a T x N_STATE absolute-state window, compute the consecutive
    deltas the same way training data does (first row gets zero delta, since
    there's no earlier sample). Position channels come back as body-frame
    (forward/lateral) deltas, not world dx/dy; heading as wrapped d(theta)."""
    abs_state_window = np.asarray(abs_state_window, dtype=float)
    delta = np.zeros_like(abs_state_window)
    if len(abs_state_window) > 1:
        delta[1:] = state_delta_body_frame(abs_state_window[:-1], abs_state_window[1:])
    return delta


def get_data(data_dir, data_source, seq_length=SEQ_LENGTH, lookahead=LOOKAHEAD):
    segments = load_segments_from_files(data_dir, data_source)
    return windows_from_segments(segments, seq_length, lookahead)


def get_test_data(data_dir, test_data_source, seq_length=SEQ_LENGTH, lookahead=LOOKAHEAD):
    """Kept for backward-compat one-step-ahead evaluation. Also returns the
    raw absolute state at the *end* of each input window (the anchor state),
    which is what you need to turn a predicted delta back into an absolute
    state for plotting."""
    segments = load_segments(os.path.join(data_dir, test_data_source))
    inputs, outputs = windows_from_segments(segments, seq_length, lookahead)
    anchor_states = [seg['state'][i + seq_length - 1]
                     for seg in segments
                     for i in range(len(seg['state']) - lookahead - seq_length)]
    return inputs, outputs, anchor_states


def load_trajectory_segments(data_dir, file, seq_length=SEQ_LENGTH):
    """Load one CSV and split it into continuous (no-gap, no-reset) segments
    on the uniform DT grid. This is what closed-loop / rollout evaluation
    needs: real control inputs for every step, plus the true state trajectory
    to seed the rollout and to compare predictions against.

    Returns: list of dicts, each with:
        'controls': (T, len(CONTROL_COLS))
        'state':    (T, len(STATE_COLS))   -- ground truth
    Segments shorter than seq_length+2 are dropped (can't seed + roll at least 1 step).
    """
    segments = load_segments(os.path.join(data_dir, file), min_len=seq_length + 2)
    if not segments:
        print(f"{file}: no segment long enough for seq_length={seq_length}")
    return segments
