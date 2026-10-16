"""Read/write the single-source-of-truth calibration YAML.

Path priority: $ROS2_CALIBRATION_YAML > ~/.ros/calibration_results.yaml.
Uses ruamel.yaml so existing comments/formatting are preserved on partial updates.
"""

from __future__ import annotations

import datetime
import getpass
import os
from pathlib import Path
from typing import Any

from ruamel.yaml import YAML
from ruamel.yaml.comments import CommentedMap


_DEFAULT_PATH = Path.home() / ".ros" / "calibration_results.yaml"
_ENV_VAR = "ROS2_CALIBRATION_YAML"


def _default_template() -> dict[str, Any]:
    return {
        "imu": {
            "gyro_bias": [0.0, 0.0, 0.0],
            "gyro_covariance_diag": [0.0, 0.0, 0.0],
            "accel_bias_xy": [0.0, 0.0],
            "accel_covariance_diag": [0.0, 0.0, 0.0],
            "accel_gravity_magnitude": 9.80665,
            "axis_sanity_passed": False,
            "axis_sanity_notes": "",
            "yaw_trust": False,
            "calibrated_at": None,
            "calibrated_by": None,
        },
        "base": {
            "linear_scale": 1.0,
            "wheel_radius_nominal": None,
            "wheel_radius_corrected": None,
            "angular_scale": 1.0,
            "wheel_separation_multiplier_nominal": None,
            "wheel_separation_multiplier_corrected": None,
            "calibrated_at": None,
            "calibrated_by": None,
        },
        "gripper": {
            "servo_id": None,
            "positions": [],
            "linear_fit": None,
            "calibrated_at": None,
            "calibrated_by": None,
        },
        "meta": {
            "schema_version": 1,
            "robot_name": "wildbot",
        },
    }


def _yaml() -> YAML:
    y = YAML()
    y.indent(mapping=2, sequence=4, offset=2)
    y.preserve_quotes = True
    return y


def get_path() -> Path:
    env = os.environ.get(_ENV_VAR)
    if env:
        return Path(env).expanduser()
    return _DEFAULT_PATH


def load() -> CommentedMap:
    """Load YAML from disk; create with default template if missing."""
    path = get_path()
    yaml = _yaml()
    if not path.exists():
        path.parent.mkdir(parents=True, exist_ok=True)
        with path.open("w") as f:
            yaml.dump(_default_template(), f)
    with path.open("r") as f:
        data = yaml.load(f)
    if data is None:
        data = CommentedMap()
    return data


def _now_iso() -> str:
    return datetime.datetime.now(datetime.timezone.utc).isoformat(timespec="seconds")


def update_section(
    section: str,
    values: dict[str, Any],
    calibrated_by: str | None = None,
) -> Path:
    """Partial-update a top-level section, stamp calibrated_at + calibrated_by, write back."""
    path = get_path()
    yaml = _yaml()
    data = load()
    if section not in data or not isinstance(data[section], dict):
        data[section] = CommentedMap()

    sect = data[section]
    for k, v in values.items():
        sect[k] = v

    if "calibrated_at" in sect or section in {"imu", "base", "gripper"}:
        sect["calibrated_at"] = _now_iso()
        sect["calibrated_by"] = calibrated_by or getpass.getuser()

    with path.open("w") as f:
        yaml.dump(data, f)
    return path


def get_section(section: str) -> dict[str, Any]:
    data = load()
    raw = data.get(section, {})
    if isinstance(raw, CommentedMap):
        return dict(raw)
    return raw or {}
