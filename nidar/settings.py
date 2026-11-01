"""
Loaders for measured and tunable values stored in config/*.json.

The camera mounting matrix is NOT loaded from a file. It is the approved
Mounting A definition and stays in nidar/camera_config.py.
"""

import json
from pathlib import Path

import numpy as np

from nidar.camera_config import CameraConfig
from nidar.range_guard import RangeGuardAction, RangeGuardConfig

CAMERA_KEYS = frozenset(
    {
        "image_width",
        "image_height",
        "fx",
        "fy",
        "cx",
        "cy",
        "offset_cam_in_body_m",
        "distortion_enabled",
        "k1",
        "k2",
        "p1",
        "p2",
    }
)
RANGE_GUARD_KEYS = frozenset(
    {
        "enabled",
        "max_off_nadir_deg",
        "medium_confidence_off_nadir_deg",
        "action_on_exceed_max_off_nadir",
    }
)
TARGET_LOGIC_KEYS = frozenset({"reassignment"})
REASSIGNMENT_KEYS = frozenset(
    {"low_confidence_below", "high_confidence_at", "min_margin"}
)


class SettingsError(ValueError):
    """A settings file is missing, malformed, or out of range."""


def _read_object(path, required, allowed):
    path = Path(path)
    try:
        raw = json.loads(path.read_text(encoding="utf-8"))
    except FileNotFoundError as exc:
        raise SettingsError(f"Settings file not found: {path}") from exc
    except json.JSONDecodeError as exc:
        raise SettingsError(f"Invalid JSON in {path}: {exc}") from exc
    if not isinstance(raw, dict):
        raise SettingsError(f"{path}: top level must be a JSON object")
    missing = required - raw.keys()
    if missing:
        raise SettingsError(f"{path}: missing keys {sorted(missing)}")
    unknown = raw.keys() - allowed
    if unknown:
        raise SettingsError(f"{path}: unknown keys {sorted(unknown)}")
    return raw


def _number(path, key, value):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise SettingsError(f"{path}: {key} must be a number")
    return float(value)


def load_camera_config(path) -> CameraConfig:
    """Load a CameraConfig. Mounting matrix comes from camera_config.py."""
    from nidar.camera_config import SYNTHETIC_CAMERA_CONFIG

    raw = _read_object(path, CAMERA_KEYS, CAMERA_KEYS)
    offset = raw["offset_cam_in_body_m"]
    if not isinstance(offset, list) or len(offset) != 3:
        raise SettingsError(
            f"{path}: offset_cam_in_body_m must be a list of 3 numbers"
        )
    offset = tuple(
        _number(path, "offset_cam_in_body_m", value) for value in offset
    )

    if not isinstance(raw["distortion_enabled"], bool):
        raise SettingsError(f"{path}: distortion_enabled must be true or false")

    return CameraConfig(
        image_width=int(_number(path, "image_width", raw["image_width"])),
        image_height=int(_number(path, "image_height", raw["image_height"])),
        fx=_number(path, "fx", raw["fx"]),
        fy=_number(path, "fy", raw["fy"]),
        cx=_number(path, "cx", raw["cx"]),
        cy=_number(path, "cy", raw["cy"]),
        offset_cam_in_body=offset,
        R_cam_to_body=SYNTHETIC_CAMERA_CONFIG.R_cam_to_body.copy(),
        distortion_enabled=raw["distortion_enabled"],
        k1=_number(path, "k1", raw["k1"]),
        k2=_number(path, "k2", raw["k2"]),
        p1=_number(path, "p1", raw["p1"]),
        p2=_number(path, "p2", raw["p2"]),
    )


def load_range_guard_config(path) -> RangeGuardConfig:
    """Load a RangeGuardConfig. Angles are given in degrees in the file."""
    raw = _read_object(path, RANGE_GUARD_KEYS, RANGE_GUARD_KEYS)

    if not isinstance(raw["enabled"], bool):
        raise SettingsError(f"{path}: enabled must be true or false")

    max_deg = _number(path, "max_off_nadir_deg", raw["max_off_nadir_deg"])
    medium_deg = _number(
        path,
        "medium_confidence_off_nadir_deg",
        raw["medium_confidence_off_nadir_deg"],
    )
    if not (0.0 < max_deg <= medium_deg <= 90.0):
        raise SettingsError(
            f"{path}: require 0 < max_off_nadir_deg <= "
            "medium_confidence_off_nadir_deg <= 90"
        )

    try:
        action = RangeGuardAction(raw["action_on_exceed_max_off_nadir"])
    except ValueError as exc:
        allowed = [action.value for action in RangeGuardAction]
        raise SettingsError(
            f"{path}: action_on_exceed_max_off_nadir must be one of {allowed}"
        ) from exc

    return RangeGuardConfig(
        enabled=raw["enabled"],
        max_off_nadir_angle_rad=float(np.radians(max_deg)),
        medium_confidence_off_nadir_rad=float(np.radians(medium_deg)),
        action_on_exceed_max_off_nadir=action,
    )


def _section(path, raw, key, allowed):
    section = raw.get(key)
    if not isinstance(section, dict):
        raise SettingsError(f"{path}: '{key}' must be an object")
    missing = allowed - section.keys()
    if missing:
        raise SettingsError(f"{path}: {key} missing {sorted(missing)}")
    unknown = section.keys() - allowed
    if unknown:
        raise SettingsError(f"{path}: {key} has unknown keys {sorted(unknown)}")
    return section


def load_reassignment_policy(path):
    """Load ReassignmentPolicy from config/target_logic.json."""
    from nidar.mission import ReassignmentPolicy

    raw = _read_object(path, TARGET_LOGIC_KEYS, TARGET_LOGIC_KEYS)
    reassignment = _section(path, raw, "reassignment", REASSIGNMENT_KEYS)
    try:
        return ReassignmentPolicy(
            low_confidence_below=_number(
                path, "low_confidence_below", reassignment["low_confidence_below"]
            ),
            high_confidence_at=_number(
                path, "high_confidence_at", reassignment["high_confidence_at"]
            ),
            min_margin=_number(path, "min_margin", reassignment["min_margin"]),
        )
    except ValueError as exc:
        raise SettingsError(f"{path}: reassignment: {exc}") from exc
