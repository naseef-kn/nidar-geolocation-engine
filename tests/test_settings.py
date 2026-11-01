import json
from pathlib import Path

import numpy as np
import pytest

from nidar.camera_config import SYNTHETIC_CAMERA_CONFIG
from nidar.range_guard import RangeGuardAction
from nidar.settings import SettingsError, load_camera_config, load_range_guard_config

ROOT = Path(__file__).resolve().parent.parent


def write(tmp_path, data, name="s.json"):
    path = tmp_path / name
    path.write_text(json.dumps(data), encoding="utf-8")
    return path


def camera_data(**overrides):
    data = json.loads((ROOT / "config" / "camera.json").read_text(encoding="utf-8"))
    data.update(overrides)
    return data


def guard_data(**overrides):
    data = json.loads(
        (ROOT / "config" / "range_guard.json").read_text(encoding="utf-8")
    )
    data.update(overrides)
    return data


# --- Shipped files match the current hardcoded values ---

def test_shipped_camera_matches_synthetic_config():
    config = load_camera_config(ROOT / "config" / "camera.json")
    assert config.image_width == SYNTHETIC_CAMERA_CONFIG.image_width
    assert config.fx == SYNTHETIC_CAMERA_CONFIG.fx
    assert config.cx == SYNTHETIC_CAMERA_CONFIG.cx
    assert config.offset_cam_in_body == SYNTHETIC_CAMERA_CONFIG.offset_cam_in_body


def test_shipped_range_guard_matches_defaults():
    from nidar.range_guard import RangeGuardConfig

    loaded = load_range_guard_config(ROOT / "config" / "range_guard.json")
    default = RangeGuardConfig()
    assert loaded.max_off_nadir_angle_rad == pytest.approx(
        default.max_off_nadir_angle_rad
    )
    assert loaded.medium_confidence_off_nadir_rad == pytest.approx(
        default.medium_confidence_off_nadir_rad
    )
    assert (
        loaded.action_on_exceed_max_off_nadir
        is default.action_on_exceed_max_off_nadir
    )


# --- Camera validation ---

def test_camera_unknown_key_rejected(tmp_path):
    with pytest.raises(SettingsError, match="unknown keys"):
        load_camera_config(write(tmp_path, camera_data(fx_typo=1.0)))


def test_camera_missing_key_rejected(tmp_path):
    data = camera_data()
    del data["fx"]
    with pytest.raises(SettingsError, match="missing"):
        load_camera_config(write(tmp_path, data))


def test_camera_offset_must_be_three_numbers(tmp_path):
    with pytest.raises(SettingsError, match="offset"):
        load_camera_config(write(tmp_path, camera_data(offset_cam_in_body_m=[0, 0])))


def test_camera_distortion_flag_must_be_boolean(tmp_path):
    with pytest.raises(SettingsError, match="distortion_enabled"):
        load_camera_config(write(tmp_path, camera_data(distortion_enabled="yes")))


def test_camera_bool_not_accepted_as_number(tmp_path):
    with pytest.raises(SettingsError, match="fx"):
        load_camera_config(write(tmp_path, camera_data(fx=True)))


# --- Range guard validation ---

def test_range_guard_converts_degrees_to_radians(tmp_path):
    config = load_range_guard_config(
        write(
            tmp_path,
            guard_data(
                max_off_nadir_deg=30.0,
                medium_confidence_off_nadir_deg=45.0,
            ),
        )
    )
    assert config.max_off_nadir_angle_rad == pytest.approx(np.radians(30.0))


def test_range_guard_order_enforced(tmp_path):
    with pytest.raises(SettingsError, match="require"):
        load_range_guard_config(
            write(
                tmp_path,
                guard_data(
                    max_off_nadir_deg=70.0,
                    medium_confidence_off_nadir_deg=60.0,
                ),
            )
        )


def test_range_guard_action_by_value(tmp_path):
    config = load_range_guard_config(
        write(tmp_path, guard_data(action_on_exceed_max_off_nadir="reject"))
    )
    assert config.action_on_exceed_max_off_nadir is RangeGuardAction.REJECT


def test_range_guard_unknown_action_rejected(tmp_path):
    with pytest.raises(SettingsError, match="action_on_exceed"):
        load_range_guard_config(
            write(tmp_path, guard_data(action_on_exceed_max_off_nadir="ignore"))
        )


def test_range_guard_enabled_must_be_boolean(tmp_path):
    with pytest.raises(SettingsError, match="enabled"):
        load_range_guard_config(write(tmp_path, guard_data(enabled=1)))
