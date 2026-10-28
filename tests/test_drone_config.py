import json

import pytest

from nidar.drone_config import DroneConfig, DroneConfigError, load_drone_config


def write_json(tmp_path, data):
    path = tmp_path / "drone.json"
    path.write_text(json.dumps(data), encoding="utf-8")
    return path


def valid_data(**overrides):
    data = {"drone_id": "alpha", "capabilities": ["scout", "delivery"]}
    data.update(overrides)
    return data


# --- Valid configurations ---

def test_valid_config_loads(tmp_path):
    cfg = load_drone_config(write_json(tmp_path, valid_data()))
    assert cfg.drone_id == "alpha"
    assert cfg.capabilities == ("scout", "delivery")


def test_defaults_applied(tmp_path):
    cfg = load_drone_config(write_json(tmp_path, valid_data()))
    assert cfg.telemetry_source == "simulated"
    assert cfg.camera_calibration_file is None


def test_any_valid_name_accepted():
    cfg = DroneConfig(drone_id="charlie_3", capabilities=("scout",))
    assert cfg.drone_id == "charlie_3"


def test_can_reports_capabilities():
    cfg = DroneConfig(drone_id="alpha", capabilities=("scout",))
    assert cfg.can("scout") is True
    assert cfg.can("delivery") is False


def test_shipped_config_files_load():
    from pathlib import Path
    root = Path(__file__).resolve().parent.parent
    alpha = load_drone_config(root / "config" / "alpha.json")
    bravo = load_drone_config(root / "config" / "bravo.json")
    assert alpha.drone_id == "alpha"
    assert bravo.drone_id == "bravo"
    assert alpha.can("scout") and alpha.can("delivery")


# --- Invalid drone IDs ---

@pytest.mark.parametrize("bad_id", ["Alpha", "alpha one", "1alpha", "", "a" * 33, "alpha-1"])
def test_invalid_drone_id_rejected(bad_id):
    with pytest.raises(DroneConfigError, match="Invalid drone_id"):
        DroneConfig(drone_id=bad_id, capabilities=("scout",))


# --- Invalid capabilities ---

def test_empty_capabilities_rejected():
    with pytest.raises(DroneConfigError, match="must not be empty"):
        DroneConfig(drone_id="alpha", capabilities=())


def test_unknown_capability_rejected():
    with pytest.raises(DroneConfigError, match="Unknown capabilities"):
        DroneConfig(drone_id="alpha", capabilities=("scout", "bomber"))


def test_duplicate_capability_rejected():
    with pytest.raises(DroneConfigError, match="Duplicate"):
        DroneConfig(drone_id="alpha", capabilities=("scout", "scout"))


# --- Invalid telemetry source ---

def test_unknown_telemetry_source_rejected():
    with pytest.raises(DroneConfigError, match="Unknown telemetry_source"):
        DroneConfig(drone_id="alpha", capabilities=("scout",), telemetry_source="radio")


# --- File-level errors ---

def test_missing_file_rejected(tmp_path):
    with pytest.raises(DroneConfigError, match="not found"):
        load_drone_config(tmp_path / "nope.json")


def test_invalid_json_rejected(tmp_path):
    path = tmp_path / "drone.json"
    path.write_text("{not json", encoding="utf-8")
    with pytest.raises(DroneConfigError, match="Invalid JSON"):
        load_drone_config(path)


def test_missing_required_key_rejected(tmp_path):
    with pytest.raises(DroneConfigError, match="missing required keys"):
        load_drone_config(write_json(tmp_path, {"drone_id": "alpha"}))


def test_typo_key_rejected(tmp_path):
    data = valid_data(telemetry_sourse="mavlink")
    with pytest.raises(DroneConfigError, match="unknown keys"):
        load_drone_config(write_json(tmp_path, data))


def test_capabilities_must_be_list(tmp_path):
    data = valid_data(capabilities="scout")
    with pytest.raises(DroneConfigError, match="must be a list"):
        load_drone_config(write_json(tmp_path, data))