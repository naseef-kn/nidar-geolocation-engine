import json

import pytest

from nidar.drone_config import (
    DroneConfig,
    DroneConfigError,
    load_drone_config,
    payload_capacity,
)


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


# --- Payload capacity ---

def test_capacity_rounds_down():
    # 25 - 10 = 15 kg spare, 4 kg per packet -> 3 packets (not 3.75 rounded up)
    assert payload_capacity(25.0, 10.0, 4.0) == 3


def test_capacity_exact_fit():
    assert payload_capacity(25.0, 10.0, 2.5) == 6


def test_capacity_zero_when_airframe_is_heavy():
    assert payload_capacity(25.0, 24.0, 2.0) == 0


def test_capacity_float_tolerance():
    # 2.0 kg spare / 0.5 kg packets must be 4, not 3 from float error
    assert payload_capacity(12.0, 10.0, 0.5) == 4


def test_airframe_over_limit_rejected():
    with pytest.raises(DroneConfigError, match="exceeds"):
        payload_capacity(25.0, 26.0, 1.0)


def test_nonpositive_packet_rejected():
    with pytest.raises(DroneConfigError, match="packet_kg"):
        payload_capacity(25.0, 10.0, 0.0)


def test_negative_airframe_rejected():
    with pytest.raises(DroneConfigError, match="airframe_kg"):
        payload_capacity(25.0, -1.0, 1.0)


def test_drone_config_exposes_capacity():
    cfg = DroneConfig(
        drone_id="alpha",
        capabilities=("delivery",),
        airframe_kg=10.0,
        packet_kg=2.0,
    )
    assert cfg.payload_capacity == 7


def test_drone_config_rejects_airframe_over_limit():
    with pytest.raises(DroneConfigError):
        DroneConfig(
            drone_id="alpha",
            capabilities=("scout",),
            airframe_kg=26.0,
            packet_kg=1.0,
        )


def test_loaded_config_capacity_from_json(tmp_path):
    data = valid_data(airframe_kg=10.0, packet_kg=2.5)
    cfg = load_drone_config(write_json(tmp_path, data))
    assert cfg.payload_capacity == 6


def test_shipped_configs_have_capacity():
    from pathlib import Path

    root = Path(__file__).resolve().parent.parent
    alpha = load_drone_config(root / "config" / "alpha.json")
    bravo = load_drone_config(root / "config" / "bravo.json")

    for config in (alpha, bravo):
        assert config.payload_capacity == payload_capacity(
            config.max_total_kg,
            config.airframe_kg,
            config.packet_kg,
        )