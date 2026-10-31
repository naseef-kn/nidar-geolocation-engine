import json

import pytest

from nidar.drone_config import (
    DroneConfig,
    DroneConfigError,
    load_drone_config,
    load_fleet_limit,
    validate_fleet,
)


def drone(drone_id, airframe=10.0, packet=0.2, packets=0):
    return DroneConfig(
        drone_id=drone_id,
        capabilities=("scout", "delivery"),
        airframe_kg=airframe,
        packet_kg=packet,
        payload_packets=packets,
    )


def test_two_drones_within_limit_pass():
    total = validate_fleet(
        [drone("alpha", packets=7), drone("bravo", packets=7)], 25.0
    )
    assert total == pytest.approx(2 * (10.0 + 7 * 0.2))


def test_fleet_over_limit_rejected():
    # 10 kg airframe + 7 x 2 kg = 24 kg each, 48 kg together
    heavy = [
        drone("alpha", packet=2.0, packets=7),
        drone("bravo", packet=2.0, packets=7),
    ]
    with pytest.raises(DroneConfigError, match="exceeds"):
        validate_fleet(heavy, 25.0)


def test_exactly_at_limit_passes():
    # Each drone weighs 12.5 kg, for a 25 kg fleet total.
    a = drone("alpha", airframe=10.0, packet=0.5, packets=5)
    b = drone("bravo", airframe=10.0, packet=0.5, packets=5)
    assert validate_fleet([a, b], 25.0) == pytest.approx(25.0)


def test_duplicate_drone_ids_rejected():
    with pytest.raises(DroneConfigError, match="Duplicate drone IDs"):
        validate_fleet([drone("alpha"), drone("alpha")], 25.0)


def test_payload_packets_above_capacity_rejected():
    # Capacity = floor((25 - 10) / 2) = 7.
    with pytest.raises(DroneConfigError, match="exceeds this drone's capacity"):
        drone("alpha", packet=2.0, packets=8)


@pytest.mark.parametrize("bad", [-1, 1.5, True])
def test_invalid_payload_packets_rejected(bad):
    with pytest.raises(DroneConfigError):
        drone("alpha", packets=bad)


def test_loaded_mass_is_airframe_plus_packets():
    config = drone("alpha", airframe=10.0, packet=0.2, packets=5)
    assert config.loaded_mass_kg == pytest.approx(11.0)


def test_payload_packets_loads_from_json(tmp_path):
    path = tmp_path / "drone.json"
    path.write_text(
        json.dumps(
            {
                "drone_id": "alpha",
                "capabilities": ["scout"],
                "airframe_kg": 10.0,
                "packet_kg": 0.2,
                "payload_packets": 4,
            }
        ),
        encoding="utf-8",
    )
    assert load_drone_config(path).payload_packets == 4


def test_fleet_limit_loads_from_file(tmp_path):
    path = tmp_path / "fleet.json"
    path.write_text(json.dumps({"max_total_kg": 25.0}), encoding="utf-8")
    assert load_fleet_limit(path) == 25.0


@pytest.mark.parametrize("bad", [0, -5, "25", True])
def test_fleet_limit_rejects_invalid_values(tmp_path, bad):
    path = tmp_path / "fleet.json"
    path.write_text(json.dumps({"max_total_kg": bad}), encoding="utf-8")
    with pytest.raises(DroneConfigError):
        load_fleet_limit(path)


def test_shipped_fleet_file_is_25_kg():
    from pathlib import Path

    root = Path(__file__).resolve().parent.parent
    assert load_fleet_limit(root / "config" / "fleet.json") == 25.0
