"""
Per-drone configuration for NIDAR, plus fleet-level weight checks.

A DroneConfig describes what a drone can do, where its telemetry comes from,
its weights, and how many packets it carries on this mission. The 25 kg limit
applies to the whole fleet: the sum of all drones' loaded masses must not
exceed it. The limit is read from config/fleet.json.
"""

import json
import math
import re
from dataclasses import dataclass
from pathlib import Path

VALID_CAPABILITIES = frozenset({"scout", "delivery"})
VALID_TELEMETRY_SOURCES = frozenset({"simulated", "mavlink"})
ALLOWED_KEYS = frozenset(
    {
        "drone_id",
        "capabilities",
        "telemetry_source",
        "camera_calibration_file",
        "airframe_kg",
        "packet_kg",
        "payload_packets",
    }
)
REQUIRED_KEYS = frozenset({"drone_id", "capabilities"})

# Default only. The real value is read from config/fleet.json.
FLEET_MAX_TOTAL_KG = 25.0
_WEIGHT_TOLERANCE_KG = 1e-9

_DRONE_ID_PATTERN = re.compile(r"^[a-z][a-z0-9_]{0,31}$")


class DroneConfigError(ValueError):
    """Raised when a drone configuration is invalid."""


def payload_capacity(max_total_kg: float, airframe_kg: float, packet_kg: float) -> int:
    """
    Maximum whole packets one drone can carry under max_total_kg on its own.

    Rounds down so the limit is never exceeded. This is an upper bound per
    drone; the fleet check in validate_fleet is the binding limit.
    """
    if packet_kg <= 0:
        raise DroneConfigError("packet_kg must be greater than 0")
    if airframe_kg < 0:
        raise DroneConfigError("airframe_kg must not be negative")
    spare = max_total_kg - airframe_kg
    if spare < 0:
        raise DroneConfigError(
            f"airframe_kg ({airframe_kg}) already exceeds the "
            f"{max_total_kg} kg limit"
        )
    # Small tolerance so 2.0 kg / 0.5 kg is 4 packets, not 3 from float error.
    return math.floor(spare / packet_kg + 1e-9)


def validate_fleet(configs, max_total_kg: float = FLEET_MAX_TOTAL_KG) -> float:
    """
    Check the whole fleet against the combined weight limit.

    Returns the total loaded mass in kg. Raises DroneConfigError if drone IDs
    repeat or the total exceeds max_total_kg.
    """
    ids = [config.drone_id for config in configs]
    duplicates = sorted({drone_id for drone_id in ids if ids.count(drone_id) > 1})
    if duplicates:
        raise DroneConfigError(f"Duplicate drone IDs in fleet: {duplicates}")

    total = sum(config.loaded_mass_kg for config in configs)
    if total > max_total_kg + _WEIGHT_TOLERANCE_KG:
        raise DroneConfigError(
            f"Fleet loaded mass {total:.3f} kg exceeds the "
            f"{max_total_kg} kg combined limit"
        )
    return total


def load_fleet_limit(path: str | Path) -> float:
    """Read the combined fleet weight limit from a JSON file."""
    path = Path(path)
    try:
        raw = json.loads(path.read_text(encoding="utf-8"))
    except FileNotFoundError as exc:
        raise DroneConfigError(f"Fleet file not found: {path}") from exc
    except json.JSONDecodeError as exc:
        raise DroneConfigError(f"Invalid JSON in {path}: {exc}") from exc

    if not isinstance(raw, dict) or set(raw.keys()) != {"max_total_kg"}:
        raise DroneConfigError(f"{path}: must contain exactly the key 'max_total_kg'")
    value = raw["max_total_kg"]
    if (
        isinstance(value, bool)
        or not isinstance(value, (int, float))
        or value <= 0
    ):
        raise DroneConfigError(f"{path}: max_total_kg must be a positive number")
    return float(value)


@dataclass(frozen=True)
class DroneConfig:
    drone_id: str
    capabilities: tuple
    telemetry_source: str = "simulated"
    camera_calibration_file: str | None = None
    airframe_kg: float = 0.0
    packet_kg: float = 1.0
    payload_packets: int = 0
    max_total_kg: float = FLEET_MAX_TOTAL_KG

    def __post_init__(self):
        if not _DRONE_ID_PATTERN.match(self.drone_id):
            raise DroneConfigError(
                f"Invalid drone_id {self.drone_id!r}: use lowercase letters, "
                "digits and underscores, starting with a letter (max 32 chars)"
            )
        if len(self.capabilities) == 0:
            raise DroneConfigError("capabilities must not be empty")
        if len(set(self.capabilities)) != len(self.capabilities):
            raise DroneConfigError(f"Duplicate capabilities: {self.capabilities}")
        unknown = set(self.capabilities) - VALID_CAPABILITIES
        if unknown:
            raise DroneConfigError(
                f"Unknown capabilities {sorted(unknown)}; "
                f"allowed: {sorted(VALID_CAPABILITIES)}"
            )
        if self.telemetry_source not in VALID_TELEMETRY_SOURCES:
            raise DroneConfigError(
                f"Unknown telemetry_source {self.telemetry_source!r}; "
                f"allowed: {sorted(VALID_TELEMETRY_SOURCES)}"
            )
        # Validates weights and raises if the airframe alone exceeds the limit.
        capacity = payload_capacity(self.max_total_kg, self.airframe_kg, self.packet_kg)

        if isinstance(self.payload_packets, bool) or not isinstance(
            self.payload_packets, int
        ):
            raise DroneConfigError("payload_packets must be a whole number")
        if self.payload_packets < 0:
            raise DroneConfigError("payload_packets must not be negative")
        if self.payload_packets > capacity:
            raise DroneConfigError(
                f"payload_packets ({self.payload_packets}) exceeds this drone's "
                f"capacity ({capacity})"
            )

    @property
    def payload_capacity(self) -> int:
        """Maximum packets this drone could carry on its own (upper bound)."""
        return payload_capacity(self.max_total_kg, self.airframe_kg, self.packet_kg)

    @property
    def loaded_mass_kg(self) -> float:
        """Airframe plus the packets this drone carries on this mission."""
        return self.airframe_kg + self.payload_packets * self.packet_kg

    def can(self, capability: str) -> bool:
        """True if this drone has the given capability."""
        return capability in self.capabilities


def load_drone_config(path: str | Path) -> DroneConfig:
    """Load and validate a drone configuration from a JSON file."""
    path = Path(path)
    try:
        raw = json.loads(path.read_text(encoding="utf-8"))
    except FileNotFoundError as exc:
        raise DroneConfigError(f"Config file not found: {path}") from exc
    except json.JSONDecodeError as exc:
        raise DroneConfigError(f"Invalid JSON in {path}: {exc}") from exc

    if not isinstance(raw, dict):
        raise DroneConfigError(f"{path}: top level must be a JSON object")

    missing = REQUIRED_KEYS - raw.keys()
    if missing:
        raise DroneConfigError(f"{path}: missing required keys {sorted(missing)}")

    unknown_keys = raw.keys() - ALLOWED_KEYS
    if unknown_keys:
        # Catches typos such as "telemetry_sourse"
        raise DroneConfigError(f"{path}: unknown keys {sorted(unknown_keys)}")

    if not isinstance(raw["capabilities"], list):
        raise DroneConfigError(f"{path}: capabilities must be a list")

    return DroneConfig(
        drone_id=raw["drone_id"],
        capabilities=tuple(raw["capabilities"]),
        telemetry_source=raw.get("telemetry_source", "simulated"),
        camera_calibration_file=raw.get("camera_calibration_file"),
        airframe_kg=raw.get("airframe_kg", 0.0),
        packet_kg=raw.get("packet_kg", 1.0),
        payload_packets=raw.get("payload_packets", 0),
    )
