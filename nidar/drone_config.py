"""
Per-drone configuration for NIDAR.

A DroneConfig describes what a drone *can* do (its capabilities), where its
telemetry comes from, and how much packet payload it can carry under the
total weight limit. The active task is assigned later by the mission planner.
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
        "max_total_kg",
    }
)
REQUIRED_KEYS = frozenset({"drone_id", "capabilities"})

DEFAULT_MAX_TOTAL_KG = 25.0

_DRONE_ID_PATTERN = re.compile(r"^[a-z][a-z0-9_]{0,31}$")


class DroneConfigError(ValueError):
    """Raised when a drone configuration is invalid."""


def payload_capacity(max_total_kg: float, airframe_kg: float, packet_kg: float) -> int:
    """
    Number of whole packets that fit under the total weight limit.

    Rounds down, so the limit is never exceeded.
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


@dataclass(frozen=True)
class DroneConfig:
    drone_id: str
    capabilities: tuple
    telemetry_source: str = "simulated"
    camera_calibration_file: str | None = None
    airframe_kg: float = 0.0
    packet_kg: float = 1.0
    max_total_kg: float = DEFAULT_MAX_TOTAL_KG

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
        # Validates weights and raises if the airframe exceeds the limit.
        payload_capacity(self.max_total_kg, self.airframe_kg, self.packet_kg)

    @property
    def payload_capacity(self) -> int:
        """Maximum number of packets this drone may carry."""
        return payload_capacity(self.max_total_kg, self.airframe_kg, self.packet_kg)

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
        max_total_kg=raw.get("max_total_kg", DEFAULT_MAX_TOTAL_KG),
    )
