"""
Per-drone configuration for NIDAR.

A DroneConfig describes what a drone *can* do (its capabilities) and where
its telemetry comes from. The active task (scouting or delivering) is
assigned later by the mission planner, not stored here.
"""

import json
import re
from dataclasses import dataclass
from pathlib import Path

VALID_CAPABILITIES = frozenset({"scout", "delivery"})
VALID_TELEMETRY_SOURCES = frozenset({"simulated", "mavlink"})
ALLOWED_KEYS = frozenset(
    {"drone_id", "capabilities", "telemetry_source", "camera_calibration_file"}
)
REQUIRED_KEYS = frozenset({"drone_id", "capabilities"})

_DRONE_ID_PATTERN = re.compile(r"^[a-z][a-z0-9_]{0,31}$")


class DroneConfigError(ValueError):
    """Raised when a drone configuration is invalid."""


@dataclass(frozen=True)
class DroneConfig:
    drone_id: str
    capabilities: tuple
    telemetry_source: str = "simulated"
    camera_calibration_file: str | None = None

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
    )