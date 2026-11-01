"""
Stable target IDs for one drone's detections.

IDs have the form "<drone_id>-<counter>", for example "alpha-0001".
The drone ID prefix means IDs from different drones never collide.
"""

import re

_DRONE_ID_PATTERN = re.compile(r"^[a-z][a-z0-9_]{0,31}$")


class TargetIdFactory:
    def __init__(self, drone_id: str):
        if not _DRONE_ID_PATTERN.match(drone_id):
            raise ValueError(f"invalid drone_id for target IDs: {drone_id!r}")
        self._prefix = drone_id
        self._next = 1

    def new_id(self) -> str:
        target_id = f"{self._prefix}-{self._next:04d}"
        self._next += 1
        return target_id
