"""
Cross-frame deduplication in metres.

Two geolocated targets are treated as the same survivor when their
horizontal separation is within duplicate_radius_m. Time-window matching
is added once frame timestamps exist (checkpoint C2).

Distance uses a local flat-Earth approximation, which is accurate for
separations of tens of metres.
"""

import math
from dataclasses import dataclass

EARTH_RADIUS_M = 6_371_000.0


def horizontal_distance_m(lat1, lon1, lat2, lon2):
    """Approximate ground distance in metres between two lat/lon points."""
    mean_lat = math.radians((lat1 + lat2) / 2.0)
    dx = math.radians(lon2 - lon1) * math.cos(mean_lat) * EARTH_RADIUS_M
    dy = math.radians(lat2 - lat1) * EARTH_RADIUS_M
    return math.hypot(dx, dy)


@dataclass(frozen=True)
class KnownTarget:
    target_id: str
    latitude: float
    longitude: float


class CrossFrameDeduplicator:
    """Keeps a list of known survivors and says whether a new one is a repeat."""

    def __init__(self, duplicate_radius_m: float):
        if not (math.isfinite(duplicate_radius_m) and duplicate_radius_m > 0):
            raise ValueError(
                "duplicate_radius_m must be a finite number greater than 0"
            )
        self.duplicate_radius_m = duplicate_radius_m
        self._known: list[KnownTarget] = []

    def find_match(self, latitude, longitude):
        """Return the matching KnownTarget, or None if the position is new."""
        best, best_d = None, None
        for known in self._known:
            distance = horizontal_distance_m(
                latitude, longitude, known.latitude, known.longitude
            )
            if distance <= self.duplicate_radius_m and (
                best_d is None or distance < best_d
            ):
                best, best_d = known, distance
        return best

    def admit(self, target_id, latitude, longitude):
        """
        Record a new survivor. Returns True if it is new, False if it matches
        an existing one (in which case nothing is stored).
        """
        if self.find_match(latitude, longitude) is not None:
            return False
        self._known.append(KnownTarget(target_id, latitude, longitude))
        return True

    def __len__(self):
        return len(self._known)
