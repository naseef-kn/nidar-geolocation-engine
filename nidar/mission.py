"""
Target model, target registry, and the delivery reassignment rule.

Pure logic: no transport and no drones. The coordinator uses these
pieces and the tests exercise them directly.
"""

from dataclasses import dataclass, replace
from enum import Enum


class MissionError(Exception):
    """Base class for mission errors."""


class UnknownTargetError(MissionError):
    """A target ID is not in the registry."""


class DuplicateTargetError(MissionError):
    """A target with this ID is already in the registry."""


class TargetStatus(Enum):
    DETECTED = "detected"
    ASSIGNED = "assigned"
    IN_DELIVERY = "in_delivery"
    DROPPING = "dropping"
    DELIVERED = "delivered"
    FLAGGED = "flagged"


@dataclass(frozen=True)
class Target:
    target_id: str
    latitude: float
    longitude: float
    confidence: float
    status: TargetStatus = TargetStatus.DETECTED
    assigned_to: str | None = None

    def __post_init__(self):
        if not 0.0 <= self.confidence <= 1.0:
            raise ValueError(
                f"confidence must be in [0, 1], got {self.confidence}"
            )


@dataclass(frozen=True)
class ReassignmentPolicy:
    low_confidence_below: float = 0.6
    high_confidence_at: float = 0.85
    min_margin: float = 0.2

    def __post_init__(self):
        if not 0.0 <= self.low_confidence_below < self.high_confidence_at <= 1.0:
            raise ValueError(
                "require 0 <= low_confidence_below < high_confidence_at <= 1"
            )
        if self.min_margin < 0.0:
            raise ValueError("min_margin must be >= 0")


class TargetRegistry:
    """Holds every known target, keyed by target_id."""

    def __init__(self):
        self._targets: dict[str, Target] = {}

    def add(self, target: Target) -> None:
        if target.target_id in self._targets:
            raise DuplicateTargetError(f"Target {target.target_id!r} already exists")
        self._targets[target.target_id] = target

    def get(self, target_id: str) -> Target:
        if target_id not in self._targets:
            raise UnknownTargetError(f"Unknown target {target_id!r}")
        return self._targets[target_id]

    def update(self, target: Target) -> None:
        if target.target_id not in self._targets:
            raise UnknownTargetError(f"Unknown target {target.target_id!r}")
        self._targets[target.target_id] = target

    def unassigned_detected(self) -> list[Target]:
        """Targets still waiting for assignment, highest confidence first."""
        detected = [
            t for t in self._targets.values()
            if t.status is TargetStatus.DETECTED
        ]
        return sorted(detected, key=lambda t: t.confidence, reverse=True)


def should_switch(
    current: Target, candidate: Target, policy: ReassignmentPolicy
) -> bool:
    """Return True only if every reassignment condition holds."""
    if current.status is not TargetStatus.IN_DELIVERY:
        return False
    if current.status is TargetStatus.DROPPING:
        return False  # payload is being released; never switch away
    if current.assigned_to is None:
        return False
    if candidate.status is not TargetStatus.DETECTED:
        return False
    if current.confidence >= policy.low_confidence_below:
        return False
    if candidate.confidence < policy.high_confidence_at:
        return False
    if candidate.confidence - current.confidence < policy.min_margin:
        return False
    return True


def switch_delivery(
    registry: TargetRegistry,
    current_id: str,
    candidate_id: str,
    policy: ReassignmentPolicy,
) -> bool:
    """
    Reassign a drone from its current target to a better candidate.

    Returns True if the switch happened, False if the rule rejected it.
    Raises UnknownTargetError if either ID is not in the registry.
    The registry is changed only when the switch happens, and both
    updates are made together.
    """
    current = registry.get(current_id)
    candidate = registry.get(candidate_id)

    if not should_switch(current, candidate, policy):
        return False

    drone_id = current.assigned_to
    registry.update(replace(current, status=TargetStatus.FLAGGED, assigned_to=None))
    registry.update(
        replace(candidate, status=TargetStatus.ASSIGNED, assigned_to=drone_id)
    )
    return True
