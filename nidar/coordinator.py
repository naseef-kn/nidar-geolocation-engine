"""
Coordinator: runs at the GCS, owns the target registry, and decides assignments.

Drones report facts and execute. The coordinator decides which drone delivers
to which target, and when a delivery should be switched to a better target.
"""

from nidar.mission import (
    ReassignmentPolicy,
    Target,
    TargetRegistry,
    TargetStatus,
)
from nidar.transport import GCS_ID, Message, Transport


class Coordinator:
    def __init__(
        self, transport: Transport, policy: ReassignmentPolicy | None = None
    ) -> None:
        self._transport = transport
        self._policy = policy or ReassignmentPolicy()
        self.registry = TargetRegistry()
        # Latest payload count each drone reported. Unknown drones are absent.
        self._payload: dict[str, int] = {}
        # Drones this coordinator may assign deliveries to.
        self._known_drones: set[str] = set()
        # Alerts sent so far, as (drone_id, target_id). Kept for tests and debugging.
        self.alerts: list[tuple[str, str]] = []

    def register_drone(self, drone_id: str, payload_capacity: int) -> None:
        """
        Record a drone's starting payload so it can be assigned deliveries.

        This does not register the drone with the transport. The drone must
        be registered with the transport separately (the Drone class does this).
        """
        self._known_drones.add(drone_id)
        self._payload[drone_id] = payload_capacity

    def payload_remaining(self, drone_id: str) -> int:
        return self._payload.get(drone_id, 0)

    def process_inbox(self) -> None:
        """Handle every pending message addressed to the GCS."""
        while (message := self._transport.receive(GCS_ID)) is not None:
            self._handle(message)

    def _handle(self, message: Message) -> None:
        handler = {
            "target_found": self._on_target_found,
            "delivery_started": self._on_delivery_started,
            "delivery_dropping": self._on_delivery_dropping,
            "delivery_complete": self._on_delivery_complete,
            "scout_status": self._on_scout_status,
        }.get(message.msg_type)
        if handler is not None:
            handler(message)

    # --- Inbound handlers ---

    def _on_target_found(self, message: Message) -> None:
        p = message.payload
        target = Target(
            target_id=p["target_id"],
            latitude=p["latitude"],
            longitude=p["longitude"],
            confidence=p["confidence"],
        )
        self.registry.add(target)
        self._maybe_alert_for(
            target,
            reporter=message.sender,
            phase=p.get("phase", "outbound"),
        )

    def _on_delivery_started(self, message: Message) -> None:
        self._set_status(message.payload["target_id"], TargetStatus.IN_DELIVERY)

    def _on_delivery_dropping(self, message: Message) -> None:
        self._set_status(message.payload["target_id"], TargetStatus.DROPPING)

    def _on_delivery_complete(self, message: Message) -> None:
        target_id = message.payload["target_id"]
        self._set_status(target_id, TargetStatus.DELIVERED)
        # The drone's own report is authoritative for payload.
        drone_id = message.sender
        if "payload_remaining" in message.payload:
            self._payload[drone_id] = message.payload["payload_remaining"]

    def _on_scout_status(self, message: Message) -> None:
        if "payload_remaining" in message.payload:
            self._payload[message.sender] = message.payload["payload_remaining"]

    # --- Assignment ---

    def assign_new_delivery(self, target_id: str) -> str | None:
        """
        Assign an unassigned target to the drone with the most payload left.

        Returns the chosen drone ID, or None if no drone has payload left.
        """
        target = self.registry.get(target_id)
        if target.status is not TargetStatus.DETECTED:
            raise ValueError(f"Target {target_id!r} is not available for assignment")

        candidates = [d for d in self._known_drones if self._payload.get(d, 0) > 0]
        if not candidates:
            return None
        # Most payload first; drone ID breaks ties so the choice is deterministic.
        drone_id = max(candidates, key=lambda d: (self._payload[d], -ord(d[0]), d))

        self.registry.update(
            Target(
                target_id=target.target_id,
                latitude=target.latitude,
                longitude=target.longitude,
                confidence=target.confidence,
                status=TargetStatus.ASSIGNED,
                assigned_to=drone_id,
            )
        )
        self._send_to(drone_id, "mission_start", {"target_id": target_id})
        return drone_id

    def _maybe_alert_for(
        self, target: Target, reporter: str, phase: str
    ) -> str | None:
        """
        Alert another drone to a high-confidence target seen on a return leg,
        when the reporting drone has no payload left.

        Returns the alerted drone ID, or None if no alert was sent.
        """
        if phase != "wayback":
            return None
        if target.confidence < self._policy.high_confidence_at:
            return None
        if self.payload_remaining(reporter) > 0:
            return None

        candidates = sorted(
            d for d in self._known_drones
            if d != reporter and self._payload.get(d, 0) > 0
        )
        if not candidates:
            return None
        # max() returns the first maximum, so ties go to the alphabetically first drone.
        drone_id = max(candidates, key=lambda d: self._payload[d])

        self._send_to(drone_id, "target_alert", {
            "target_id": target.target_id,
            "latitude": target.latitude,
            "longitude": target.longitude,
            "confidence": target.confidence,
            "reported_by": reporter,
        })
        self.alerts.append((drone_id, target.target_id))
        return drone_id

    # --- Helpers ---

    def _set_status(self, target_id: str, status: TargetStatus) -> None:
        target = self.registry.get(target_id)
        self.registry.update(
            Target(
                target_id=target.target_id,
                latitude=target.latitude,
                longitude=target.longitude,
                confidence=target.confidence,
                status=status,
                assigned_to=target.assigned_to,
            )
        )

    def _send_to(self, drone_id: str, msg_type: str, payload: dict) -> None:
        self._transport.send(
            Message(
                sender=GCS_ID,
                recipient=drone_id,
                msg_type=msg_type,
                payload=payload,
            )
        )
