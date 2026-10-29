"""
A drone as a participant in the NIDAR message system.

A Drone knows its own ID and capabilities, registers itself with the
transport, and sends and receives messages through it. It contains no
mission logic; task assignment is handled elsewhere.
"""

from typing import Any

from nidar.drone_config import DroneConfig
from nidar.transport import GCS_ID, Message, Transport

# Which message types each capability is allowed to send.
# heartbeat is allowed for every drone.
MESSAGE_TYPES_BY_CAPABILITY = {
    "scout": frozenset({"target_found", "scout_status"}),
    "delivery": frozenset(
        {"delivery_started", "delivery_dropping", "delivery_complete"}
    ),
}
ALWAYS_ALLOWED = frozenset({"heartbeat"})


class MessageNotAllowedError(Exception):
    """The drone tried to send a message type its capabilities do not permit."""


class Drone:
    def __init__(self, config: DroneConfig, transport: Transport) -> None:
        self.config = config
        self._transport = transport
        self._transport.register(config.drone_id)

    @property
    def drone_id(self) -> str:
        return self.config.drone_id

    def allowed_message_types(self) -> frozenset[str]:
        allowed = set(ALWAYS_ALLOWED)
        for capability in self.config.capabilities:
            allowed |= MESSAGE_TYPES_BY_CAPABILITY[capability]
        return frozenset(allowed)

    def send_to_gcs(
        self, msg_type: str, payload: dict[str, Any] | None = None
    ) -> None:
        if msg_type not in self.allowed_message_types():
            raise MessageNotAllowedError(
                f"Drone {self.drone_id!r} with capabilities "
                f"{self.config.capabilities} may not send {msg_type!r}"
            )
        self._transport.send(
            Message(
                sender=self.drone_id,
                recipient=GCS_ID,
                msg_type=msg_type,
                payload=payload or {},
            )
        )

    def poll(self) -> list[Message]:
        """Return all pending messages for this drone, oldest first."""
        messages = []
        while (message := self._transport.receive(self.drone_id)) is not None:
            messages.append(message)
        return messages
