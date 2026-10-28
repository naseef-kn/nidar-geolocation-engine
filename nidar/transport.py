"""
Message transport between drones and the Ground Control Station (GCS).

Drones never talk to each other directly through this module. All messages
are routed through the transport, which is the single place that knows who
is registered and where each message goes.

InMemoryTransport keeps everything in one Python process. It is used for
tests and simulation. A radio or network transport can later replace it
without changing the drone code, because both implement the same interface.
"""

import re
from abc import ABC, abstractmethod
from collections import deque
from dataclasses import dataclass, field
from typing import Any

GCS_ID = "gcs"
RESERVED_IDS = frozenset({GCS_ID})

# Same format as drone IDs in drone_config.py
_ENDPOINT_ID_PATTERN = re.compile(r"^[a-z][a-z0-9_]{0,31}$")


class TransportError(Exception):
    """Base class for all transport errors."""


class InvalidEndpointIdError(TransportError):
    """An endpoint ID does not match the allowed format."""


class ReservedEndpointError(TransportError):
    """An endpoint ID is reserved and cannot be registered by a drone."""


class DuplicateEndpointError(TransportError):
    """An endpoint with this ID is already registered."""


class UnknownSenderError(TransportError):
    """A message was sent from an endpoint that is not registered."""


class UnknownRecipientError(TransportError):
    """A message was addressed to an endpoint that is not registered."""


class UnknownEndpointError(TransportError):
    """An operation referred to an endpoint that is not registered."""


def validate_endpoint_id(endpoint_id: str) -> None:
    """Raise if an endpoint ID is malformed or reserved."""
    if not _ENDPOINT_ID_PATTERN.match(endpoint_id):
        raise InvalidEndpointIdError(
            f"Invalid endpoint ID {endpoint_id!r}: use lowercase letters, "
            "digits and underscores, starting with a letter (max 32 chars)"
        )
    if endpoint_id in RESERVED_IDS:
        raise ReservedEndpointError(
            f"Endpoint ID {endpoint_id!r} is reserved for the GCS"
        )


@dataclass(frozen=True)
class Message:
    """A single message between two endpoints."""

    sender: str
    recipient: str
    msg_type: str
    payload: dict[str, Any] = field(default_factory=dict)

    def __post_init__(self):
        if not self.msg_type:
            raise ValueError("msg_type must not be empty")


class Transport(ABC):
    """Interface every transport must implement."""

    @abstractmethod
    def register(self, endpoint_id: str) -> None:
        """Make an endpoint known to the transport so it can send and receive."""

    @abstractmethod
    def send(self, message: Message) -> None:
        """Deliver a message to its recipient's inbox."""

    @abstractmethod
    def receive(self, endpoint_id: str) -> Message | None:
        """Return the oldest message for this endpoint, or None if its inbox is empty."""


class InMemoryTransport(Transport):
    """Transport that keeps one FIFO inbox per endpoint in memory."""

    def __init__(self):
        self._inboxes: dict[str, deque[Message]] = {}
        # GCS always exists, so drones can reach it without registration.
        self._inboxes[GCS_ID] = deque()

    def register(self, endpoint_id: str) -> None:
        validate_endpoint_id(endpoint_id)
        if endpoint_id in self._inboxes:
            raise DuplicateEndpointError(f"Endpoint {endpoint_id!r} already registered")
        self._inboxes[endpoint_id] = deque()

    def send(self, message: Message) -> None:
        if message.sender not in self._inboxes:
            raise UnknownSenderError(f"Unknown sender {message.sender!r}")
        if message.recipient not in self._inboxes:
            raise UnknownRecipientError(f"Unknown recipient {message.recipient!r}")
        self._inboxes[message.recipient].append(message)

    def receive(self, endpoint_id: str) -> Message | None:
        if endpoint_id not in self._inboxes:
            raise UnknownEndpointError(f"Unknown endpoint {endpoint_id!r}")
        inbox = self._inboxes[endpoint_id]
        return inbox.popleft() if inbox else None
