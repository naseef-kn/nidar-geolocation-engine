import pytest

from nidar.drone import MessageNotAllowedError, Drone
from nidar.drone_config import DroneConfig
from nidar.transport import DuplicateEndpointError, GCS_ID, InMemoryTransport, Message


@pytest.fixture
def transport():
    return InMemoryTransport()


def make_drone(transport, drone_id="alpha", capabilities=("scout", "delivery")):
    return Drone(DroneConfig(drone_id=drone_id, capabilities=capabilities), transport)


# --- Registration ---

def test_drone_registers_with_transport(transport):
    make_drone(transport, "alpha")
    transport.send(Message("gcs", "alpha", "mission_start"))
    assert transport.receive("alpha").msg_type == "mission_start"


def test_duplicate_drone_id_rejected(transport):
    make_drone(transport, "alpha")
    with pytest.raises(DuplicateEndpointError):
        make_drone(transport, "alpha")


# --- Sending permissions ---

def test_scout_can_send_target_found(transport):
    drone = make_drone(transport, capabilities=("scout",))
    drone.send_to_gcs("target_found", {"lat": 12.97, "lon": 77.59})
    received = transport.receive(GCS_ID)
    assert received.msg_type == "target_found"
    assert received.payload == {"lat": 12.97, "lon": 77.59}
    assert received.sender == "alpha"


def test_delivery_can_send_delivery_complete(transport):
    drone = make_drone(transport, capabilities=("delivery",))
    drone.send_to_gcs("delivery_complete")
    assert transport.receive(GCS_ID).msg_type == "delivery_complete"


def test_any_drone_can_send_heartbeat(transport):
    drone = make_drone(transport, capabilities=("scout",))
    drone.send_to_gcs("heartbeat")
    assert transport.receive(GCS_ID).msg_type == "heartbeat"


def test_scout_cannot_send_delivery_message(transport):
    drone = make_drone(transport, capabilities=("scout",))
    with pytest.raises(MessageNotAllowedError):
        drone.send_to_gcs("delivery_complete")


def test_delivery_only_drone_cannot_send_target_found(transport):
    drone = make_drone(transport, capabilities=("delivery",))
    with pytest.raises(MessageNotAllowedError):
        drone.send_to_gcs("target_found")


def test_rejected_message_is_not_sent(transport):
    drone = make_drone(transport, capabilities=("scout",))
    with pytest.raises(MessageNotAllowedError):
        drone.send_to_gcs("delivery_complete")
    assert transport.receive(GCS_ID) is None


def test_unknown_message_type_rejected(transport):
    drone = make_drone(transport)
    with pytest.raises(MessageNotAllowedError):
        drone.send_to_gcs("self_destruct")


def test_payload_defaults_to_empty_dict(transport):
    drone = make_drone(transport)
    drone.send_to_gcs("heartbeat")
    assert transport.receive(GCS_ID).payload == {}


# --- Polling ---

def test_poll_returns_all_messages_oldest_first(transport):
    drone = make_drone(transport, "alpha")
    transport.send(Message("gcs", "alpha", "mission_start", {"seq": 1}))
    transport.send(Message("gcs", "alpha", "mission_update", {"seq": 2}))
    messages = drone.poll()
    assert [m.payload["seq"] for m in messages] == [1, 2]


def test_poll_empties_inbox(transport):
    drone = make_drone(transport, "alpha")
    transport.send(Message("gcs", "alpha", "mission_start"))
    drone.poll()
    assert drone.poll() == []


def test_poll_empty_inbox_returns_empty_list(transport):
    drone = make_drone(transport, "alpha")
    assert drone.poll() == []


def test_poll_only_returns_own_messages(transport):
    alpha = make_drone(transport, "alpha")
    make_drone(transport, "bravo")
    transport.send(Message("gcs", "bravo", "mission_start"))
    assert alpha.poll() == []


# --- Two drones talking through GCS ---

def test_two_drones_end_to_end(transport):
    alpha = make_drone(transport, "alpha", capabilities=("scout",))
    bravo = make_drone(transport, "bravo", capabilities=("delivery",))

    alpha.send_to_gcs("target_found", {"id": 1})
    gcs_inbox = transport.receive(GCS_ID)
    assert gcs_inbox.sender == "alpha"

    transport.send(Message(GCS_ID, "bravo", "mission_start", {"target": 1}))
    bravo_inbox = bravo.poll()
    assert len(bravo_inbox) == 1
    assert bravo_inbox[0].payload == {"target": 1}
