import pytest

from nidar.coordinator import Coordinator
from nidar.mission import ReassignmentPolicy, Target, TargetStatus
from nidar.transport import GCS_ID, InMemoryTransport, Message


@pytest.fixture
def setup():
    transport = InMemoryTransport()
    transport.register("alpha")
    transport.register("bravo")
    coord = Coordinator(transport)
    coord.register_drone("alpha", payload_capacity=7)
    coord.register_drone("bravo", payload_capacity=7)
    return transport, coord


def report_target(
    transport,
    sender,
    target_id,
    confidence,
    phase="outbound",
    lat=12.97,
    lon=77.59,
):
    transport.send(Message(sender, GCS_ID, "target_found", {
        "target_id": target_id, "latitude": lat,
        "longitude": lon, "confidence": confidence, "phase": phase,
    }))


def set_payload(transport, coord, drone_id, remaining):
    transport.send(Message(drone_id, GCS_ID, "scout_status",
                           {"payload_remaining": remaining}))
    coord.process_inbox()


def drain(transport, endpoint):
    out = []
    while (m := transport.receive(endpoint)) is not None:
        out.append(m)
    return out


# --- Assignment ---

def test_assigns_to_drone_with_most_payload(setup):
    transport, coord = setup
    coord.register_drone("alpha", payload_capacity=3)
    coord.register_drone("bravo", payload_capacity=5)
    report_target(transport, "alpha", "t1", 0.7)
    coord.process_inbox()
    assert coord.assign_new_delivery("t1") == "bravo"


def test_drone_with_zero_payload_is_skipped(setup):
    transport, coord = setup
    coord.register_drone("alpha", payload_capacity=0)
    coord.register_drone("bravo", payload_capacity=2)
    report_target(transport, "alpha", "t1", 0.7)
    coord.process_inbox()
    assert coord.assign_new_delivery("t1") == "bravo"


def test_no_drone_with_payload_returns_none(setup):
    transport, coord = setup
    coord.register_drone("alpha", payload_capacity=0)
    coord.register_drone("bravo", payload_capacity=0)
    report_target(transport, "alpha", "t1", 0.7)
    coord.process_inbox()
    assert coord.assign_new_delivery("t1") is None


def test_assignment_sends_mission_start(setup):
    transport, coord = setup
    report_target(transport, "alpha", "t1", 0.7)
    coord.process_inbox()
    coord.assign_new_delivery("t1")
    msgs = drain(transport, "alpha") + drain(transport, "bravo")
    assert [m.msg_type for m in msgs] == ["mission_start"]
    assert msgs[0].payload == {"target_id": "t1"}


# --- Status transitions ---

def test_delivery_status_flow(setup):
    transport, coord = setup
    report_target(transport, "alpha", "t1", 0.7)
    coord.process_inbox()
    drone = coord.assign_new_delivery("t1")
    transport.send(Message(drone, GCS_ID, "delivery_started", {"target_id": "t1"}))
    coord.process_inbox()
    assert coord.registry.get("t1").status is TargetStatus.IN_DELIVERY
    transport.send(Message(drone, GCS_ID, "delivery_complete",
                           {"target_id": "t1", "payload_remaining": 6}))
    coord.process_inbox()
    assert coord.registry.get("t1").status is TargetStatus.DELIVERED
    assert coord.payload_remaining(drone) == 6


def test_dropping_report_blocks_switch(setup):
    transport, coord = setup
    report_target(transport, "alpha", "t1", 0.4)
    coord.process_inbox()
    drone = coord.assign_new_delivery("t1")
    transport.send(Message(drone, GCS_ID, "delivery_started", {"target_id": "t1"}))
    transport.send(Message(drone, GCS_ID, "delivery_dropping", {"target_id": "t1"}))
    coord.process_inbox()
    report_target(transport, "alpha", "t2", 0.95)
    coord.process_inbox()
    assert coord.registry.get("t1").status is TargetStatus.DROPPING
    assert coord.registry.get("t2").status is TargetStatus.DETECTED


def test_no_switch_when_delivery_is_confident(setup):
    transport, coord = setup
    report_target(transport, "alpha", "t1", 0.8)
    coord.process_inbox()
    drone = coord.assign_new_delivery("t1")
    transport.send(Message(drone, GCS_ID, "delivery_started", {"target_id": "t1"}))
    coord.process_inbox()
    report_target(transport, "alpha", "t2", 0.99)
    coord.process_inbox()
    assert coord.registry.get("t1").status is TargetStatus.IN_DELIVERY
    assert coord.registry.get("t2").status is TargetStatus.DETECTED


# --- Payload reports ---

def test_scout_status_updates_payload(setup):
    transport, coord = setup
    transport.send(Message("alpha", GCS_ID, "scout_status", {"payload_remaining": 4}))
    coord.process_inbox()
    assert coord.payload_remaining("alpha") == 4


# --- Alert rule ---

def test_wayback_high_conf_empty_reporter_alerts_other_drone(setup):
    transport, coord = setup
    set_payload(transport, coord, "alpha", 0)
    report_target(transport, "alpha", "t1", 0.95, phase="wayback")
    coord.process_inbox()
    alerts = drain(transport, "bravo")
    assert [m.msg_type for m in alerts] == ["target_alert"]
    assert alerts[0].payload["target_id"] == "t1"
    assert alerts[0].payload["reported_by"] == "alpha"
    assert coord.alerts == [("bravo", "t1")]


def test_alert_does_not_change_target_status(setup):
    transport, coord = setup
    set_payload(transport, coord, "alpha", 0)
    report_target(transport, "alpha", "t1", 0.95, phase="wayback")
    coord.process_inbox()
    t = coord.registry.get("t1")
    assert t.status is TargetStatus.DETECTED
    assert t.assigned_to is None


def test_outbound_high_conf_does_not_alert(setup):
    transport, coord = setup
    set_payload(transport, coord, "alpha", 0)
    report_target(transport, "alpha", "t1", 0.95, phase="outbound")
    coord.process_inbox()
    assert drain(transport, "bravo") == []
    assert coord.alerts == []


def test_wayback_with_reporter_payload_does_not_alert(setup):
    transport, coord = setup
    report_target(transport, "alpha", "t1", 0.95, phase="wayback")
    coord.process_inbox()
    assert coord.alerts == []


def test_wayback_below_high_confidence_does_not_alert(setup):
    transport, coord = setup
    set_payload(transport, coord, "alpha", 0)
    report_target(transport, "alpha", "t1", 0.7, phase="wayback")
    coord.process_inbox()
    assert coord.alerts == []


def test_no_alert_when_no_other_drone_has_payload(setup):
    transport, coord = setup
    set_payload(transport, coord, "alpha", 0)
    set_payload(transport, coord, "bravo", 0)
    report_target(transport, "alpha", "t1", 0.95, phase="wayback")
    coord.process_inbox()
    assert coord.alerts == []
