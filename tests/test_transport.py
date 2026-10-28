import pytest

from nidar.transport import (
    GCS_ID,
    DuplicateEndpointError,
    InMemoryTransport,
    InvalidEndpointIdError,
    Message,
    ReservedEndpointError,
    UnknownEndpointError,
    UnknownRecipientError,
    UnknownSenderError,
)


@pytest.fixture
def transport():
    t = InMemoryTransport()
    t.register("alpha")
    t.register("bravo")
    return t


def msg(sender, recipient, msg_type="status", **payload):
    return Message(sender=sender, recipient=recipient, msg_type=msg_type, payload=payload)


# --- Delivery ---

def test_message_reaches_correct_recipient(transport):
    transport.send(msg("alpha", "bravo", battery=80))
    received = transport.receive("bravo")
    assert received.sender == "alpha"
    assert received.payload == {"battery": 80}


def test_message_not_delivered_to_other_endpoints(transport):
    transport.send(msg("alpha", "bravo"))
    assert transport.receive("alpha") is None
    assert transport.receive(GCS_ID) is None


def test_fifo_ordering(transport):
    transport.send(msg("alpha", "bravo", seq=1))
    transport.send(msg("alpha", "bravo", seq=2))
    assert transport.receive("bravo").payload == {"seq": 1}
    assert transport.receive("bravo").payload == {"seq": 2}


def test_empty_inbox_returns_none(transport):
    assert transport.receive("alpha") is None


def test_receive_consumes_message(transport):
    transport.send(msg("alpha", "bravo"))
    transport.receive("bravo")
    assert transport.receive("bravo") is None


# --- GCS ---

def test_gcs_is_registered_by_default():
    t = InMemoryTransport()
    t.register("alpha")
    t.send(msg("alpha", GCS_ID, "target_found"))
    assert t.receive(GCS_ID).msg_type == "target_found"


def test_gcs_can_send_to_drone(transport):
    transport.send(msg(GCS_ID, "alpha", "mission_start"))
    assert transport.receive("alpha").sender == GCS_ID


def test_drone_cannot_register_gcs_name(transport):
    with pytest.raises(ReservedEndpointError):
        transport.register(GCS_ID)


# --- Errors ---

def test_unknown_recipient_raises(transport):
    with pytest.raises(UnknownRecipientError):
        transport.send(msg("alpha", "charlie"))


def test_unknown_sender_raises(transport):
    with pytest.raises(UnknownSenderError):
        transport.send(msg("charlie", "alpha"))


def test_failed_send_leaves_inboxes_unchanged(transport):
    with pytest.raises(UnknownRecipientError):
        transport.send(msg("alpha", "charlie"))
    assert transport.receive("alpha") is None
    assert transport.receive("bravo") is None


def test_receive_unknown_endpoint_raises(transport):
    with pytest.raises(UnknownEndpointError):
        transport.receive("charlie")


def test_duplicate_registration_raises(transport):
    with pytest.raises(DuplicateEndpointError):
        transport.register("alpha")


@pytest.mark.parametrize("bad_id", ["Alpha", "alpha one", "1alpha", "", "a" * 33])
def test_invalid_endpoint_id_rejected(bad_id):
    t = InMemoryTransport()
    with pytest.raises(InvalidEndpointIdError):
        t.register(bad_id)


def test_empty_msg_type_rejected():
    with pytest.raises(ValueError, match="msg_type"):
        Message(sender="alpha", recipient="bravo", msg_type="")
