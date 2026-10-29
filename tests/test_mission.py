import pytest

from nidar.mission import (
    DuplicateTargetError,
    ReassignmentPolicy,
    Target,
    TargetRegistry,
    TargetStatus,
    UnknownTargetError,
    should_switch,
    switch_delivery,
)

POLICY = ReassignmentPolicy()  # low<0.6, high>=0.85, margin>=0.2


def delivering(target_id="t1", confidence=0.4, drone="bravo"):
    return Target(
        target_id=target_id,
        latitude=12.97,
        longitude=77.59,
        confidence=confidence,
        status=TargetStatus.IN_DELIVERY,
        assigned_to=drone,
    )


def detected(target_id="t2", confidence=0.9):
    return Target(
        target_id=target_id,
        latitude=12.98,
        longitude=77.60,
        confidence=confidence,
    )


def setup_registry(current, candidate):
    registry = TargetRegistry()
    registry.add(current)
    registry.add(candidate)
    return registry


# --- Reassignment happens when every condition holds ---

def test_switch_when_all_conditions_met():
    registry = setup_registry(delivering(confidence=0.4), detected(confidence=0.9))
    assert switch_delivery(registry, "t1", "t2", POLICY) is True


def test_switch_flags_old_target_and_keeps_it():
    registry = setup_registry(delivering(confidence=0.4), detected(confidence=0.9))
    switch_delivery(registry, "t1", "t2", POLICY)
    old = registry.get("t1")
    assert old.status is TargetStatus.FLAGGED
    assert old.assigned_to is None


def test_switch_assigns_candidate_to_same_drone():
    registry = setup_registry(delivering(drone="bravo"), detected())
    switch_delivery(registry, "t1", "t2", POLICY)
    new = registry.get("t2")
    assert new.status is TargetStatus.ASSIGNED
    assert new.assigned_to == "bravo"


# --- Rule rejects each unsafe or unnecessary switch ---

def test_no_switch_when_current_is_already_confident():
    registry = setup_registry(delivering(confidence=0.7), detected(confidence=0.95))
    assert switch_delivery(registry, "t1", "t2", POLICY) is False
    assert registry.get("t1").status is TargetStatus.IN_DELIVERY


def test_no_switch_when_candidate_not_high_enough():
    registry = setup_registry(delivering(confidence=0.4), detected(confidence=0.8))
    assert switch_delivery(registry, "t1", "t2", POLICY) is False


def test_no_switch_when_margin_too_small():
    # 0.4 current, 0.9 candidate: margin 0.5 is below the required 0.51.
    policy = ReassignmentPolicy(min_margin=0.51)
    registry = setup_registry(delivering(confidence=0.4), detected(confidence=0.9))
    assert switch_delivery(registry, "t1", "t2", policy) is False


def test_no_switch_when_current_not_yet_started():
    current = Target(
        target_id="t1", latitude=0, longitude=0, confidence=0.4,
        status=TargetStatus.ASSIGNED, assigned_to="bravo",
    )
    registry = setup_registry(current, detected())
    assert switch_delivery(registry, "t1", "t2", POLICY) is False


def test_no_switch_when_candidate_already_assigned():
    taken = Target(
        target_id="t2", latitude=0, longitude=0, confidence=0.9,
        status=TargetStatus.ASSIGNED, assigned_to="alpha",
    )
    registry = setup_registry(delivering(confidence=0.4), taken)
    assert switch_delivery(registry, "t1", "t2", POLICY) is False
    assert registry.get("t2").assigned_to == "alpha"


def test_unknown_candidate_raises_and_changes_nothing():
    registry = setup_registry(delivering(confidence=0.4), detected())
    with pytest.raises(UnknownTargetError):
        switch_delivery(registry, "t1", "missing", POLICY)
    assert registry.get("t1").status is TargetStatus.IN_DELIVERY


def test_should_switch_is_pure():
    current = delivering(confidence=0.4)
    candidate = detected(confidence=0.9)
    before = (current, candidate)
    should_switch(current, candidate, POLICY)
    assert (current, candidate) == before


# --- Policy and target validation ---

def test_policy_rejects_low_threshold_not_below_high():
    with pytest.raises(ValueError):
        ReassignmentPolicy(low_confidence_below=0.9, high_confidence_at=0.8)


@pytest.mark.parametrize("bad", [-0.1, 1.5])
def test_target_confidence_out_of_range_rejected(bad):
    with pytest.raises(ValueError):
        Target(target_id="t", latitude=0, longitude=0, confidence=bad)


# --- Registry ---

def test_registry_rejects_duplicate_id():
    registry = TargetRegistry()
    registry.add(detected("t1"))
    with pytest.raises(DuplicateTargetError):
        registry.add(detected("t1"))


def test_registry_unknown_id_raises():
    with pytest.raises(UnknownTargetError):
        TargetRegistry().get("nope")


def test_unassigned_detected_sorted_highest_confidence_first():
    registry = TargetRegistry()
    registry.add(detected("low", confidence=0.5))
    registry.add(detected("high", confidence=0.95))
    registry.add(detected("mid", confidence=0.7))
    ids = [t.target_id for t in registry.unassigned_detected()]
    assert ids == ["high", "mid", "low"]


# --- Drop phase protection ---

def dropping(target_id="t1", confidence=0.4, drone="bravo"):
    return Target(
        target_id=target_id,
        latitude=12.97,
        longitude=77.59,
        confidence=confidence,
        status=TargetStatus.DROPPING,
        assigned_to=drone,
    )


def test_dropping_target_is_never_switched():
    # Same conditions as a valid switch, except the current target is dropping.
    registry = setup_registry(dropping(confidence=0.4), detected(confidence=0.95))
    assert switch_delivery(registry, "t1", "t2", POLICY) is False


def test_dropping_target_registry_unchanged_after_rejected_switch():
    registry = setup_registry(dropping(confidence=0.4), detected(confidence=0.95))
    switch_delivery(registry, "t1", "t2", POLICY)
    assert registry.get("t1").status is TargetStatus.DROPPING
    assert registry.get("t1").assigned_to == "bravo"
    assert registry.get("t2").status is TargetStatus.DETECTED


def test_should_switch_false_for_dropping_even_if_other_conditions_hold():
    assert (
        should_switch(dropping(confidence=0.1), detected(confidence=0.99), POLICY)
        is False
    )
