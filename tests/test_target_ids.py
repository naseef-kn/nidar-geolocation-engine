import pytest

from nidar.target_ids import TargetIdFactory


def test_ids_are_sequential_and_prefixed():
    factory = TargetIdFactory("alpha")
    assert [factory.new_id() for _ in range(3)] == [
        "alpha-0001",
        "alpha-0002",
        "alpha-0003",
    ]


def test_two_drones_never_collide():
    alpha = TargetIdFactory("alpha")
    bravo = TargetIdFactory("bravo")
    ids_alpha = {alpha.new_id() for _ in range(50)}
    ids_bravo = {bravo.new_id() for _ in range(50)}
    assert ids_alpha.isdisjoint(ids_bravo)


def test_each_factory_starts_at_one():
    assert TargetIdFactory("alpha").new_id() == "alpha-0001"
    assert TargetIdFactory("alpha").new_id() == "alpha-0001"


@pytest.mark.parametrize("bad", ["Alpha", "", "1alpha", "alpha one"])
def test_invalid_drone_id_rejected(bad):
    with pytest.raises(ValueError):
        TargetIdFactory(bad)
