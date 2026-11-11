import math

import pytest

from nidar.crossframe_dedup import (
    CrossFrameDeduplicator,
    horizontal_distance_m,
)

LAT, LON = 12.9716, 77.5946


def offset_m(north_m, east_m):
    """Return (lat, lon) displaced from (LAT, LON) by metres north and east."""
    dlat = math.degrees(north_m / 6_371_000.0)
    dlon = math.degrees(east_m / (6_371_000.0 * math.cos(math.radians(LAT))))
    return LAT + dlat, LON + dlon


def test_zero_distance_for_same_point():
    assert horizontal_distance_m(LAT, LON, LAT, LON) == 0.0


def test_distance_matches_offset_north():
    lat, lon = offset_m(30.0, 0.0)
    assert horizontal_distance_m(LAT, LON, lat, lon) == pytest.approx(
        30.0, rel=1e-3
    )


def test_distance_matches_offset_east():
    lat, lon = offset_m(0.0, 40.0)
    assert horizontal_distance_m(LAT, LON, lat, lon) == pytest.approx(
        40.0, rel=1e-3
    )


def test_first_target_is_admitted():
    deduplicator = CrossFrameDeduplicator(3.0)
    assert deduplicator.admit("alpha-0001", LAT, LON) is True
    assert len(deduplicator) == 1


def test_repeat_within_radius_is_rejected():
    deduplicator = CrossFrameDeduplicator(3.0)
    deduplicator.admit("alpha-0001", LAT, LON)
    lat, lon = offset_m(2.0, 0.0)
    assert deduplicator.admit("bravo-0001", lat, lon) is False
    assert len(deduplicator) == 1


def test_target_outside_radius_is_admitted():
    deduplicator = CrossFrameDeduplicator(3.0)
    deduplicator.admit("alpha-0001", LAT, LON)
    lat, lon = offset_m(10.0, 0.0)
    assert deduplicator.admit("alpha-0002", lat, lon) is True
    assert len(deduplicator) == 2


def test_find_match_returns_closest_within_radius():
    deduplicator = CrossFrameDeduplicator(5.0)
    deduplicator.admit("a", *offset_m(0.0, 0.0))
    deduplicator.admit("b", *offset_m(0.0, 6.0))
    lat, lon = offset_m(0.0, 3.5)
    assert deduplicator.find_match(lat, lon).target_id == "b"


@pytest.mark.parametrize("bad", [0.0, -1.0, float("nan"), float("inf")])
def test_invalid_radius_rejected(bad):
    with pytest.raises(ValueError):
        CrossFrameDeduplicator(bad)
