import math

import pytest

from ekf_adapter.tracking_policy import TrackingMode, classify_tracking_mode

ISC_VALID = {"TRACKING", "PREDICTING", "PREDICTING_DEGRADED"}
ISC_INVALID = {"EXPIRED"}
PLANNER_ALLOWED = {"TRACKING", "PREDICTING"}
RANK = {
    "TRACKING": 0,
    "PREDICTING": 1,
    "COASTING": 1,
    "PREDICTING_DEGRADED": 2,
    "EXPIRED": 3,
    "LOST": 3,
}


def published(mode):
    return str(getattr(mode, "value", mode))


def rank(mode):
    return RANK[published(mode)]


def classify(age, phase):
    return classify_tracking_mode(age <= 0.25, age, phase)


def test_published_names_are_in_system_vocabulary():
    names = {published(mode) for mode in TrackingMode}
    assert names <= ISC_VALID | ISC_INVALID, names


@pytest.mark.parametrize(
    "phase,age,expected",
    [
        ("FOLLOW", 0.05, "TRACKING"),
        ("FOLLOW", 0.8, "PREDICTING"),
        ("FOLLOW", 1.5, "PREDICTING_DEGRADED"),
        ("FOLLOW", 2.5, "EXPIRED"),
        ("APPROACH", 0.05, "TRACKING"),
        ("APPROACH", 0.4, "PREDICTING"),
        ("APPROACH", 0.8, "PREDICTING_DEGRADED"),
        ("APPROACH", 1.2, "EXPIRED"),
    ],
)
def test_mode_matches_system_spec(phase, age, expected):
    assert published(classify(age, phase)) == expected


def test_planner_blocked_when_degraded_or_expired():
    assert published(classify(1.5, "FOLLOW")) not in PLANNER_ALLOWED
    assert published(classify(2.5, "FOLLOW")) not in PLANNER_ALLOWED


def test_expired_is_rejected_by_isc():
    assert published(classify(2.5, "FOLLOW")) in ISC_INVALID


@pytest.mark.parametrize("phase", ["FOLLOW", "APPROACH"])
def test_severity_never_decreases_with_age(phase):
    ranks = [rank(classify(i * 0.05, phase)) for i in range(80)]
    assert ranks == sorted(ranks)


@pytest.mark.parametrize("age", [0.3, 0.6, 0.9, 1.2, 1.8])
def test_approach_is_never_more_lenient_than_follow(age):
    follow = rank(classify_tracking_mode(False, age, "FOLLOW"))
    approach = rank(classify_tracking_mode(False, age, "APPROACH"))
    assert approach >= follow


@pytest.mark.parametrize("phase", ["FOLLOW", "APPROACH"])
def test_never_measured_is_worst(phase):
    mode = classify_tracking_mode(False, math.inf, phase)
    assert rank(mode) == max(RANK.values())