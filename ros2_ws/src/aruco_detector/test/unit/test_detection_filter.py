from aruco_detector.detection_filter import filter_detections, is_far_enough, is_high_enough


def det(distance, z):
    return {'pose': {'distance': distance, 'z': z}}


def test_is_far_enough_inclusive():
    assert is_far_enough(det(1.0, 1.0), 1.0)
    assert not is_far_enough(det(0.99, 1.0), 1.0)


def test_is_high_enough_inclusive():
    assert is_high_enough(det(1.0, 0.5), 0.5)
    assert not is_high_enough(det(1.0, 0.49), 0.5)


def test_filter_keeps_all_with_zero_thresholds():
    items = [det(1.0, 1.0), det(2.0, 0.0)]
    assert filter_detections(items, 0.0, 0.0) == items


def test_filter_drops_negative_z_with_default_threshold():
    assert filter_detections([det(1.0, -0.1)], 0.0, 0.0) == []


def test_filter_applies_both_conditions():
    items = [det(0.5, 2.0), det(2.0, 0.2), det(2.0, 2.0)]
    assert filter_detections(items, 1.0, 1.0) == [items[2]]


def test_filter_empty_and_order_preserved():
    assert filter_detections([], 1.0, 1.0) == []
    items = [det(3.0, 3.0), det(2.0, 2.0), det(4.0, 4.0)]
    assert filter_detections(items, 0.0, 0.0) == items