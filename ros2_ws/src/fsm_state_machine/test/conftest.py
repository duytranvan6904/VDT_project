import os

# Cô lập khỏi hệ thống thật đang chạy trên cùng mạng (đặt TRƯỚC khi rclpy.init).
os.environ.setdefault("ROS_DOMAIN_ID", "87")
os.environ.setdefault("ROS_LOCALHOST_ONLY", "1")

import pytest  # noqa: E402
import rclpy  # noqa: E402

from fsm_harness import Harness  # noqa: E402


def pytest_configure(config):
    config.addinivalue_line("markers", "slow: kịch bản vòng kín chạy thời gian thực (> 10 s)")


@pytest.fixture(scope="session", autouse=True)
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def fsm():
    """fsm(params=None, pre_kill=False, snapshots=True) -> Harness đã chạy.

    Mỗi test dùng một fsm_node mới + một test-node mới (bắt buộc: `system/killed` là
    transient_local, publisher cũ còn sống sẽ latch `killed=true` sang test sau).
    """
    created = []

    def make(params=None, **kw):
        h = Harness(params, **kw)
        created.append(h)
        h.start()
        return h

    yield make
    for h in reversed(created):
        h.stop()