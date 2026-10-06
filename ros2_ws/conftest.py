import pytest

from ros2_ws.ros_utils import Rig, init_ros


@pytest.fixture
def make_rig():
    rigs = []

    def factory(node_cls, params=None):
        init_ros(params)
        rig = Rig(node_cls())
        rigs.append(rig)
        return rig

    yield factory
    for rig in rigs:
        rig.close()